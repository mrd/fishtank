-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Generic 'Boids' algorithm implementation.  'Boids' are objects with
-- position and velocity that decide the velocity on each frame by
-- adding up the result of a set of simple rules.
--
-- Default rules are provided for easy instantiation of common
-- behaviors: flocking towards average position, avoidance of fellow
-- boids, tendency towards average velocity, and avoidance of walls.

module Boids ( Flock(..), makeFlock, flockUpdate, normalizeVelocity
             , Boid(..), makeBoid
             , defaultRule1, defaultRule2, defaultRule3, defaultRule4 ) where

import Data.IORef ( IORef, newIORef )
import System.Exit ( exitWith, ExitCode(ExitSuccess) )
import Graphics.UI.GLUT
import Control.Monad
import qualified Quat as Q
import GLDouble
import Util

-- Rule 1: steer towards average position
-- Rule 2: collision avoidance
-- Rule 3: average velocity
-- Rule 4: avoid walls

type Bounds = (GLdouble, GLdouble)

data (Eq a, Ord a) => 
     Flock a = F { flockSize   :: Int
                 , flockBoids  :: [Boid a] }

makeFlock bs = 
  F { flockSize   = length bs
    , flockBoids  = bs }

data (Show a, Eq a, Ord a) =>
     Boid a = B { i         :: Int
                , radius    :: GLdouble
                , pos       :: Vector3d
                , vel       :: Vector3d
                , speeds    :: Bounds
                , quat      :: Q.Quat
                , otherData :: a }
  deriving (Show, Eq, Ord)

makeBoid i r p v s o = B { i = i, radius = r, pos = p, vel = v
                         , speeds = s, otherData = o
                         , quat = Q.lookAt (1, 0, 0) (vx, vy, vz) (0, 1, 0) }
  where Vector3 vx vy vz = v

flockUpdate rules fl = fl { flockBoids = map doBoid bs }
  where
    bs = flockBoids fl
    n = fromIntegral $ length bs
    doBoid b = b { vel = limitVelocity (speeds b) vel', quat = q' }
      where
        vel' = vel b `vecAdd` vecSum (map (\ r -> r b (others, c, v)) rules)
        others = filter (/= b) bs
        c = (1/(n - 1)) `vecScale` vecSum (map pos others)
        v = (1/(n - 1)) `vecScale` vecSum (map vel others)
        Vector3 vx vy vz = vel'
        angle = angleBetween (Vector3 0 1 0) vel'
        q  = quat b
        -- This hack side-steps the problem of twirling boids when the
        -- velocity angle is too close to vertical.
        q' = if angle < 5 || angle > 175 then q 
             else Q.lookAt (1, 0, 0) (vx, vy, vz) (0, 1, 0)

-- Commented out SLERP: linear interpretation of rotation because it
-- made it too hard for boids to turn around, for now.
--
--        q'' = Q.slerp q q' 0.15
--        vel'' = Vector3 x y z
--          where 
--            (x, y, z) = 
--              Q.toVec (Q.reciprocal q'' `Q.mul` Q.fromVec (vx, vy, vz) `Q.mul` q'')

defaultRule1 f b (others, c, v) = vecScale f (c `vecSub` pos b)
defaultRule2 f b (others, c, v) = vecSum . flip map others $ \ b' ->
          if magnitude (pos b' `vecSub` pos b) < radius b then
            vecScale f (pos b `vecSub` pos b')
          else Vector3 0 0 0
defaultRule3 f b (others, c, v) = vecScale f (v `vecSub` vel b)
defaultRule4 f bnds b (others, c, v) = 
  vecSum [ if x < minX + r then Vector3 (abs vx+f) vy vz    else Vector3 0 0 0
         , if x > maxX - r then Vector3 (-(abs vx+f)) vy vz else Vector3 0 0 0 
         , if y < minY + r then Vector3 vx (abs vy+f) vz    else Vector3 0 0 0
         , if y > maxY - r then Vector3 vx (-(abs vy+f)) vz else Vector3 0 0 0
         , if z < minZ + r then Vector3 vx vy (abs vz+f)    else Vector3 0 0 0
         , if z > maxZ - r then Vector3 vx vy (-(abs vz+f)) else Vector3 0 0 0 ]
  where
    ((minX, maxX), (minY, maxY), (minZ, maxZ)) = bnds
    r = 2 * radius b
    Vector3 x y z = pos b
    Vector3 vx vy vz = vel b

normalizeVelocity maxS v = (maxS / (magnitude v)) `vecScale` v

limitVelocity (minS, maxS) v 
  | speed > maxS = (maxS / (magnitude v)) `vecScale` v
  | speed < minS = (minS / (magnitude v)) `vecScale` v
  | otherwise            = v
  where 
    speed = magnitude v

