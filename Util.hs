-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Assorted convenience utility functions.

module Util where
import Graphics.Rendering.OpenGL.GL
import Data.List (foldl')
import System.Random

type Vector3d = Vector3 GLdouble

uncurry3 f (a, b, c) = f a b c
color3d'    = color     :: Color3 GLdouble -> IO ()
color3d     = color3d' . uncurry3 Color3 
scaled'     = scale     :: GLdouble -> GLdouble -> GLdouble -> IO ()
scaled      = uncurry3 scaled'
vertex3d'   = vertex    :: Vertex3 GLdouble -> IO ()
vertex3d    = vertex3d' . uncurry3 Vertex3
normal3d'   = normal    :: Normal3 GLdouble -> IO ()
normal3d    = normal3d' . uncurry3 Normal3
rotated'    = rotate    :: GLdouble -> Vector3d -> IO ()
rotated a   = rotated' a . uncurry3 Vector3
translated' = translate :: Vector3d -> IO ()
translated  = translated' . uncurry3 Vector3
texCoord2d' = texCoord  :: TexCoord2 GLdouble -> IO ()
texCoord2d  = texCoord2d' . uncurry TexCoord2

r2d a = a * 180 / pi            -- radian to degree

-- some vector ops  
magnitude (Vector3 x y z) = sqrt (x*x + y*y + z*z)
s `vecScale` Vector3 x y z = Vector3 (s*x) (s*y) (s*z)
Vector3 x1 y1 z1 `vecAdd` Vector3 x2 y2 z2 = Vector3 (x1+x2) (y1+y2) (z1+z2)
Vector3 x1 y1 z1 `vecSub` Vector3 x2 y2 z2 = Vector3 (x1-x2) (y1-y2) (z1-z2)
vecReciprocal (Vector3 x y z) = Vector3 (1/x) (1/y) (1/z)
vecSum l = foldl' vecAdd (Vector3 0 0 0) l
(Vector3 x1 y1 z1) `dotP` (Vector3 x2 y2 z2) = x1*x2 + y1*y2 + z1*z2
(Vector3 x1 y1 z1) `crossP` (Vector3 x2 y2 z2) = 
    Vector3 (y1*z2 - z1*y2) (z1*x2 - z2*x1) (x1*y2 - x2*y1)
projectV v n = Vector3 ux uy uz
  where
    Vector3 vx vy vz = v
    Vector3 nx ny nz = n
    nmag          = sqrt (nx*nx + ny*ny + nz*nz)
    (nx',ny',nz') = (nx / nmag, ny / nmag, nz / nmag)
    dp            = vx * nx' + vy * ny' + vz * nz'
    ux            = vx - nx' * dp
    uy            = vy - ny' * dp
    uz            = vz - nz' * dp
angleBetween u v = r2d (acos ((u `dotP` v) / (magnitude u * magnitude v)))

randomVector3d :: IO Vector3d
randomVector3d = do
  x <- randomRIO (-1, 1)
  y <- randomRIO (-1, 1)
  z <- randomRIO (-1, 1)
  return $ Vector3 x y z

randomChoice :: [a] -> IO a
randomChoice [] = error "randomChoice: empty list"
randomChoice l = (l !!) `fmap` randomRIO (0, length l - 1)
