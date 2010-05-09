-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Hierarchical object construction: a tree of OpenGL commands with
-- quaternions and translations specified for each node.  Compiles
-- into display lists for eventual display.

module Hier ( Hier(..), Compiled, drawCompiled, compileHier, makeHier
            , getCompiledPart, drawCompiledPart, drawModifiedCompiled ) where
import Graphics.Rendering.OpenGL.GL
import Data.IORef ( IORef, newIORef )
import Util
import Data.Maybe
import Data.List ( lookup )
import Control.Monad
import qualified Quat as Q

type Triple = (GLdouble, GLdouble, GLdouble)

data Hier a = H { hierDraw     :: IO ()
                , hierTrans    :: Triple
                , hierQuat     :: Q.Quat
                , hierData     :: a
                , hierChildren :: [Hier a] }

newtype Compiled a = C (DisplayList, IORef Triple, IORef Q.Quat, IORef a, [Compiled a])

drawCompiled (C (dl, r_t, r_q, _, cs)) = 
  preservingMatrix $ do
    t <- get r_t
    q <- get r_q
    translated t
    m <- Q.toMatrix q :: IO (GLmatrix GLdouble)
    multMatrix m
    callList dl
    mapM_ drawCompiled cs

drawModifiedCompiled changes (C (dl, r_t, r_q, r_x, cs)) = 
  preservingMatrix $ do
    t <- get r_t
    q <- get r_q
    x <- get r_x
    translated t
    let q' = case lookup x changes of
               Just r  -> r `Q.mul` q
               Nothing -> q
    m <- Q.toMatrix q' :: IO (GLmatrix GLdouble)
    multMatrix m
    callList dl
    mapM_ (drawModifiedCompiled changes) cs

drawCompiledPart a (C (dl, r_t, r_q, r_x, cs)) = 
  preservingMatrix $ do
    t <- get r_t
    q <- get r_q
    translated t
    m <- Q.toMatrix q :: IO (GLmatrix GLdouble)
    multMatrix m
    x <- get r_x
    if x == a then callList dl else return ()
    mapM_ (drawCompiledPart a) cs

makeHier draw t q dat cs = 
  H { hierDraw     = draw
    , hierTrans    = t
    , hierQuat     = q
    , hierData     = dat
    , hierChildren = cs }
    
compileHier (H { hierDraw     = dr
               , hierTrans    = t
               , hierQuat     = q
               , hierData     = dat
               , hierChildren = hs }) = do
  cs <- mapM compileHier hs
  dl <- defineNewList Compile dr
  r_t <- newIORef t
  r_q <- newIORef q
  r_d <- newIORef dat
  return $ C (dl, r_t, r_q, r_d, cs)

getCompiledPart a (c@(C (dl, r_t, r_q, r_x, cs))) = do
  x <- get r_x
  if x == a 
    then return (Just c)
    else do
      let loop [] = return Nothing
          loop (c:cs) = do
            m_c' <- getCompiledPart a c
            case m_c' of
              Just c' -> return (Just c')
              Nothing -> loop cs
      loop cs
