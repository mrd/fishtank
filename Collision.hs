-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Generic collision detection module.  "detect" expects a
-- transformation to be provided that squeezes your world into a 1x1x1
-- box centered around the origin.  Then you supply a list of OpenGL
-- commands to draw your objects in your world.  "detect" will tell
-- you which ones are potentially involved in a collision.
--
-- This algorithm is based on the CULLIDE paper: 
--
--   Govindaraju, et al. CULLIDE: Interactive Collision Detection
--   Between Complex Models in Large Environments Using Graphics
--   Hardware, ACM SIGGRAPH/Eurographics Graphics Hardware, 2003.
--
-- The idea is very simple however: use one of the occlusion-query
-- OpenGL extensions to detect pixel overlap in an orthographic
-- projection.  Do this from three orthogonal directions, and you will
-- know which objects are always overlapped, hence: potential
-- collision.

module Collision ( detect ) where

import Graphics.Rendering.OpenGL.GL 
  hiding ( GLuint, queryResultAvailable, queryResult )
import Graphics.Rendering.OpenGL.GLU
import Graphics.Rendering.OpenGL.Raw
import Graphics.Rendering.OpenGL.Raw.ARB.OcclusionQuery
import Foreign.Ptr
import Foreign.Storable
import Foreign.ForeignPtr
import Data.Array.Storable
import Data.Array.IO
import Control.Monad

-- The occlusion_query_ARB extension is only available through the
-- OpenGL RAW interface, so we need to write some glue code to make it
-- nicer.  This is essentially pointer-manipulation in Haskell.

genQueries :: Int -> IO [GLuint]
genQueries n 
  | n >= 1 = do
    a <- newArray (0, n-1) 0 :: IO (StorableArray Int GLuint)
    withStorableArray a $ \ ptr ->
      glGenQueries (fromIntegral n) ptr
    flip mapM [0 .. (n-1)] $ \ i ->
      readArray a i
  | otherwise = return []

deleteQueries :: [GLuint] -> IO ()
deleteQueries l = do
  let n = length l
  a <- newListArray (0, n-1) l :: IO (StorableArray Int GLuint)
  withStorableArray a $ \ ptr ->
    glDeleteQueries (fromIntegral n) ptr

querySupport :: IO Bool
querySupport = do
  x <- mallocForeignPtr
  v <- withForeignPtr x $ \ p -> do
    glGetQueryiv gl_SAMPLES_PASSED gl_QUERY_COUNTER_BITS p
    v <- peek p
    return $ v /= 0
  finalizeForeignPtr x
  return v

queryResultAvailable :: GLuint -> IO Bool
queryResultAvailable i = do
  x <- mallocForeignPtr
  v <- withForeignPtr x $ \ p -> do
    glGetQueryObjectiv i gl_QUERY_RESULT_AVAILABLE p
    v <- peek p
    return $ v /= 0
  finalizeForeignPtr x
  return v

queryResult :: GLuint -> IO GLuint
queryResult i = do
  x <- mallocForeignPtr
  r <- withForeignPtr x $ \ p -> do
    glGetQueryObjectuiv i gl_QUERY_RESULT p
    peek p
  finalizeForeignPtr x
  return r

-- collision detection
--
-- transform: action which transforms the viewing volume into a cube
-- with side-length 1 centered on the origin.
--
-- actions: list of actions that draw the objects

detect transform actions = do
  let n = length actions
  
  -- generate query indices
  indices <- genQueries n
  
  -- save old settings
  glPushMatrix
  mmode <- get matrixMode
  tex1d <- get (texture Texture1D)
  tex2d <- get (texture Texture2D)
  tex3d <- get (texture Texture3D)
  light <- get lighting
  cmask <- get colorMask
  dmask <- get depthMask
  dfunc <- get depthFunc
  
  -- disable color buffer temporarily
  colorMask $= Color4 Disabled Disabled Disabled Disabled
  depthMask $= Enabled
  -- disable texturing
  texture Texture1D $= Disabled
  texture Texture2D $= Disabled
  texture Texture3D $= Disabled
  -- disable lighting
  lighting $= Disabled

  testFPtr   <- mallocForeignPtr
  sampleFPtr <- mallocForeignPtr

  samples <- newArray (0, n-1) 0 :: IO (IOUArray Int Int)

  matrixMode $= Projection
  glPushMatrix

  axis transform
       (rotate (-90) (Vector3 0 1 (0 :: Double))) -- X axis
       n actions indices testFPtr sampleFPtr samples
  axis transform
       (rotate ( 90) (Vector3 1 0 (0 :: Double))) -- Y axis
       n actions indices testFPtr sampleFPtr samples
  axis transform
       (return ())                                -- Z axis
       n actions indices testFPtr sampleFPtr samples
  
  
  -- cleanup
  finalizeForeignPtr sampleFPtr
  finalizeForeignPtr testFPtr
  colorMask $= cmask
  depthMask $= dmask
  depthFunc $= dfunc
  deleteQueries indices
  texture Texture1D $= tex1d
  texture Texture2D $= tex2d
  texture Texture3D $= tex3d
  lighting $= light
  matrixMode $= Projection
  glPopMatrix
  matrixMode $= mmode
  glPopMatrix

  map (>= 3) `fmap` getElems samples

axis transform orient n actions indices testFPtr sampleFPtr samples = do
  -- setup orthographic projection  
  matrixMode $= Projection
  loadIdentity
  ortho (-0.5) (0.5) (-0.5) (0.5) (-0.5) (0.5)
  orient
  transform
  matrixMode $= Modelview 0

  -- Do a forward pass,    
  pass False n actions indices testFPtr sampleFPtr samples
  -- then a backward pass.
  pass True  n actions indices testFPtr sampleFPtr samples


pass rev n actions indices testFPtr sampleFPtr samples = do
    
  clear [ DepthBuffer ]
  loadIdentity
  -- send the queries to the accelerator
  let actions' = if rev then reverse actions else actions
  flip mapM_ (zip actions' indices) $ \ (a, i) -> do
    depthMask $= Disabled
    depthFunc $= Just Gequal
    glBeginQuery gl_SAMPLES_PASSED i
    a
    glEndQuery gl_SAMPLES_PASSED

    depthMask $= Enabled
    depthFunc $= Just Lequal
    a

  glFlush

  -- query the completion of 3/4*n because this gives us some time to
  -- start receiving results before the graphics accelerator has
  -- finished processing them all, thus preventing it from going idle.
  let i = (3 * n) `div` 4

  let loop p = do glGetQueryObjectiv (fromIntegral i) gl_QUERY_RESULT_AVAILABLE p
                  v <- peek p
                  if v /= 0 then return () else loop p
  withForeignPtr testFPtr loop

  -- retrieve sample data
  withForeignPtr sampleFPtr $ \ samplePtr -> do  
    flip mapM_ (zip indices [0 .. (n-1)]) $ \ (i, j) -> do
      glGetQueryObjectuiv i gl_QUERY_RESULT samplePtr
      let j' = if rev then n - 1 - j else j
      s  <- readArray samples j'
      s' <- fromIntegral `fmap` peek samplePtr
      writeArray samples j' (s + (if s' > 0 then 1 else 0))
  return ()
