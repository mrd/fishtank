-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- "main" function, GLUT initialization and setup, and screen setup.

module Main where

import Data.IORef ( IORef, newIORef )
import System.Exit ( exitWith, ExitCode( .. ) )
import Graphics.UI.GLUT
import Control.Monad
import System.Random
import Data.List ( foldl' )
import qualified Data.Quaternion as Q
import Boids
import Util
import Vivarium
import Hier ( drawCompiled )

viewingDelta = 0.5
frameDelay = floor (1000.0 / 60.0)

main :: IO ()
main = do
  (progName, _args) <- getArgsAndInitialize
  printHelp
  initialDisplayMode $= [ DoubleBuffered, RGBMode, WithDepthBuffer ]
  initialWindowSize $= Size 640 480
  initialWindowPosition $= Position 100 100
  createWindow progName
  myInit

  -- make sure to start in a state without collisions  
  let loop = do
        state <- makeState0
        events <- getFlocks state >>= checkCollisions state
        if null events then return state else loop
  
  state0 <- loop
  
  case _args of
    "-m":n:_ -> do
      displayCallback $= testmodel (read n :: Int) state0
      idleCallback $= Just (postRedisplay Nothing)
    _      -> do
      displayCallback $= display state0
      addTimerCallback frameDelay $ computeFrame state0
  anaglyph state0 $= "-A" `elem` _args
  reshapeCallback $= Just reshape
  keyboardMouseCallback $= Just (keyboard state0)
  motionCallback $= Just (motion state0)
  attachMenu RightButton $ 
    Menu [ MenuEntry "Add Food Particle" (addFood state0)
         , MenuEntry "Kill a Boid" (randomChoice matingFlocks >>= \ f ->
                                      -- Randomly kill a boid, if that
                                      -- won't reduce the population
                                      -- below the minimum.
                                      flockByNum f state0 $~ \ fl ->
                                        let n = length (flockBoids fl) in
                                          if n <= fst (flockMinMax !! f) then
                                            fl
                                          else
                                            fl { flockBoids = tail (flockBoids fl) })
         , MenuEntry "Toggle Verbose Mode" (verbose state0 $~ not)
         , MenuEntry "Toggle Analgyphic Mode" (anaglyph state0 $~ not)
         , MenuEntry "Toggle Scatter Mode" (scatter state0 $~ not)
         , MenuEntry "Toggle Fog" (fog $~ (\ c -> if c == Enabled then Disabled else Enabled))
         , MenuEntry "Quit" (exitWith ExitSuccess) ]
  mainLoop

computeFrame :: State -> TimerCallback
computeFrame state = do
  vivariumUpdate state
  postRedisplay Nothing
  addTimerCallback frameDelay $ computeFrame state

zoomFactor = 0.25
  
keyboard :: State -> KeyboardMouseCallback
keyboard state (Char '\27') Down _ _ = exitWith ExitSuccess
keyboard state (Char 's') Down _ _ = scatter state $~ not
keyboard state (Char 'f') Down _ _ = addFood state
keyboard state (Char 'D') Down _ _ = dumpState state
keyboard state (MouseButton LeftButton) Down _ mpos = do
  lastMPos state $= mpos
  curButton state $= Just LeftButton
keyboard state (MouseButton WheelUp) Down _ mpos = do
  viewingS state $~ (* (1 + zoomFactor))
keyboard state (MouseButton WheelDown) Down _ mpos = do
  viewingS state $~ (* (1 - zoomFactor))
keyboard state _            _    _ _ = return ()

motion :: State -> MotionCallback
motion state (Position x y) = do
  cb <- get (curButton state)
  case cb of
    Just LeftButton -> do
      Position last_x last_y <- get (lastMPos state)
      let dx = fromIntegral $ x - last_x
      let dy = fromIntegral $ y - last_y
      let mag = sqrt (dx * dx + dy * dy)
      if mag == 0 then return () -- bogus mouse motion event
        else do
          let q = Q.angleAxis viewingDelta (dy, dx, 0)
          viewingQ state $~ (Q.normalize . Q.mul q)
          lastMPos state $= Position x y
          return ()
    _ -> return ()

reshape :: ReshapeCallback
reshape size = do
  viewport $= (Position 0 0, size)
  matrixMode $= Projection
  loadIdentity
  frustum (-1) 1 (-1) 1 1.0 100
  lookAt (Vertex3 0 0 (2.5+boundMaxZ)) (Vertex3 0 0 0) (Vector3 0 1 0)
  -- ortho (-0.5) 0.5 (-0.5) 0.5 (-0.5) 0.5
  -- scaled (1/5, 1/5, 1/5)
  matrixMode $= Modelview 0

eyeSep = 1 / 75                 -- arbitrary choice
display :: State -> DisplayCallback
display state = do
  ana <- get (anaglyph state)
  clear [ ColorBuffer, DepthBuffer ]
  loadIdentity   -- clear the matrix

  let common = do
          vs <- get (viewingS state)
          translated (0, 0, (2.5+boundMaxZ) - (2.5+boundMaxZ)/vs)
          vq <- get (viewingQ state)
          m  <- newMatrix ColumnMajor (Q.rowMajorElems vq) :: IO (GLmatrix GLdouble)
          multMatrix m
          position (Light 0) $= light0Position          
          vivariumDisplay state
  
  if ana 
    then do
      colorMask $= Color4 Enabled Disabled Disabled Enabled
      translated (-eyeSep, 0, 0)
    else return ()
    
  common
  
  if ana 
    then do
      flush
      clear [ DepthBuffer ]
      colorMask $= Color4 Disabled Disabled Enabled Enabled
      loadIdentity
      translated (eyeSep, 0, 0)
      common
      colorMask $= Color4 Enabled Enabled Enabled Enabled
    else return ()


  swapBuffers

testmodel :: Int -> State -> DisplayCallback
testmodel n state = do
  if n == 0 then exitWith (ExitFailure 1)
            else return ()
  clear [ ColorBuffer, DepthBuffer ]
  loadIdentity   -- clear the matrix
  vq <- get (viewingQ state)
  m  <- newMatrix ColumnMajor (Q.rowMajorElems vq) :: IO (GLmatrix GLdouble)
  multMatrix m
  scaled (5, 5, 5)
  vs <- get (viewingS state)
  scaled (vs, vs, vs)
  drawCompiled (boidByNum n state)
  swapBuffers

light0Position = Vertex4 0 (1.5*realToFrac boundMaxY) 0 0
light0Ambient = Color4 0.5 0.6 0.7 1
light0Diffuse = Color4 1 1 1 1
light0Specular = Color4 0.8 0.8 0.8 1
light0Direction = Normal3 0 (-1) 0

bgcolor = Color4 0 0.2 0.3 0

myInit :: IO ()
myInit = do
  matrixMode $= Modelview 0
  loadIdentity
  clearColor $= bgcolor
  shadeModel $= Smooth
  polygonMode $= (Fill, Fill)   -- fill front and back
  colorMaterial $= Just (Front, AmbientAndDiffuse)
  lighting $= Enabled
  light (Light 0) $= Enabled
  ambient (Light 0) $= light0Ambient
  diffuse (Light 0) $= light0Diffuse
  specular (Light 0) $= light0Specular
  spotDirection (Light 0) $= light0Direction
  normalize $= Enabled
  depthFunc $= Just Less
  texture Texture2D $= Enabled  
  fog $= Enabled
  fogColor $= bgcolor
  fogMode $= Linear 0 (1.5*realToFrac (boundMaxY - boundMinZ))
  hint Fog $= Nicest


printHelp = do
  putStrLn "What you see:"
  putStrLn "  Fish schools and dolphins using the 'Boids' algorithm to simulate"
  putStrLn "  flocking and predator/prey relationships.  Collisions are detected"
  putStrLn "  and potentially interpreted as feeding or mating."
  putStrLn "Usage:"
  putStrLn "  f: randomly add food particle"
  putStrLn "  s: scatter-mode -- fish flocks move apart"
  putStrLn "  D: dump state to stdout"
  putStrLn "  mousedrag: re-orient view"
  putStrLn "  mousewheel: zoom in/out"
  putStrLn "  rightclick: show menu"
  putStrLn "  ESC: quit"

dumpState state = do
  flocks <- getFlocks state
  forM_ [1 .. maxFlockNum] $ \ f -> do
    putStrLn ("FLOCK"++show f++":")
    mapM_ print (flocks !! f)
  viewQ <- get (viewingQ state)
  vs    <- get (viewingS state)
  putStrLn ("viewQ="++show viewQ)
  putStrLn ("viewS="++show vs)
  return ()
