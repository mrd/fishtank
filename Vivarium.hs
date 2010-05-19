-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Project-specific design of boids, tank, behavior are here.

module Vivarium where
import Data.IORef ( IORef, newIORef )
import Graphics.UI.GLUT
import Util
import Boids
import qualified Data.Quaternion as Q
import Control.Monad
import System.Random
import Debug.Trace
import GLDouble ((~==), dmax, epsilon)
import Hier
import JpegTexture
import Graphics.Collision.Cullide
import Data.List

--------------------------------------------------
-- Begin global parameters

-- Boundaries of tank
boundMinX = -2
boundMaxX = 2
boundMinY = -2
boundMaxY = 2
boundMinZ = -2
boundMaxZ = 2

-- Various Boids-algorithm rule parameters
rule1Factor = 0.0005            -- steer towards average center
rule2Factor = 0.10              -- avoid each other
rule3Factor = 0.125             -- maintain average velocity
rule4Factor = 0.0005            -- avoid walls
avoidanceFactor = 0.00005       -- for avoiding predators
chaseFactor = 0.00005           -- for chasing prey
foodChaseFactor = 20 * chaseFactor -- for chasing food

boidMinScale = 0.05             -- tiniest boid scale
scaleFactor = 0.002             -- how fast boids grow with age
speedFactor = 0.0005            -- how fast boids slow down with age

minTailAngle = -30              -- tail/fin animation min
maxTailAngle =  30              -- tail/fin animation max
tailAFactor  = 500              -- how much faster than swimspeed does tail move

maxFlockNum = length boidCompileFs -- number of flocks

--------------------------------------------------
-- Begin parameters of flocks and boids

-- List of initialization functions
boidInit n = 
  [ undefined, initFishBoid 1, initDolphinBoid 2, initFishBoid 3
  , initFoodBoid 4, initFishBoid 5, initEggBoid 6, initEggBoid 7
  , initEggBoid 8, initRockBoid 9, initRockBoid 10 ]
  !! n

-- List of Hier compile functions
boidCompileFs = 
  [ compileFish (boidsW !! 1) (boidsH !! 1) "boid1body.jpg" (0.2, 0.6, 0.6)
  , compileDolphin (boidsW !! 2) (boidsH !! 2) (0.5, 0.5, 0.5)
  , compileFish (boidsW !! 3) (boidsH !! 3) "boid3body.jpg" (0.6, 0.6, 0.2)
  , compileFood (boidsW !! 4) (boidsH !! 4) (0, 1, 0) 
  , compileFish (boidsW !! 5) (boidsH !! 5) "boid5body.jpg" (0.9, 0.3, 0.3) 
  , compileEgg (boidsW !! 6) (boidsH !! 6) (0.2, 0.6, 0.6)
  , compileEgg (boidsW !! 7) (boidsH !! 7) (0.6, 0.6, 0.2)
  , compileEgg (boidsW !! 8) (boidsH !! 8) (0.9, 0.3, 0.3) 
  , compileRock (boidsW !! 9) (boidsH !! 9) "boid9body.jpg" (0.3, 0.3, 0.3) 
  , compileRock (boidsW !! 10) (boidsH !! 10) "boid10body.jpg" (0.3, 0.3, 0.3) ]

-- List of Boids-rule generating functions
flockRules f = 
  [ undefined, preyFlockRules, predatorFlockRules, preyFlockRules
  , foodFlockRules 4, preyFlockRules, eggFlockRules 6, eggFlockRules 7, eggFlockRules 8
  , rockFlockRules 9, rockFlockRules 10 ] 
  !! f

-- List of per-Boid event insertion functions
boidEventF =
  [ undefined, noEventF, noEventF, noEventF, noEventF, noEventF
  , eggEventF 1, eggEventF 3, eggEventF 5, noEventF, noEventF ]

-- Min/Max population of each flock
flockMinMax = [ (0, 0), (3, 10), (2, 5), (3, 10), (0, 5), (3, 10)
              , (0, 0), (0, 0), (0, 0), (0, 3), (0, 3) ]

-- Bounding sphere radius of each flock's boids  
boidsRadius = [0, 0.08, 0.16, 0.08, 0.02, 0.08, 0.015, 0.015, 0.015, 0.20, 0.20]

-- Width value of each flock's boids
boidsW = [0, 0.05, 0.07, 0.06, 0.015, 0.06, 0.01, 0.01, 0.01, 0.20, 0.20]

-- Height value of each flock's boids
boidsH = [0, 0.07, 0.14, 0.06, 0.015, 0.04, 0.01, 0.01, 0.01, 0.20, 0.20]

-- Min/Max speeds for each flock
speedsMinMax = [ (0, 0), (0.005, 0.02), (0.01, 0.02), (0.005, 0.02), (0.005, 0.005)
               , (0.005, 0.02), (0.005, 0.005), (0.005, 0.005)
               , (0.005, 0.005), (0.005, 0.005), (0.005, 0.005) ]

-- Which flocks are predators
predatorFlocks = [2]

-- Which flocks are prey
preyFlocks = [1,3,5]

-- Which flocks are just food
foodFlocks = [4]

-- Which flocks can eat food
hungryFlocks = [1,2,3,5]

-- Which flocks can mate
matingFlocks = [1,3,5]
-- The associated mating function (returns True if decide to allow mating)
boidMatingF = [ undefined, scaleWeightedMate 0.5
              , neverMate, scaleWeightedMate 0.5
              , neverMate, scaleWeightedMate 0.5
              , neverMate, neverMate, neverMate
              , neverMate, neverMate ]

-- Association list mapping a flock to its egg flock
eggFlocks = [(1,6),(3,7),(5,8)]

--------------------------------------------------
-- Helper functions  

initFishBoid f a = do
  let r = boidsRadius !! f
  x <- randomRIO (boundMinX + r, boundMaxX - r)
  y <- randomRIO (boundMinY + r, boundMaxY - r)
  z <- randomRIO (boundMinZ + r, boundMaxZ - r)
  let (minS, maxS) = (iterate maxSpeedF (fst (speedsMinMax !! f), snd (speedsMinMax !! f))) !! a
  v <- normalizeVelocity maxS `liftM` randomVector3d
  let s = (iterate scaleF boidMinScale) !! a
  let tailA = initTailA (magnitude v) (0, 1) a
  return ((x,y,z), (minS, maxS), v, s, tailA, (0, 1, 0))

initDolphinBoid f a = do
  let r = boidsRadius !! f
  x <- randomRIO (boundMinX + r, boundMaxX - r)
  y <- randomRIO (boundMinY + r, boundMaxY - r)
  z <- randomRIO (boundMinZ + r, boundMaxZ - r)
  let (minS, maxS) = (iterate maxSpeedF (fst (speedsMinMax !! f), snd (speedsMinMax !! f))) !! a
  v <- normalizeVelocity maxS `liftM` randomVector3d
  let s = (iterate scaleF boidMinScale) !! a
  let tailA = initTailA (magnitude v) (0, 1) a
  return ((x,y,z), (minS, maxS), v, s, tailA, (0, 0, 1))

initFoodBoid f a = do
  let r = boidsRadius !! f
  x <- randomRIO (boundMinX + r, boundMaxX - r)
  y <- randomRIO (boundMinY + r, boundMaxY - r)
  z <- randomRIO (boundMinZ + r, boundMaxZ - r)
  let v = Vector3 0 (-fst (speedsMinMax !! f)) 0
  return ((x,y,z), (fst (speedsMinMax !! f), fst (speedsMinMax !! f)), v, 1, (0, 0), (0, 0, 0))

initEggBoid f a = do
  let r = boidsRadius !! f
  x <- randomRIO (boundMinX + r, boundMaxX - r)
  y <- randomRIO (boundMinY + r, boundMaxY - r)
  z <- randomRIO (boundMinZ + r, boundMaxZ - r)
  let v = Vector3 0 (-fst (speedsMinMax !! f)) 0
  return ((x,y,z), (fst (speedsMinMax !! f), fst (speedsMinMax !! f)), v, 1, (0, 0), (0, 0, 0))

initRockBoid f a = do
  let r = boidsRadius !! f
  x <- randomRIO (boundMinX + r, boundMaxX - r)
  let y = boundMinY + r
  z <- randomRIO (boundMinZ + r, boundMaxZ - r)
  let v = Vector3 0 (-fst (speedsMinMax !! f)) 0
  return ((x,y,z), (fst (speedsMinMax !! f), fst (speedsMinMax !! f)), v, 1, (0, 0), (0, 0, 0))

preyFlockRules state = do
  scat <- get (scatter state)   -- whether scatter is toggled
  flocks <- getFlocks state
  return ( map (avoid avoidanceFactor) (predatorFlocks >>= (flocks !!)) ++
           map (chase foodChaseFactor) (foodFlocks >>= (flocks !!)) ++
           [ rule1 ((if scat then -1 else 1) * rule1Factor)
           , rule2 rule2Factor
           , rule3 rule3Factor
           , rule4 rule4Factor ] )

predatorFlockRules state = do
  flocks <- getFlocks state
  return ([ rule2 rule2Factor, rule4 rule4Factor ] ++
          map (chase chaseFactor) (preyFlocks >>= (flocks !!)))

foodFlockRules f state = return [constRule (Vector3 0 (-fst (speedsMinMax !! f)) 0)]

eggFlockRules f state = return [constRule (Vector3 0 (-fst (speedsMinMax !! f)) 0)]

rockFlockRules f state = return [constRule (Vector3 0 (-fst (speedsMinMax !! f)) 0)]

eggHatchAge = 250
eggEventF species b events 
  | biAge (otherData b) > eggHatchAge = events $~ (Hatch (biFlock (otherData b), i b) species :)
  | otherwise = return ()

neverMate _ = return False
alwaysMate _ = return True
scaleWeightedMate factor (b1, b2) = do
  x <- randomRIO (0.0, 1.0)
  return (x <= factor * s1 * s2)
  where
    s1 = biScale (otherData b1)
    s2 = biScale (otherData b2)

compileFish w h tex color = loadJPEGTexture tex >>= (compileHier . body)
  where
    qstyle = (QuadricStyle (Just Smooth) GenerateTextureCoordinates Outside FillStyle)
    nose = makeHier (do color3d color
                        preservingMatrix $ do
                          rotated 90 (0, 1, 0) 
                          renderQuadric qstyle $ Cylinder (w*0.1) (w*0.1) (w*0.2) 10 5)
                    (0.9*w, 0, 0) Q.zero (Nothing) []
    body tex = makeHier (do color3d (1,1,1)
                            textureBinding Texture2D $= Just tex
                            preservingMatrix $ do
                              scaled (1.0, h/w, 0.5)
                              renderQuadric qstyle $ Sphere w 18 9
                            textureBinding Texture2D $= Nothing)
                        (0, 0, 0)
                        Q.zero (Nothing) [nose, tail, finR, finL, dors, eyeR, eyeL]
    tail = makeHier (do color3d color
                        preservingMatrix $ do
                          rotated 90 (1,0,0)
                          preservingMatrix $ do
                            translated (-0.1*w, 0, 0)
                            rotated (-25) (0,1,0)
                            scaled (1, 0.5, h/w)
                            renderQuadric qstyle $ Cylinder (0.3*min w h) 0 (max w h*0.65) 10 5
                          preservingMatrix $ do
                            translated (-0.1*w, 0, 0)
                            rotated (-155) (0,1,0)
                            scaled (1, 0.5, h/w)
                            renderQuadric qstyle $ Cylinder (0.3*min w h) 0 (max w h*0.65) 10 5)
                    (-0.9*w, 0, 0) Q.zero (Just Tail) []
    finR = makeHier (do color3d color
                        renderPrimitive Triangles $ do
                          normal3d (0, 0, 1)
                          vertex3d (w/2, 0, 0)
                          vertex3d (0, 0, 0)
                          vertex3d (0, -h/2, 0))
                    (-0.1*w, -h/2, w*0.4)
                    (Q.angleAxis (-45) (1,0,0)) (Just RFin) []
    finL = makeHier (do color3d color
                        renderPrimitive Triangles $ do
                          normal3d (0, 0, -1)
                          vertex3d (w/2, 0, 0)
                          vertex3d (0, 0, 0)
                          vertex3d (0, -h/2, 0))
                    (-0.1*w, -h/2, -w*0.4)
                    (Q.angleAxis (45) (1,0,0)) (Just LFin) []
    dors = makeHier (do color3d color
                        preservingMatrix $ do
                          scaled (1, -1, 1)
                          renderPrimitive Triangles $ do
                            normal3d (0, 0, 1)
                            vertex3d (w/2, 0, 0)
                            vertex3d (0, 0, 0)
                            vertex3d (0, -h/2, 0))
                    (-0.1*w, 0.9*h, 0)
                    (Q.zero) (Just Dorsal) []
    eyeR = makeHier (do color3d (1,1,1)
                        renderQuadric qstyle $ Sphere (w*0.15) 10 5)
                    (w*0.75, w*0.4, w*0.25)
                    Q.zero (Nothing) [pupl]
    eyeL = makeHier (do color3d (1,1,1)
                        renderQuadric qstyle $ Sphere (w*0.15) 10 5)
                    (w*0.75, w*0.4, -w*0.25)
                    (Q.angleAxis 90 (0,1,0)) (Nothing) [pupl]
    pupl = makeHier (do color3d (0,0,0)
                        renderQuadric qstyle $ Sphere (w*0.05) 10 5)
                    (w*0.08,0,w*0.08)
                    Q.zero (Nothing) []

compileDolphin w h color = compileHier body
  where
    qstyle = (QuadricStyle (Just Smooth) GenerateTextureCoordinates Outside FillStyle)
    nose = makeHier (do color3d color
                        preservingMatrix $ do
                          rotated 90 (0, 1, 0)
                          renderQuadric qstyle $ Cylinder (w*0.3) (w*0.2) (1.25*h) 10 5)
                    (w*0.6, 0, 0) Q.zero (Just Jaws) []
    body = makeHier (do color3d color
                        preservingMatrix $ do
                          scaled (2, 1, 0.5)
                          renderQuadric qstyle $ Sphere w 18 9)
                    (0, 0, 0)
                    Q.zero (Nothing) [nose, tail, finR, finL, dors, eyeR, eyeL]
    tail = makeHier (do color3d color
                        preservingMatrix $ do
                          translated (-0.1*h, 0, 0)
                          rotated (-25) (0,1,0)
                          scaled (1, 0.4, 1)
                          renderQuadric qstyle $ Cylinder (0.5*w) 0 (0.8*h) 10 5
                        preservingMatrix $ do
                          translated (-0.1*h, 0, 0)
                          rotated (-155) (0,1,0)
                          scaled (1, 0.4, 1)
                          renderQuadric qstyle $ Cylinder (0.5*w) 0 (0.8*h) 10 5)
                    (-0.9*h, 0, 0) Q.zero (Just Tail) []
    finR = makeHier (do color3d color
                        renderPrimitive Triangles $ do
                          normal3d (0, 0, 1)
                          vertex3d (w/2, 0, 0)
                          vertex3d (0, 0, 0)
                          vertex3d (0, -h/2, 0))
                    (0, -w/2, w*0.4)
                    (Q.angleAxis (-45) (1,0,0)) (Just RFin) []
    finL = makeHier (do color3d color
                        renderPrimitive Triangles $ do
                          normal3d (0, 0, -1)
                          vertex3d (w/2, 0, 0)
                          vertex3d (0, 0, 0)
                          vertex3d (0, -h/2, 0))
                    (0, -w/2, -w*0.4)
                    (Q.angleAxis (45) (1,0,0)) (Just LFin) []
    dors = makeHier (do color3d color
                        preservingMatrix $ do
                          scaled (1, -1, 1)
                          renderPrimitive Triangles $ do
                            normal3d (0, 0, 1)
                            vertex3d (w/2, 0, 0)
                            vertex3d (0, 0, 0)
                            vertex3d (0, -h/2, 0))
                    (0, 0.9*w, 0)
                    (Q.zero) (Just Dorsal) []
    eyeR = makeHier (do color3d (1,1,1)
                        renderQuadric qstyle $ Sphere (w/5) 10 5)
                    (w*1.2, w*0.4, w*0.3)
                    Q.zero (Nothing) [pupl]
    eyeL = makeHier (do color3d (1,1,1)
                        renderQuadric qstyle $ Sphere (w/5) 10 5)
                    (w*1.2, w*0.4, -w*0.3)
                    (Q.angleAxis 90 (0,1,0)) (Nothing) [pupl]
    pupl = makeHier (do color3d (0,0,0)
                        renderQuadric qstyle $ Sphere (w/8) 10 5)
                    (w/8,0,w/8)
                    Q.zero (Nothing) []

compileFood w h color = compileHier food
  where
    qstyle = (QuadricStyle (Just Smooth) GenerateTextureCoordinates Outside FillStyle)
    food = makeHier (do color3d color
                        renderQuadric qstyle $ Sphere w 10 5)
                    (0, 0, 0)
                    (Q.zero) (Nothing) []

compileEgg w h color = compileHier egg
  where
    qstyle = (QuadricStyle (Just Smooth) GenerateTextureCoordinates Outside FillStyle)
    egg = makeHier (do color3d color
                       renderQuadric qstyle $ Sphere w 10 5)
                    (0, 0, 0)
                    (Q.zero) (Nothing) []

compileRock w h tex color = loadJPEGTexture tex >>= (compileHier . rock)
  where
    qstyle = (QuadricStyle (Just Smooth) GenerateTextureCoordinates Outside FillStyle)
    rock tex = makeHier (do color3d color
                            preservingMatrix $ do
                              translated (h, 0, 0)
                              textureBinding Texture2D $= Just tex
                              clipPlane (ClipPlaneName 0) $= Just (Plane (-1) 0 0 0.0)
                              renderQuadric qstyle $ Sphere w 8 4
                              clipPlane (ClipPlaneName 0) $= Nothing
                              textureBinding Texture2D $= Nothing)
                    (0, 0, 0)
                    (Q.zero) (Nothing) []

rule1 f = defaultRule1 f
rule2 f = defaultRule2 f
rule3 f = defaultRule3 f
rule4 f = defaultRule4 f 
            ((boundMinX, boundMaxX), (boundMinY, boundMaxY), (boundMinZ, boundMaxZ))

avoid f c b _ = vecScale (f / (m'*m'*m')) v
  where
    v = pos b `vecSub` pos c
    m = magnitude v
    m' = if m ~== 0 then epsilon else m
chase f = avoid (- f)

constRule v _ _ = v  

----------------------------------------------------------------
-- Below this point is all generalized based on parameters above

data BoidInfo = BI { biFlock :: Int
                   , biAge   :: Int
                   , biScale :: GLdouble
                   , biTailA :: (GLdouble, GLdouble)
                   , biTailV :: (GLdouble, GLdouble, GLdouble) }
  deriving (Show, Eq, Ord)

scaleF s = s + (scaleFactor * (1 - s))
maxSpeedF (minS, maxS) = (minS, maxS - (speedFactor * (maxS - minS)))

data Joint = Jaws | LFin | RFin | Dorsal | Tail deriving (Show, Eq, Ord)

data State = State { flockList :: [IORef (Flock BoidInfo)]
                   , boidList  :: [Compiled (Maybe Joint)]
                   , tankDL    :: DisplayList
                   , curButton :: IORef (Maybe MouseButton)
                   , lastMPos  :: IORef Position
                   , viewingQ  :: IORef (Q.Quat GLdouble)
                   , viewingS  :: IORef GLdouble
                   , anaglyph  :: IORef Bool
                   , verbose   :: IORef Bool
                   , scatter   :: IORef Bool }

flockByNum n = (!! n) . flockList
boidByNum n = (!! n) . boidList
getFlocks state = ([]:) `fmap` mapM ((flockBoids `fmap`) . get) (tail (flockList state))
boidEvents n = (boidEventF !! n)
noEventF b events = return ()
mating n = (boidMatingF !! n)

makeState0 = do
  flocks <- forM [1 .. maxFlockNum] $ \ fnum -> do
    bs <- forM [1 .. snd (flockMinMax !! fnum)] $ \ i -> do
      let r = boidsRadius !! fnum
      a <- randomRIO (0, 1000)
      ((x, y, z), (minS, maxS), v, s, tailA, tailV) <- boidInit fnum a
      return $ makeBoid (fromIntegral i) r
                        (Vector3 x y z) v (minS, maxS)
                        (BI { biFlock = fnum, biAge = a, biScale = s
                            , biTailA = tailA, biTailV = tailV })
    newIORef $ makeFlock bs
    

  let bounds = ((boundMinX, boundMaxX), (boundMinY, boundMaxY), (boundMinZ, boundMaxZ))

  tank <- makeTank
  boidModels <- sequence boidCompileFs
  cb0 <- newIORef Nothing
  mp0 <- newIORef (Position 0 0)
  vq0 <- newIORef (Q.angleAxis 0 (1, 0, 0))
  vs0 <- newIORef 1
  sc0 <- newIORef (False)
  ana <- newIORef (False)
  vrb <- newIORef (False)
  return $ State { flockList = undefined : flocks
                 , boidList = undefined : boidModels
                 , tankDL = tank, curButton = cb0
                 , lastMPos = mp0, viewingQ = vq0, viewingS = vs0
                 , verbose = vrb, anaglyph = ana, scatter = sc0 }

initTailA speed tailA age = (iterate (tailAngleF speed) tailA) !! age
tailAngleF speed (angle, dir) = (angle'', dir')
  where
    angle' = tailAFactor * speed * dir + angle
    (angle'', dir') = if angle' < minTailAngle then
                        (minTailAngle, dir * (-1))
                      else if angle' > maxTailAngle then
                        (maxTailAngle, dir * (-1))
                      else
                        (angle', dir)

boidUpdate b = b { pos = pos', otherData = od', speeds = ss' }
  where
    pos' = pos b `vecAdd` vel b
    od   = otherData b
    od'  = (otherData b) { biAge = biAge od + 1, biScale = s', biTailA = tA' }
    ss'  = maxSpeedF $ speeds b
    s'   = scaleF $ biScale od
    tA   = biTailA od
    tA'  = tailAngleF (magnitude (vel b)) tA

collisionTransform = (scaled (1 / (boundMaxX - boundMinX + 1), 
                              1 / (boundMaxY - boundMinY + 1), 
                              1 / (boundMaxZ - boundMinZ + 1)))

vivariumUpdate state = do
  -- Apply rules to all the boids
  forM_ [1 .. maxFlockNum] $ \ f -> do
    r <- flockRules f state
    flockByNum f state $~ flockUpdate r

  -- Save a copy of the state before  
  flocks0 <- getFlocks state

  -- Add all velocities to all positions
  forM [1 .. maxFlockNum] $ \ f -> do
    flockByNum f state $~ \ fl -> 
      fl { flockBoids = map boidUpdate (flockBoids fl) }

  -- Do collision detection and dispatch events        
  events <- checkCollisions state flocks0
  moreEvents <- newIORef []
  getFlocks state >>= 
    (mapM_ (\ b -> boidEvents (biFlock (otherData b)) b moreEvents) . concat)
  get moreEvents >>= ((processEvents state) . (events ++))

checkWalls events (b, b')
  | (x', y', z') ~== (x'', y'', z'') = return ()
  | otherwise = events $~ (Move (fl, i b) (Vector3 x'' y'' z'', vel b') :)
  where
    Vector3 x y z = pos b
    Vector3 x' y' z' = pos b'
    fl = biFlock (otherData b)
    r = boidsRadius !! fl
    x'' = if boundMinX + r >= x' then max x x' 
          else if x' >= boundMaxX - r then min x x' else x'
    y'' = if boundMinY + r >= y' then max y y'
          else if y' >= boundMaxY - r then min y y' else y'
    z'' = if boundMinZ + r >= z' then max z z'
          else if z' >= boundMaxZ - r then min z z' else z'

fineGrainedCollisions state flocks0 colindices events = do
  flocks <- getFlocks state
  let colindices' = map snd $ filter fst colindices
  let lookup (flock, index) = filter ((== index) . i) (flocks !! flock)
  let lookup0 (flock, index) = filter ((== index) . i) (flocks0 !! flock)
  -- pairs contains all potentially colliding pairs of boids, and their
  -- state in the previous frame (denoted _0)
  let pairs = [ ((b1, b1_0), (b2, b2_0)) 
              | fi1 <- colindices', fi2 <- colindices'
              , fi1 /= fi2
              , b1   <- lookup fi1,  b2   <- lookup fi2
              , b1_0 <- lookup0 fi1, b2_0 <- lookup0 fi2]
  -- pairs' combs out invalid pairs by checking bounding spheres
  let pairs' = filter (\ ((b1, _), (b2, _)) -> 
                          let dv = pos b2 `vecSub` pos b1
                              r  = radius b1 + radius b2 in
                            dv `dotP` dv < r * r)
                      pairs
  let name b = (biFlock (otherData b), i b)

  -- Helper function to determine if b1 ate b2  
  let checkEats (b1, b2)
        | f1 `elem` hungryFlocks && f2 `elem` foodFlocks = do
          -- Coming into contact with food is sufficient to eat it
          events $~ (Eats (f1, i b1) (f2, i b2) :)
          return True
        | f1 `elem` predatorFlocks && f2 `elem` preyFlocks = do
          -- b1 is in predator relationship with b2
          -- check for specific collision between b1's jaw and b2
          collides <- detect collisionTransform
              [ drawCompiledBoidPart Jaws (boidByNum f1 state) b1
              , drawCompiledBoid (boidByNum f2 state) b2 ]
          if and collides 
            then do events $~ (Eats (f1, i b1) (f2, i b2) :)
                    return True
            else return False        
        | otherwise = return False
        where 
          f1 = biFlock (otherData b1)
          f2 = biFlock (otherData b2)

  -- Helper function to determine if b1 mates with b2
  let checkMates (b1, b2)
        | f1 `elem` matingFlocks && f1 == f2 = do
          yes <- mating f1 (b1, b2)
          if yes then do events $~ (Mates (f1, i b1) (f2, i b2) :)
                         return True
                 else return False
        | otherwise = return False
        where 
          f1 = biFlock (otherData b1)
          f2 = biFlock (otherData b2)

  -- Handle collisions between boids, pair-wise  
  let doPair ((b1, b1_0), (b2, b2_0)) = do
          -- first check if b1 ate b2
          checkEats (b1, b2) 

          -- check if b1 mates with b2 (in addition to colliding)
          checkMates (b1, b2)
          
          -- in case of collision, deflect to a velocity perpendicular to
          -- the current velocity and the vector between the two boids.
          let dv_ = pos b1 `vecSub` pos b2
          -- be wary of divide-by-zero
          let dv = if magnitude dv_ ~== 0 then Vector3 epsilon 0 0 else dv_
          let r = radius b1 + radius b2
          let u = (r / (2 * magnitude dv)) `vecScale` dv
          
          -- let deflection mean velocity projected onto the plane with normal dv
          let deflect_ = vecScale 0.5 dv `vecAdd` (vel b1 `projectV` dv)
          let deflect  = if magnitude deflect_ ~== 0 then dv else deflect_
          let deflect' = vecScale (magnitude (vel b1) / magnitude deflect) deflect

          -- add a small random component to the deflection
          rv <- randomVector3d
          let rv' = vecScale (magnitude deflect' / (4 * dmax (epsilon, magnitude rv))) rv
          let deflect'' = deflect' `vecAdd` rv'
                          
          -- putStrLn ("b1="++show (i b1)++" b2="++show (i b2)++" r="++show r++" |dv_0|="++show(magnitude(pos b1_0 `vecSub` pos b2_0))++" |dv|="++show(magnitude dv)++" |deflect'|="++show(magnitude deflect'))
                          
          -- go back to b1_0 position but take one deflected step in
          -- order to make some progress
          let p' = deflect'' `vecAdd` pos b1_0
          events $~ (Move (name b1) (p', deflect'') :)  
  forM_ pairs' doPair
  return events

checkCollisions state flocks0 = do
  events <- newIORef []

  flocks <- getFlocks state

  let indices = map (\ b -> (biFlock (otherData b), i b)) (concat flocks)

  -- list of OpenGL commands
  let actions = 
        map (\ b -> drawCompiledBoid (boidByNum (biFlock (otherData b)) state) b)
          (concat flocks)

  -- First, check for wall collisions  
  mapM_ (checkWalls events) $ zip (concat flocks0) (concat flocks)

  -- Invoke OpenGL hardware-based collision detector
  collides <- detect collisionTransform actions

  -- colindices are booleans paired with boid (flock, index) pairs  
  let colindices = zip collides indices

  -- Do fine-grained collision analysis  
  events' <- fineGrainedCollisions state flocks0 colindices events
           
  get events'

data Event = Eats (Int, Int) (Int, Int)
           | Mates (Int, Int) (Int, Int)
           | Move (Int, Int) (Vector3d, Vector3d)
           | Hatch (Int, Int) Int
  deriving Show

processEvents state events = do
  vrb <- get (verbose state)
  forM_ events $ \ ev -> case ev of
    --------------------------------------------------
    Eats (predflock, pred) (preyflock, prey) -> do
      preyfl <- get (flockByNum preyflock state)
      if length (flockBoids preyfl) > fst (flockMinMax !! preyflock) then do
        if vrb then print ev else return ()
        flockByNum preyflock state $~ \ fl ->
          fl { flockBoids = filter ((/= prey) . i) (flockBoids fl) }
        flockByNum predflock state $~ \ fl ->
          fl { flockBoids = flip map (flockBoids fl) $ \ b ->
                                if i b == pred then
                                  let sc = biScale (otherData b) in
                                    -- Eating restores maximum speed and increases size
                                    b { speeds = (fst (speeds b), 
                                                  snd (speedsMinMax !! predflock))
                                      , otherData = (otherData b) 
                                          { biScale = (1 - sc)*0.5 + sc } }
                                else
                                  b }
        else return ()
    --------------------------------------------------
    Mates (f1, i1) (f2, i2) -> do
      -- get list of boids in same flock as (f1, i1)
      bs <- flockBoids `fmap` get (flockByNum f1 state)
      -- find an associated egg-flock  
      let eggFlock = case lookup f1 eggFlocks of
                        Just f  -> f
                        Nothing -> f1 -- if no egg flock, just clone
      eggs <- flockBoids `fmap` get (flockByNum eggFlock state)
      let numBoids = length bs + length eggs
      case (numBoids < snd (flockMinMax !! f1), find ((== i1) . i) bs) of
        -- True if not overpopulated, and b1 is the Boid corresponding to i1  
        (True, Just b1) -> do
          -- Get an unused index  
          let newI = 1 + maximum (1:[ i b | b <- eggs ])
          (_, (minS, maxS), v, s, tailA, tailV) <- boidInit eggFlock 1
          -- Start in the same position as b1
          let newB = makeBoid (fromIntegral newI) (boidsRadius !! eggFlock)
                       (pos b1) v (minS, maxS)
                       (BI { biFlock = eggFlock, biAge = 1, biScale = s
                           , biTailA = tailA, biTailV = tailV })
          flockByNum eggFlock state $~ \ fl -> fl { flockBoids = newB : flockBoids fl }
          if vrb then print ev else return ()
        _ -> return ()
    --------------------------------------------------
    Move (flock, index) (p, v) -> do
      -- print ev
      flockByNum flock state $~ \ fl ->
        fl { flockBoids = flip map (flockBoids fl) $ \ b ->
                              if i b == index then
                                b { pos = p, vel = v }
                              else
                                b }
    --------------------------------------------------
    Hatch (eggFlock, eggI) speciesFlock -> do
      eggs <- flockBoids `fmap` get (flockByNum eggFlock state)
      case filter ((== eggI) . i) eggs of
        [] -> return ()
        egg:_ -> do
          -- Add new member of species
          bs <- flockBoids `fmap` get (flockByNum speciesFlock state)
          let newI = 1 + maximum (1:[ i b | b <- bs ])
          (_, (minS, maxS), v, s, tailA, tailV) <- boidInit speciesFlock 1
          -- Start in the same position as egg
          let newB = makeBoid (fromIntegral newI) (boidsRadius !! speciesFlock)
                       (pos egg) v (minS, maxS)
                       (BI { biFlock = speciesFlock, biAge = 1, biScale = s
                           , biTailA = tailA, biTailV = tailV })
          -- Add new member    
          flockByNum speciesFlock state $~ 
            \ fl -> fl { flockBoids = newB : flockBoids fl }
          -- Remove egg
          flockByNum eggFlock state $~ 
            \ fl -> fl { flockBoids = filter ((/= eggI) . i) (flockBoids fl) }
          if vrb then print ev else return ()

  return ()

addFood state = do
  foodFlock <- randomChoice foodFlocks
  let r = boidsRadius !! foodFlock
  x <- randomRIO (boundMinX + r, boundMaxX - r)
  y <- randomRIO (boundMinY + r, boundMaxY - r)
  z <- randomRIO (boundMinZ + r, boundMaxZ - r)
  let v = Vector3 0 (-fst (speedsMinMax !! foodFlock)) 0
  bs <- flockBoids `fmap` get (flockByNum foodFlock state)
  let newI = 1 + maximum (1:[ i b | b <- bs ])
  let newB = makeBoid (fromIntegral newI) (boidsRadius !! foodFlock)
               (Vector3 x y z) v 
               (fst (speedsMinMax !! foodFlock), fst (speedsMinMax !! foodFlock))
               (BI { biFlock = foodFlock, biAge = 1, biScale = 1
                   , biTailA = (0, 0), biTailV = (0, 0, 0) })
  flockByNum foodFlock state $~ \ fl -> fl { flockBoids = newB : flockBoids fl }


vivariumDisplay state = do
  callList (tankDL state)
  flocks <- getFlocks state
  forM_ (concat flocks) $ \ b -> 
    drawCompiledBoid (boidByNum (biFlock (otherData b)) state) b

toMatrix :: Q.Quat GLdouble -> IO (GLmatrix GLdouble)
toMatrix = newMatrix ColumnMajor . Q.rowMajorElems -- OpenGL uses column-major

drawCompiledBoid c (B { i = i, pos = p, vel = v, quat = q, otherData = od }) = do
  preservingMatrix $ do
    translated' p
    -- let q = Q.lookAt (1, 0, 0) (vx, vy, vz) (0, 1, 0) where Vector3 vx vy vz = v
    m <- toMatrix q
    multMatrix m
    scaled (biScale od, biScale od, biScale od)
    if (snd (biTailA od)) == 0 
      then drawCompiled c
      else let r = Q.angleAxis (fst (biTailA od)) (biTailV od) 
               s = Q.angleAxis (fst (biTailA od)) (1, 0, 0)
               t = Q.angleAxis (fst (biTailA od)) (-1, 0, 0)
            in drawModifiedCompiled [(Just Tail, r), (Just LFin, s), (Just RFin, t)] c

drawCompiledBoidPart a c (B { i = i, pos = p, vel = v, quat = q, otherData = od }) = do
  preservingMatrix $ do
    translated' p
    -- let q = Q.lookAt (1, 0, 0) (vx, vy, vz) (0, 1, 0) where Vector3 vx vy vz = v
    m <- toMatrix q
    multMatrix m
    scaled (biScale od, biScale od, biScale od)
    drawCompiledPart (Just a) c

makeTank = do
  let w = boundMaxX; nw = boundMinX
  let h = boundMaxY; nh = boundMinY
  let d = boundMaxZ; nd = boundMinZ
  -- Seafloor texture formed from GIMP fill-pattern "mud" designed by
  -- Helen Triantafillou http://www.helensimages.com/
  tex <- loadJPEGTexture "seafloor.jpg"
  defineNewList Compile $ do
    color3d (0, 1, 1)
    
    renderPrimitive LineStrip $ do
      -- top of tank  
      vertex3d ( w, h, d)
      vertex3d (nw, h, d)
      vertex3d (nw, h,nd)
      vertex3d ( w, h,nd)
      vertex3d ( w, h, d)
    renderPrimitive Lines $ do
      -- sides of tank
      vertex3d (nw,nh,nd)
      vertex3d (nw, h,nd)
      vertex3d (nw,nh, d)
      vertex3d (nw, h, d)
      vertex3d ( w,nh,nd)
      vertex3d ( w, h,nd)
      vertex3d ( w,nh, d)
      vertex3d ( w, h, d)
    textureBinding Texture2D $= Just tex
    color3d (1, 1, 1)
    renderPrimitive Quads $ do
      -- bottom of tank
      normal3d (0, 1, 0)
      texCoord2d (1, 1)
      vertex3d ( w,nh, d)
      texCoord2d (0, 1)
      vertex3d (nw,nh, d)
      texCoord2d (0, 0)
      vertex3d (nw,nh,nd)
      texCoord2d (1, 0)
      vertex3d ( w,nh,nd)
    textureBinding Texture2D $= Nothing      
