-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Approximation versions of equality (~==) and less-than (~<) and
-- some other convenience functions.
module GLDouble ((~==), (~<), dmin, dmax, sgn, epsilon) where
import Graphics.Rendering.OpenGL.GL ( Vector3 (..) )
class Approx a where
  (~==) :: a -> a -> Bool
  (~<) :: a -> a -> Bool

epsilon = 0.00001

instance Approx Double where
  a ~== b = abs (a - b) < epsilon
  a ~< b = a - b < -epsilon

instance (Approx a, Approx b, Approx c) => Approx (a, b, c) where
  (a1,b1,c1) ~== (a2,b2,c2) = a1 ~== a2 && b1 ~== b2 && c1 ~== c2
  (a1,b1,c1) ~< (a2,b2,c2) = a1 ~< a2 || (a1 ~== a2 && 
                                (b1 ~< b2 || (b1 ~== b2 && c1 ~< c2)))

instance (Approx a) => Approx (Vector3 a) where
  Vector3 a1 b1 c1 ~== Vector3 a2 b2 c2 = a1 ~== a2 && b1 ~== b2 && c1 ~== c2
  Vector3 a1 b1 c1 ~<  Vector3 a2 b2 c2 = a1 ~< a2 || (a1 ~== a2 && 
                                          (b1 ~< b2 || (b1 ~== b2 && c1 ~< c2)))


dmin (a, b) = if a ~< b then b else a
dmax (a, b) = if a ~< b then a else b

sgn a = if a ~== 0 then 0 else if a ~< 0 then -1 else 1
