-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Approximation versions of equality (~==) and less-than (~<) and
-- some other convenience functions.
module GLDouble ((~==), (~<), dmin, dmax, sgn, epsilon) where
import Data.Quaternion.Approx
import Graphics.Rendering.OpenGL.GL ( Vector3 (..) )

instance (Approx a) => Approx (Vector3 a) where
  Vector3 a1 b1 c1 ~== Vector3 a2 b2 c2 = a1 ~== a2 && b1 ~== b2 && c1 ~== c2
  Vector3 a1 b1 c1 ~<  Vector3 a2 b2 c2 = a1 ~< a2 || (a1 ~== a2 && 
                                          (b1 ~< b2 || (b1 ~== b2 && c1 ~< c2)))


dmin (a, b) = amin a b
dmax (a, b) = amax a b

epsilon :: Double
epsilon = (last . takeWhile (/= 1) . map (+1) . iterate (/2) $ 1) - 1
