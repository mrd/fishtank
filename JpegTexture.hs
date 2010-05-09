{-# LANGUAGE ForeignFunctionInterface #-}
{-# CFILES jpeg.c #-}
-- Fishtank: 3D OpenGL demo with flocking boids
-- Author: Matthew Danish.  License: BSD3 (see LICENSE file)
--
-- Foreign-function Interface to my jpeg.c module imported from
-- previous project.  I could have used a Haskell jpeg-library, but
-- why break what I know works?

module JpegTexture ( loadJPEGTexture ) where
import Foreign.C
import Graphics.Rendering.OpenGL.GL

foreign import ccall "load_JPEG_texture" c_loadJPEGTexture :: CString -> IO GLuint

loadJPEGTexture filename = TextureObject `fmap` withCString filename c_loadJPEGTexture
