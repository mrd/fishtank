# Fishtank: 3D OpenGL demo with flocking boids
# Author: Matthew Danish.  License: BSD3 (see LICENSE file)

# Project Makefile for Linux systems

HC = ghc
CC = gcc
HCFLAGS = -O2 -ljpeg
PROG = Main
HSFILES = Boids.hs GLDouble.hs Hier.hs Main.hs Quat.hs Util.hs Vivarium.hs JpegTexture.hs Collision.hs
OFILES = $(patsubst %.hs,%.o,$(HSFILES))
HIFILES = $(patsubst %.hs,%.hi,$(HSFILES))

.PHONY: all
all: $(PROG)

$(PROG): $(HSFILES) jpeg.o
	$(HC) $(HCFLAGS) --make $@ jpeg.o

.PHONY: clean
clean:
	rm -f $(PROG) $(OFILES) $(HIFILES) jpeg.o

jpeg.o: jpeg.c
