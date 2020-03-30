#  Based on the perlin noise algorithm here:
# @inproceedings{perlin2002improving,
#   title={Improving noise},
#   author={Perlin, Ken},
#   booktitle={ACM transactions on graphics (TOG)},
#   volume={21},
#   number={3},
#   pages={681--682},
#   year={2002},
#   organization={ACM}
# }

# Code Adapted/Streamlined for 1D from:
# https://github.com/JuliaGraphics/Luxor.jl

# Then converted to python for compatibility
# By Andrew Schoen

import math

STRETCH_CONSTANT_2D = -0.211324865405187    # (1/sqrt(2+1)-1)/2
SQUISH_CONSTANT_2D  =  0.366025403784439    # (sqrt(2+1)-1)/2
NORM_CONSTANT_2D    = 47
perm                = [0]*256

gradients2D = [ 5,  2,    2,  5,
               -5,  2,   -2,  5,
                5, -2,    2, -5,
               -5, -2,   -2, -5]

def clamp(x, lo, hi):
    if lo <= x <= hi:
        return x
    elif x < lo:
        return lo
    elif x > hi:
        return hi

def extrapolate(xsb, ysb, dx, dy):
    index = perm[(perm[xsb & 0xFF] + ysb) & 0xFF] & 0x0E
    return gradients2D[index % 16] * dx + gradients2D[index % 16] * dy

def simplexnoise(x, y=0):
    # Place input coordinates onto grid.
    stretchOffset = (x + y) * STRETCH_CONSTANT_2D
    xs = x + stretchOffset
    ys = y + stretchOffset

    # Floor to get grid coordinates of rhombus (stretched square) super-cell origin.
    xsb = int(math.floor(xs))
    ysb = int(math.floor(ys))

    # Skew out to get actual coordinates of rhombus origin. We'll need these later.
    squishOffset = (xsb + ysb) * SQUISH_CONSTANT_2D
    xb = xsb + squishOffset
    yb = ysb + squishOffset

    # Compute grid coordinates relative to rhombus origin.
    xins = xs - xsb
    yins = ys - ysb

    # Sum those together to get a value that determines which region we're in.
    inSum = xins + yins

    # Positions relative to origin point.
    dx0 = x - xb
    dy0 = y - yb

    # We'll be defining these inside the next block and using them afterwards.
    # dx_ext, dy_ext, xsv_ext, ysv_ext

    value = 0.0

    # Contribution (1,0)
    dx1 = dx0 - 1 - SQUISH_CONSTANT_2D
    dy1 = dy0 - 0 - SQUISH_CONSTANT_2D
    attn1 = 2 - dx1 * dx1 - dy1 * dy1
    if attn1 > 0:
        attn1 *= attn1
        value += attn1 * attn1 * extrapolate(xsb + 1, ysb + 0, dx1, dy1)

    # Contribution (0,1)
    dx2 = dx0 - 0 - SQUISH_CONSTANT_2D
    dy2 = dy0 - 1 - SQUISH_CONSTANT_2D
    attn2 = 2 - dx2 * dx2 - dy2 * dy2
    if attn2 > 0:
        attn2 *= attn2
        value += attn2 * attn2 * extrapolate(xsb + 0, ysb + 1, dx2, dy2)

    if inSum <= 1: # We're inside the triangle (2-Simplex) at (0,0)
        zins = 1 - inSum
        if zins > xins or zins > yins: # (0,0) is one of the closest two triangular vertices
            if xins > yins:
                xsv_ext = xsb + 1
                ysv_ext = ysb - 1
                dx_ext = dx0 - 1
                dy_ext = dy0 + 1
            else:
                xsv_ext = xsb - 1
                ysv_ext = ysb + 1
                dx_ext = dx0 + 1
                dy_ext = dy0 - 1
        else: # (1,0) and (0,1) are the closest two vertices.
            xsv_ext = xsb + 1
            ysv_ext = ysb + 1
            dx_ext = dx0 - 1 - 2 * SQUISH_CONSTANT_2D
            dy_ext = dy0 - 1 - 2 * SQUISH_CONSTANT_2D
    else: # We're inside the triangle (2-Simplex) at (1,1)
        zins = 2 - inSum
        if zins < xins or zins < yins: # (0,0) is one of the closest two triangular vertices
            if xins > yins:
                xsv_ext = xsb + 2
                ysv_ext = ysb + 0
                dx_ext = dx0 - 2 - 2 * SQUISH_CONSTANT_2D
                dy_ext = dy0 + 0 - 2 * SQUISH_CONSTANT_2D
            else:
                xsv_ext = xsb + 0
                ysv_ext = ysb + 2
                dx_ext = dx0 + 0 - 2 * SQUISH_CONSTANT_2D
                dy_ext = dy0 - 2 - 2 * SQUISH_CONSTANT_2D
        else:  # (1,0) and (0,1) are the closest two vertices.
            dx_ext = dx0
            dy_ext = dy0
            xsv_ext = xsb
            ysv_ext = ysb
        xsb += 1
        ysb += 1
        dx0 = dx0 - 1 - 2 * SQUISH_CONSTANT_2D
        dy0 = dy0 - 1 - 2 * SQUISH_CONSTANT_2D

    # Contribution (0,0) or (1,1)
    attn0 = 2 - dx0 * dx0 - dy0 * dy0
    if attn0 > 0:
        attn0 *= attn0
        value += attn0 * attn0 * extrapolate(xsb, ysb, dx0, dy0)

    # Extra Vertex
    attn_ext = 2 - dx_ext * dx_ext - dy_ext * dy_ext
    if attn_ext > 0:
        attn_ext *= attn_ext
        value += attn_ext * attn_ext * extrapolate(xsv_ext, ysv_ext, dx_ext, dy_ext)

    # convert to [0, 1]
    res = value / NORM_CONSTANT_2D
    return clamp((res + 1) / 2, 0.0, 1.0)

def octavesfn(x, octaves=1, persistence=1.0):
    total     = 0.0
    frequency = 1.0
    amplitude = 1.0
    maxval    = 0.0
    for i in range(octaves):
        total += simplexnoise(x * frequency) * amplitude
        maxval += amplitude
        amplitude *= persistence
        frequency *= 2
    return total / maxval

def noise1D(x, seed, detail=1, persistence=1.0):
    return octavesfn(x+seed, octaves=detail, persistence=persistence)

def noise2D(x, seedvec, detail=1, persistence=1.0):
 return [noise(x, seedvec[0],detail=detail,persistence=persistence) * 2 - 1,
         noise(x, seedvec[1],detail=detail,persistence=persistence) * 2 - 1]

def noise3D(x, seedvec, detail=1, persistence=1.0):
 return [noise(x, seedvec[0],detail=detail,persistence=persistence) * 2 - 1,
         noise(x, seedvec[1],detail=detail,persistence=persistence) * 2 - 1,
         noise(x, seedvec[2],detail=detail,persistence=persistence) * 2 - 1]

def noise4D(x, seedvec, detail=1, persistence=1.0):
 return [noise(x, seedvec[0],detail=detail,persistence=persistence),
         noise(x, seedvec[1],detail=detail,persistence=persistence),
         noise(x, seedvec[2],detail=detail,persistence=persistence),
         noise(x, seedvec[3],detail=detail,persistence=persistence)]
