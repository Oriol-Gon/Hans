#!/usr/bin/env python

import sys
import math


# Easy functions to calculate distances

def Distance(xyz,EPoint):

    x= EPoint.position.x-xyz[0]
    y= EPoint.position.y-xyz[1]
    z= EPoint.position.z-xyz[2]

    return math.sqrt(x*x+y*y+z*z)

def Distance2Points(IPoint,EPoint):

    x= EPoint.position.x-IPoint.position.x
    y= EPoint.position.y-IPoint.position.y
    z= EPoint.position.z-IPoint.position.z

    return math.sqrt(x*x+y*y+z*z)


