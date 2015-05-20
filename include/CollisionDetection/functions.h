#pragma once

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <vector>

using namespace std;

#define PI 3.14159265

/// Function normalizing a vector given as a set of 3 coordinates
void ReduceToUnit(float vector[3]);

/// Points p1, p2 and p3 are defined in a counterclockwise order
void calcNormal(float** v, float* out);
