#include "../include/CollisionDetection/functions.h"


/// Function normalizing a vector given as a set of 3 coordinates
void ReduceToUnit(float vector[3])
 {
 float length;

  // Estimates the length of the vector
  length = (float)sqrt(  (vector[0]*vector[0]) +
     (vector[1]*vector[1]) +
     (vector[2]*vector[2]));

  /// 'Division by zero' protection
  if(length == 0.0f)
  length = 1.0f;

  /// Divides every coordinate by the length of the vector
  vector[0] /= length;
  vector[1] /= length;
  vector[2] /= length;
 }

/// Points p1, p2 and p3 are defined in a counterclockwise order
void calcNormal(float **v, float *out)
 {
 float v1[3],v2[3];
  static const int x = 0;
  static const int y = 1;
  static const int z = 2;

  /// Calculates 2 vectors on the basis of 3 points in space
  v1[x] = v[0][x] - v[1][x];
  v1[y] = v[0][y] - v[1][y];
  v1[z] = v[0][z] - v[1][z];

  v2[x] = v[1][x] - v[2][x];
  v2[y] = v[1][y] - v[2][y];
  v2[z] = v[1][z] - v[2][z];

  /// Calculates the normal vector's coordinates using cross product
  out[x] = v1[y]*v2[z] - v1[z]*v2[y];
  out[y] = v1[z]*v2[x] - v1[x]*v2[z];
  out[z] = v1[x]*v2[y] - v1[y]*v2[x];

  /// Normalizing a vector
 ReduceToUnit(out);
 }
