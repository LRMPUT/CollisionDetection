#include "../include/CollisionDetection/functions.h"


// Funkcja normalizujaca wektor podany jako zbior trzech wspolrzednych
void ReduceToUnit(float vector[3])
 {
 float length;

  // Oblicz dlugosc wektora
  length = (float)sqrt(  (vector[0]*vector[0]) +
     (vector[1]*vector[1]) +
     (vector[2]*vector[2]));

  // Zabezpieczenie przed podzialem przez 0
  if(length == 0.0f)
  length = 1.0f;

  // Podziel kazda ze wspolrzednych przez dlugosc wektora
  vector[0] /= length;
  vector[1] /= length;
  vector[2] /= length;
 }



// Punkty p1, p2 i p3 zdefiniowane w odwrotnym do wskazowek zegara
// porzadku
void calcNormal(float **v, float *out)
 {
 float v1[3],v2[3];
  static const int x = 0;
  static const int y = 1;
  static const int z = 2;

  // Oblicz 2 wektory na podstawie trzech punktow
  v1[x] = v[0][x] - v[1][x];
  v1[y] = v[0][y] - v[1][y];
  v1[z] = v[0][z] - v[1][z];

  v2[x] = v[1][x] - v[2][x];
  v2[y] = v[1][y] - v[2][y];
  v2[z] = v[1][z] - v[2][z];

  // Oblicz wspolrzedne wektora normalnego na podstawie
  // iloczynu wektorowego
  out[x] = v1[y]*v2[z] - v1[z]*v2[y];
  out[y] = v1[z]*v2[x] - v1[x]*v2[z];
  out[z] = v1[x]*v2[y] - v1[y]*v2[x];
   // Normalizuj wektor
 ReduceToUnit(out);
 }
