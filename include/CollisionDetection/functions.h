#pragma once

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <vector>

using namespace std;

#define PI 3.14159265

/// Funkcja normalizujaca wektor podany jako zbior trzech wspolrzednych
void ReduceToUnit(float vector[3]);
/// Punkty p1, p2 i p3 zdefiniowane w odwrotnym do wskazowek zegara porzadku
void calcNormal(float** v, float* out);
