#pragma once

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <vector>

using namespace std;

#define PI 3.14159265
//enum param_t {d_plat,d_robot,d_legx,d_legy,d_leg_up};

/// wait
void wait(int ile);
/// przelicza stopnie na radiany
double deg2rad(double degree);
/// zamienia katy w radianach na stopnie
double rad2deg(double angle);
/// przelicza stopnie na radiany
//float deg2rad(float degree);
/// zamienia katy w radianach na stopnie
//float rad2deg(float angle);
/// zwraca wartosc bezwzgledna
/// inicjalizacja losowosci
void initializeRand();
/// losuje liczbe calkowita z przedzialu min, max
int randInt(int min, int max);
/// losuje liczbe calkowita z przedzialu min, max
float randFloat(float Min, float Max);
/// returns absolute value of 'value'
float abs(float value);
///znajduje maksimum w tablicy float
int findMax(float *max, int size, float * max_value);
///find max value
int findMax(const std::vector<float>& table, float& max_value);
///znajduje maksimum w tablicy float (wartosc bezwzgledna)
int findAbsMax(float *table, int size, float * max_value);
///znajduje maksimum w tablicy float (wartosc bezwzgledna)
int findAbsMax(const std::vector<float>& table, float& max_value);
///znajduje minimum w tablicy float
int findMin(float *table, int size, float * max_value);
///znajduje minimum w tablicy float (wartosc bezwzgledna)
int findAbsMin(float *table, int size, float * max_value);
//sprawdza czy punkt lezy wewnatrz trojkata
bool triangleIncludePoint(const float *a,const float *b,const float *c,const float *com2d);
//obliczenie powierzchni trojkata
double computeTriangleArea(const float* a, const float* b, const float* c);
//sprawdza czy punkt lezy wewnatrz wielokata
bool polygonIncludePoint(const vector< vector<float> >& vertices,float *com2d, int vert_no);
//obliczenie powierzchni wielokata
double computePolygonArea(const vector< vector<float> >& vertices, int vert_no);
//obliczenie centroidu dla wielokata
void computePolygonCentroid(const vector< vector<float> >& vertices, int vert_no, float * Cx, float *Cy);
// mapy kolorow
void hotColorMap(unsigned char *rgb,float value,float min,float max);
void jetColorMap(unsigned char *rgb,float value,float min,float max);
void coldColorMap(unsigned char *rgb,float value,float min,float max);
void blueColorMap(unsigned char *rgb,float value,float min,float max);
void positiveColorMap(unsigned char *rgb,float value,float min,float max);
void negativeColorMap(unsigned char *rgb,float value,float min,float max);
void colorMap(unsigned char *rgb,float value,float min,float max);
void cyclicColorMap(unsigned char *rgb,float value,float min,float max);
void randColorMap(unsigned char *rgb,float value,float min,float max);
void grayColorMap(unsigned char *rgb,float value,float min,float max);
/// Funkcja normalizujaca wektor podany jako zbior trzech wspolrzednych
void ReduceToUnit(float vector[3]);
/// Punkty p1, p2 i p3 zdefiniowane w odwrotnym do wskazowek zegara porzadku
void calcNormal(float** v, float* out);
/// startuje timer do pomiaru czasu wykonania polecenia
void startTimeMeasure(void);
/// zatrzymuje timer do pomiaru czasu wykonania polecenia
double stopTimeMeasure(void);
