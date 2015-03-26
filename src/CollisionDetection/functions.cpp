#include "../include/CollisionDetection/functions.h"

clock_t start_t, end_t;

/// przelicza stopnie na radiany
double deg2rad(double degree)
{
    return (degree*PI)/180;
}

/// zamienia kat w radianach na stopnie
double rad2deg(double angle)
{
    return (angle*180/PI);
}

/// zwraca wartosc bezwzgledna
/*float abs(float value)
{
    if (value>0) return value;
    else {
        value=-value;
        return value;
    }
}*/

/// wait
void wait(int ile){
    volatile float a=(volatile float) 0.1234567;
    volatile float b=(volatile float) 2.3456788;
    for (volatile int i=0 ; i<ile; i++){
        volatile float c=a*b;
    }
}

///znajduje maksimum w tablicy float
int findMax(float *table, int size, float * max_value){
    int max_iter=0;
    for(int i=1;i<size;i++) {
        if (table[i]>table[max_iter]) {
            max_iter = i;
        }
    }
    *max_value=table[max_iter];
    return max_iter;
}

///find max value
int findMax(const std::vector<float>& table, float& max_value){
    int max_iter=0;
    for(size_t i=1;i<table.size();i++) {
        if (table[i]>table[max_iter]) {
            max_iter = i;
        }
    }
    max_value=table[max_iter];
    return max_iter;
}

/// inicjalizacja losowosci
void initializeRand(){
    //srand ( time(NULL) );
    srand ( 1234678 );
}

/// losuje liczbe calkowita z przedzialu min, max
int randInt(int IMin, int IMax){
    return (IMin + rand() % (IMax - IMin));
}

/// losuje liczbe calkowita z przedzialu min, max
float randFloat(float Min, float Max){
    return (Min + rand() * (Max - Min) / RAND_MAX);
}

///znajduje maksimum w tablicy float (wartosc bezwzgledna)
int findAbsMax(float *table, int size, float * max_value){
    int max_iter=0;
    for(int i=1;i<size;i++) {
        if (fabs(table[i])>fabs(table[max_iter])) {
            max_iter = i;
        }
    }
    *max_value=fabs(table[max_iter]);
    return max_iter;
}

///znajduje maksimum w tablicy float (wartosc bezwzgledna)
int findAbsMax(const std::vector<float>& table, float& max_value){
    int max_iter=0;
    for(size_t i=1;i<table.size();i++){
        if (fabs(table[i])>fabs(table[max_iter]))
            max_iter = i;
    }
    max_value=fabs(table[max_iter]);
    return max_iter;
}

///znajduje minimum w tablicy float
int findMin(float *table, int size, float * max_value){
    int min_iter=0;
    for(int i=1;i<size;i++) {
        if (table[i]<table[min_iter]) {
            min_iter = i;
        }
    }
    *max_value=table[min_iter];
    return min_iter;
}

///znajduje minimum w tablicy float (wartosc bezwzgledna)
int findAbsMin(float *table, int size, float * max_value){
    int min_iter=0;
    for(int i=1;i<size;i++) {
        if (fabs(table[i])<fabs(table[min_iter])) {
            min_iter = i;
        }
    }
    *max_value=fabs(table[min_iter]);
    return min_iter;
}

//sprawdza czy punkt lezy wewnatrz trojkata
bool triangleIncludePoint(const float *a, const float *b, const float *c, const float *com2d){
    float S_triangle = computeTriangleArea(a,b,c);
    if (S_triangle<=(computeTriangleArea(a,b,com2d)+computeTriangleArea(b,c,com2d)+computeTriangleArea(a,c,com2d)))
        return true;
    else
        return false;
}

//obliczenie powierzchni trojkata
double computeTriangleArea(const float* a, const float* b, const float* c){
    return fabs(0.5*(a[0]*b[1]+b[0]*c[1]+c[0]*a[1]-c[0]*b[1]-a[0]*c[1]-b[0]*a[1]));
}

//sprawdza czy punkt lezy wewnatrz wielokata
bool polygonIncludePoint(const vector< vector<float> >& vertices,float *com2d, int vert_no){
    if (vert_no!=0){
        double S_triangle = computePolygonArea(vertices,vert_no);
        double area=0;
        for (int i=0;i<vert_no-1;i++){
            float vert1[2]={vertices[i][0], vertices[i][1]};
            float vert2[2]={vertices[i+1][0], vertices[i+1][1]};
            area+=computeTriangleArea(vert1,vert2,com2d);
        }
        float vert1[2]={vertices[vert_no-1][0], vertices[vert_no-1][1]};
        float vert2[2]={vertices[0][0], vertices[0][1]};
        area+=computeTriangleArea(vert1,vert2,com2d);
        if (S_triangle<=area*1.05)
            return true;
        else
            return false;
    }
    else
        return false;
}

//obliczenie centroidu dla wielokata
void computePolygonCentroid(const vector< vector<float> >& vertices, int vert_no, float * Cx, float *Cy){
    double area=computePolygonArea(vertices, vert_no);
    *Cx=0; *Cy=0;
    for (int i=0;i<vert_no-1;i++){
        *Cx+=(vertices[i][0]+vertices[i+1][0])*(vertices[i][0]*vertices[i+1][1]-vertices[i+1][0]*vertices[i][1]);
        *Cy+=(vertices[i][1]+vertices[i+1][1])*(vertices[i][0]*vertices[i+1][1]-vertices[i+1][0]*vertices[i][1]);
    }
    *Cx+=(vertices[vert_no-1][0]+vertices[0][0])*(vertices[vert_no-1][0]*vertices[0][1]-vertices[0][0]*vertices[vert_no-1][1]);
    *Cy+=(vertices[vert_no-1][1]+vertices[0][1])*(vertices[vert_no-1][0]*vertices[0][1]-vertices[0][0]*vertices[vert_no-1][1]);
    *Cx/=-6*area;
    *Cy/=-6*area;
}

//obliczenie powierzchni wielokata
double computePolygonArea(const vector< vector<float> >& vertices, int vert_no){
    double area=0;
    for (int i=0;i<vert_no-1;i++){
        area+=(vertices[i][0]*vertices[i+1][1])-(vertices[i+1][0]*vertices[i][1]);
    }
    area+=(vertices[vert_no-1][0]*vertices[0][1])-(vertices[0][0]*vertices[vert_no-1][1]);
    return fabs(area)/2;
}

void hotColorMap(unsigned char *rgb,float value,float min,float max)
{
  float max3=(max-min)/3;
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max3)
    {rgb[0]=(unsigned char)(255*value/max3);rgb[1]=0;rgb[2]=0;}
  else if(value<2*max3)
    {rgb[0]=255;rgb[1]=(unsigned char)(255*(value-max3)/max3);rgb[2]=0;}
  else if(value<max)
    {rgb[0]=255;rgb[1]=255;rgb[2]=(unsigned char)(255*(value-2*max3)/max3);}
  else {rgb[0]=rgb[1]=rgb[2]=255;}
}

void jetColorMap(unsigned char *rgb,float value,float min,float max)
{
  unsigned char c1=144;
  float max4=(max-min)/4;
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max4)
    {rgb[0]=0;rgb[1]=0;rgb[2]=c1+(unsigned char)((255-c1)*value/max4);}
  else if(value<2*max4)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max4)/max4);rgb[2]=255;}
  else if(value<3*max4)
    {rgb[0]=(unsigned char)(255*(value-2*max4)/max4);rgb[1]=255;rgb[2]=255-rgb[0];}
  else if(value<max)
    {rgb[0]=255;rgb[1]=(unsigned char)(255-255*(value-3*max4)/max4);rgb[2]=0;}
  else {rgb[0]=255;rgb[1]=rgb[2]=0;}
}

void coldColorMap(unsigned char *rgb,float value,float min,float max)
{
  float max3=(max-min)/3;
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max3)
    {rgb[0]=0;rgb[1]=0;rgb[2]=(unsigned char)(255*value/max3);}
  else if(value<2*max3)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max3)/max3);rgb[2]=255;}
  else if(value<max)
    {rgb[0]=(unsigned char)(255*(value-2*max3)/max3);rgb[1]=255;rgb[2]=255;}
  else {rgb[0]=rgb[1]=rgb[2]=255;}
}

void blueColorMap(unsigned char *rgb,float value,float min,float max)
{
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max)
    {rgb[0]=0;rgb[1]=0;rgb[2]=(unsigned char)(255*value/max);}
  else {rgb[0]=rgb[1]=0;rgb[2]=255;}
}

void positiveColorMap(unsigned char *rgb,float value,float min,float max)
{
  value-=min;
  max-=min;
  value/=max;

  if(value<0){
  rgb[0]=rgb[1]=rgb[2]=0;
    return;
  }
  if(value>1){
  rgb[0]=rgb[1]=rgb[2]=255;
  return;
  }

  rgb[0]=192;rgb[1]=0;rgb[2]=0;
  rgb[0]+=(unsigned char)(63*value);
  rgb[1]+=(unsigned char)(255*value);
  if(value>0.5)
  rgb[2]+=(unsigned char)(255*2*(value-0.5));
}

void negativeColorMap(unsigned char *rgb,float value,float min,float max)
{
  value-=min;
  max-=min;
  value/=max;

  rgb[0]=0;rgb[1]=0;rgb[2]=0;
  if(value<0) return;
  if(value>1){
  rgb[1]=rgb[2]=255;
  return;
  }

  rgb[1]+=(unsigned char)(255*value);
  if(value>0.5)
  rgb[2]+=(unsigned char)(255*2*(value-0.5));

}

void colorMap(unsigned char *rgb,float value,float min,float max)
{
  if(value>0)
    positiveColorMap(rgb,value,0,max);
  else
    negativeColorMap(rgb,value,min,0);
/*
  if(value>0)
    hotColorMap(rgb,value,min,max);
  else
    coldColorMap(rgb,value,min,max);
    */
}

void cyclicColorMap(unsigned char *rgb,float value,float min,float max)
{
  float max3=(max-min)/3;
  value-=(max-min)*(float)floor((value-min)/(max-min));
  if(value<max3)
    {rgb[0]=(unsigned char)(255-255*value/max3);rgb[1]=0;rgb[2]=255-rgb[0];}
  else if(value<2*max3)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max3)/max3);rgb[2]=255-rgb[1];}
  else if(value<max)
    {rgb[0]=(unsigned char)(255*(value-2*max3)/max3);rgb[1]=255-rgb[0];rgb[2]=0;}

}

void randColorMap(unsigned char *rgb,float value,float min,float max)
{
  srand((int)(65000*(value-min)/(max-min)));
  rgb[0]=(unsigned char)(255*rand());
  rgb[1]=(unsigned char)(255*rand());
  rgb[2]=(unsigned char)(255*rand());
}

void grayColorMap(unsigned char *rgb,float value,float min,float max)
{
  max-=min;
  value-=min;
  rgb[0]=rgb[1]=rgb[2]=(unsigned char)(255*value/max);
}


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

//startuje timer do pomiaru czasu wykonania polecenia
void startTimeMeasure(void){
    start_t = clock();
}

//startuje timer do pomiaru czasu wykonania polecenia
double stopTimeMeasure(void){
    end_t = clock();
    return (double)(end_t-start_t)/CLOCKS_PER_SEC;
}
