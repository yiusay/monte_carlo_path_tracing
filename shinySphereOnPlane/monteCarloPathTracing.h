#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <vector>
#include <algorithm>
#include <GL/glut.h>
using namespace std;

#include <windows.h>
#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")




extern void GLInit();
extern void CrossProduct( double *a, double *b, double *c );
extern void Normalize(double *a);
extern double DotProduct(double *a, double *b);
extern double DotProduct4D(double *a, double *b);
extern void GetEdgeVector(double *v1, double *v2, double *edge_vector);
extern void GetArea(double *normal, double &area);
extern double GetDistance(double *a, double *b);
extern double GetLength(double *a);
extern void Swap(double &a, double &b);
extern double Max(double a, double b);
extern bool SolveLinearSystem(double (*matrix)[4], double *rhs, double *solution);
extern void RotateAroundAxis(double *rotation_axis, double theta, double *vec_in, double *vec_out);
extern void ComputeReflectionVector(double *normal, double *vector_in, double *reflection_vector);

extern GLint window_width, window_height;
extern double *png_data;
extern int png_width, png_height;

extern double *table_texture;
extern int     table_texture_width, table_texture_height;

#define N_SAMPLES 1000
#define MAX_DEPTH 100
#define PI    3.14159265
#define TwoPI 6.28318531
#define InvPI 0.318309886