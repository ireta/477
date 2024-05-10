#ifndef __HW1__HELPER__
#define __HW1__HELPER__

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include "parser.h"

parser::Vec3f multS(parser::Vec3f v,float s);
parser::Vec3f add(parser::Vec3f v1, parser::Vec3f v2);
parser::Vec3f CProduct(parser::Vec3f v1, parser::Vec3f v2);
float DProduct(parser::Vec3f v1, parser::Vec3f v2);
void printVector(parser::Vec3f v);
parser::Vec3f normalize(parser::Vec3f v);
float det3x3(float m[3][3]);
std::vector<parser::Vec3f> generateRay(int i, int j, parser::Vec3f q, parser::Vec3f u, parser::Vec3f v, float l, float r, float b, float t, parser::Vec3f pos, int width,int height);
parser::Vec3f computeColor(int times, parser::Scene scene, std::vector<parser::Vec3f> ray);
float intersectSphere(float radius, parser::Vec3f vertex, std::vector<parser::Vec3f> ray);
float intersectTri(parser::Vec3f a, parser::Vec3f b, parser::Vec3f c, std::vector<parser::Vec3f> ray);
bool noShadow(parser::Vec3f intersectPoint, parser::Vec3f L, parser::Scene scene, parser::Vec3f N, parser::Vec3f lightp);
//parser::Vec3f mirror(int times, parser::Vec3f intersectPoint, parser::Vec3f go, parser::Scene scene);

#endif
