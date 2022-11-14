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
float det3x3(float **m);
std::vector<parser::Vec3f> generateRay(int i, int j, int cam, parser::Vec3f q, parser::Vec3f u, parser::Vec3f v, parser::Scene scene);
parser::Vec3f computeColor(int times, parser::Scene scene, std::vector<parser::Vec3f> ray);
float intersectSphere(parser::Sphere s, parser::Scene scene, std::vector<parser::Vec3f> ray);
float intersectTri(parser::Triangle t, parser::Scene scene, std::vector<parser::Vec3f> ray);
bool noShadow(parser::Vec3f intersectPoint, parser::Vec3f L, parser::Scene scene, parser::Vec3f N, parser::Vec3f lightp);
//parser::Vec3f mirror(int times, parser::Vec3f intersectPoint, parser::Vec3f go, parser::Scene scene);

#endif
