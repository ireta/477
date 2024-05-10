#ifndef __HELPERS_H__
#define __HELPERS_H__

#define ABS(a) ((a) > 0 ? (a) : -1 * (a))
#define EPSILON 0.000000001

#include "Matrix4.h"
#include "Vec3.h"
#include "Vec4.h"
#include <vector>

/*
 * Calculate cross product of vec3 a, vec3 b and return resulting vec3.
 */
Vec3 crossProductVec3(Vec3 a, Vec3 b);

/*
 * Calculate dot product of vec3 a, vec3 b and return resulting value.
 */
double dotProductVec3(Vec3 a, Vec3 b);

/*
 * Find length (|v|) of vec3 v.
 */
double magnitudeOfVec3(Vec3 v);

/*
 * Normalize the vec3 to make it unit vec3.
 */
Vec3 normalizeVec3(Vec3 v);

/*
 * Return -v (inverse of vec3 v)
 */
Vec3 inverseVec3(Vec3 v);

/*
 * Add vec3 a to vec3 b and return resulting vec3 (a+b).
 */
Vec3 addVec3(Vec3 a, Vec3 b);

/*
 * Subtract vec3 b from vec3 a and return resulting vec3 (a-b).
 */
Vec3 subtractVec3(Vec3 a, Vec3 b);

/*
 * Multiply each element of vec3 with scalar.
 */
Vec3 multiplyVec3WithScalar(Vec3 v, double c);

/*
 * Prints elements in a vec3. Can be used for debugging purposes.
 */
void printVec3(Vec3 v);

/*
 * Check whether vec3 a and vec3 b are equal.
 * In case of equality, returns 1.
 * Otherwise, returns 0.
 */
int areEqualVec3(Vec3 a, Vec3 b);
/*
 * Returns an identity matrix (values on the diagonal are 1, others are 0).
*/
Matrix4 getIdentityMatrix();

/*
 * Multiply matrices m1 (Matrix4) and m2 (Matrix4) and return the result matrix r (Matrix4).
 */
Matrix4 multiplyMatrixWithMatrix(Matrix4 m1, Matrix4 m2);

/*
 * Multiply matrix m (Matrix4) with vector v (vec4) and store the result in vector r (vec4).
 */
Vec4 multiplyMatrixWithVec4(Matrix4 m, Vec4 v);

Vec3 multiplyToVec3(std::vector<std::vector<double>> m1, Vec4 m2);

std::vector<std::vector<double>> multpilyToArr(std::vector<std::vector<double>> m1, Matrix4 m2);

double f01(double x, double y, double y0, double y1, double x0, double x1);

double f12(double x, double y, double y1, double y2, double x1, double x2);

double f20(double x, double y, double y0, double y2, double x0, double x2);

void printMatrix4(Matrix4 m);

Vec4 findintersection(Vec4 v1, Vec4 v2, double max, int op);

Vec4 multiplyWithScalar(Vec4 v, double s);

bool visible(double den, double num, double &tE, double &tL);

//Matrix4 camera_transform(Scene::Camera *camera);

//std::vector<int> BSP(std::vector<Scene::Triangle> triangles);

#endif
