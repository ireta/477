#include <iostream>
#include <cmath>
#include "Helpers.h"
#include "Matrix4.h"
#include "Vec3.h"
#include "Vec4.h"

using namespace std;

/*
 * Calculate cross product of vec3 a, vec3 b and return resulting vec3.
 */
/*std::vector<int> BSP(std::vector<Scene::Triangle> triangles)
{

}*/

/*Matrix4 camera_transform(Scene::Camera *camera)
{
    matrix4 camera_M;
    camera_M.val[0][0] = camera->u.x;
    camera_M.val[0][1] = camera->u.y;
    camera_M.val[0][2] = camera->u.z;
    camera_M.val[0][3] = -(camera->u.x*camera->pos.x + camera->u.y*camera->pos.y + camera->u.z*camera->pos.z);
    camera_M.val[1][0] = camera->v.x;
    camera_M.val[1][1] = camera->v.y;
    camera_M.val[1][2] = camera->v.z;
    camera_M.val[1][3] = -(camera->v.x*camera->pos.x + camera->v.y*camera->pos.y + camera->v.z*camera->pos.z);
    camera_M.val[2][0] = camera->w.x;
    camera_M.val[2][1] = camera->w.y;
    camera_M.val[2][2] = camera->w.z;
    camera_M.val[2][3] = -(camera->w.x*camera->pos.x + camera->w.y*camera->pos.y + camera->w.z*camera->pos.z);
    camerafor()_M.val[3][3] = 1;
    return camera_M;
}*/

bool visible(double den, double num, double &tE, double &tL)
{
    double t;
    if(den > 0){
        t = num/den;
        if(t > tL){
            return false;
        }else if(t > tE){
            tE = t;
        }
    }else if(den < 0){
        t = num/den;
        if(t < tE){
            return false;
        }else if(t < tL){
            tL = t;
        }
    }else if(num > 0){
        return false;
    }
    return true;
}

Vec4 multiplyWithScalar(Vec4 v, double s)
{
    v.x = v.x/s;
    v.y = v.y/s;
    v.z = v.z/s;
    v.t = v.t/s;
    return v;
}

Vec4 findintersection(Vec4 v1, Vec4 v2, double max, int op)
{
    Vec4 result;
    double t;
    if(op){
        t = (max - v1.x)/(v2.x - v1.x);
        result.x = max;
        result.y = v1.y + (v2.y - v1.y)*t;
        result.z = v1.z;
        result.t = v1.t;
    }else{
      t = (max - v1.y)/(v2.y - v1.y);
      result.x = v1.x + (v2.x - v1.x)*t;
      result.y = max;
      result.z = v1.z;
      result.t = v1.t;
    }
    return result;
}

void printMatrix4(Matrix4 m)
{
    for(int i=0 ; i<4 ; ++i){
        for(int j=0 ; j<4 ; ++j){
            std::cout << m.val[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

double f01(double x, double y, double y0, double y1, double x0, double x1)
{
    double result;
    result = x*(y0-y1) + y*(x1-x0) + x0*y1 - y0*x1;
    return result;
}

double f12(double x, double y, double y1, double y2, double x1, double x2)
{
    double result;
    result = x*(y1-y2) + y*(x2-x1) + x1*y2 - y1*x2;
    return result;
}

double f20(double x, double y, double y0, double y2, double x0, double x2)
{
    double result;
    result = x*(y2-y0) + y*(x0-x2) + x2*y0 -y2*x0;
    return result;
}

std::vector<std::vector<double>> multpilyToArr(std::vector<std::vector<double>> m1, Matrix4 m2)
{
    std::vector<std::vector<double>> result;
    int i, j, k;
    result.resize(3);
    for(i=0 ; i<3 ; ++i){
        result[i].resize(4);
    }
    for(i=0 ; i<3 ; ++i){
        for(j=0 ; j<4 ; ++j){
            for(k=0 ; k<4 ; ++k){
                result[i][j] += m1[i][k] * m2.val[k][j];
            }
        }
    }

    return result;
}

Vec3 multiplyToVec3(std::vector<std::vector<double>> m1, Vec4 m2)
{
    Vec3 result;

    result.x = m1[0][0] * m2.x + m1[0][1] * m2.y + m1[0][2] * m2.z + m1[0][3] * m2.t;
    result.y = m1[1][0] * m2.x + m1[1][1] * m2.y + m1[1][2] * m2.z + m1[1][3] * m2.t;
    result.z = m1[2][0] * m2.x + m1[2][1] * m2.y + m1[2][2] * m2.z + m1[2][3] * m2.t;

    return result;
}

Vec3 crossProductVec3(Vec3 a, Vec3 b)
{
    Vec3 result;

    result.x = a.y * b.z - b.y * a.z;
    result.y = b.x * a.z - a.x * b.z;
    result.z = a.x * b.y - b.x * a.y;

    return result;
}

/*
 * Calculate dot product of vec3 a, vec3 b and return resulting value.
 */
double dotProductVec3(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*
 * Find length (|v|) of vec3 v.
 */
double magnitudeOfVec3(Vec3 v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/*
 * Normalize the vec3 to make it unit vec3.
 */
Vec3 normalizeVec3(Vec3 v)
{
    Vec3 result;
    double d;

    d = magnitudeOfVec3(v);
    result.x = v.x / d;
    result.y = v.y / d;
    result.z = v.z / d;

    return result;
}

/*
 * Return -v (inverse of vec3 v)
 */
Vec3 inverseVec3(Vec3 v)
{
    Vec3 result;
    result.x = -v.x;
    result.y = -v.y;
    result.z = -v.z;

    return result;
}

/*
 * Add vec3 a to vec3 b and return resulting vec3 (a+b).
 */
Vec3 addVec3(Vec3 a, Vec3 b)
{
    Vec3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;

    return result;
}

/*
 * Subtract vec3 b from vec3 a and return resulting vec3 (a-b).
 */
Vec3 subtractVec3(Vec3 a, Vec3 b)
{
    Vec3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}

/*
 * Multiply each element of vec3 with scalar.
 */
Vec3 multiplyVec3WithScalar(Vec3 v, double c)
{
    Vec3 result;
    result.x = v.x * c;
    result.y = v.y * c;
    result.z = v.z * c;

    return result;
}

/*
 * Prints elements in a vec3. Can be used for debugging purposes.
 */
void printVec3(Vec3 v)
{
    cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
}

/*
 * Check whether vec3 a and vec3 b are equal.
 * In case of equality, returns 1.
 * Otherwise, returns 0.
 */
int areEqualVec3(Vec3 a, Vec3 b)
{

    /* if x difference, y difference and z difference is smaller than threshold, then they are equal */
    if ((ABS((a.x - b.x)) < EPSILON) && (ABS((a.y - b.y)) < EPSILON) && (ABS((a.z - b.z)) < EPSILON))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * Returns an identity matrix (values on the diagonal are 1, others are 0).
*/
Matrix4 getIdentityMatrix()
{
    Matrix4 result;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
            {
                result.val[i][j] = 1.0;
            }
            else
            {
                result.val[i][j] = 0.0;
            }
        }
    }

    return result;
}

/*
 * Multiply matrices m1 (Matrix4) and m2 (Matrix4) and return the result matrix r (Matrix4).
 */
Matrix4 multiplyMatrixWithMatrix(Matrix4 m1, Matrix4 m2)
{
    Matrix4 result;
    double total;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            total = 0;
            for (int k = 0; k < 4; k++)
            {
                total += m1.val[i][k] * m2.val[k][j];
            }

            result.val[i][j] = total;
        }
    }

    return result;
}

/*
 * Multiply matrix m (Matrix4) with vector v (vec4) and store the result in vector r (vec4).
 */
Vec4 multiplyMatrixWithVec4(Matrix4 m, Vec4 v)
{
    double values[4];
    double total;

    for (int i = 0; i < 4; i++)
    {
        total = 0;
        for (int j = 0; j < 4; j++)
        {
            total += m.val[i][j] * v.getElementAt(j);
        }
        values[i] = total;
    }

    return Vec4(values[0], values[1], values[2], values[3], v.colorId);
}
