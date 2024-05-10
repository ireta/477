#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

#define PI 3.14159265

using namespace tinyxml2;
using namespace std;

/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/


void Scene::forwardRenderingPipeline(Camera *camera)
{
		Matrix4 camera_M;
		Matrix4 rotation, translation, scaling;
		int i, l, m, hor, ver;
		double bary1, bary2, bary3;
		double x0, x1, x2, y0, y1, y2, xi, xf, yi, yf, zi, zf, dx, dy,dz;
		int arrtest1, arrtest2;

		/*std::cout << camera->u.x << " " << camera->u.y << " " << camera->u.z << std::endl;
		std::cout << camera->v.x << " " << camera->v.y << " " << camera->v.z << std::endl;
		std::cout << camera->w.x << " " << camera->w.y << " " << camera->w.z << std::endl;
		std::cout << std::endl;*/
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
    camera_M.val[3][3] = 1;
		//printMatrix4(camera_M);

		/*std::vector<int> hash_array; //i am so gonna regret this
		hash_array.resize(vertices.size());*/
		Matrix4 tmp;
		std::vector<Vec3> result;
		result.resize(3);
		Vec4 values;
		std::vector<std::vector<double>> vp;
		vp.resize(3);
		vp[0].resize(4);
		vp[1].resize(4);
		vp[2].resize(4);
		vp[0][0] = camera->horRes/2.0;
		vp[0][3] = (camera->horRes-1)/2.0;
		vp[1][1] = camera->verRes/2.0;
		vp[1][3] = (camera->verRes-1)/2.0;
		vp[2][2] = 0.5;
		vp[2][3] = 0.5;
		Matrix4 orth;
		orth.val[0][0] = 2.0/(camera->right - camera->left);
		orth.val[0][3] = -(camera->right + camera->left)/(camera->right - camera->left);
		orth.val[1][1] = 2.0/(camera->top - camera->bottom);
		orth.val[1][3] = -(camera->top + camera->bottom)/(camera->top - camera->bottom);
		orth.val[2][2] = -2.0/(camera->far - camera->near);
		orth.val[2][3] = -(camera->far + camera->near)/(camera->far - camera->near);
		orth.val[3][3] = 1;
		//printMatrix4(orth);
		Matrix4 p2o;
		p2o.val[0][0] = camera->near;
		p2o.val[1][1] = camera->near;
		p2o.val[2][2] = camera->far + camera->near;
		p2o.val[2][3] = camera->far * camera->near;
		p2o.val[3][2] = -1.0;
		Matrix4 per;
		per = multiplyMatrixWithMatrix(orth, p2o);
		//printMatrix4(per);
		for(i=0 ; i<meshes.size() ; ++i){
				rotation = getIdentityMatrix();
				translation = getIdentityMatrix();
				scaling = getIdentityMatrix();
				for(l=0 ; l<meshes[i]->numberOfTransformations ; ++l){
						tmp = getIdentityMatrix();
						if(meshes[i]->transformationTypes[l] == 't'){
								for(m=0 ; m<translations.size() ; ++m){
										if(translations[m]->translationId == meshes[i]->transformationIds[l]){
												break;
										}
								}
								tmp.val[0][3] = translations[m]->tx;
								tmp.val[1][3] = translations[m]->ty;
								tmp.val[2][3] = translations[m]->tz;
								translation = multiplyMatrixWithMatrix(translation, tmp);
						}else if(meshes[i]->transformationTypes[l] == 's'){
								for(m=0 ; m<scalings.size() ; ++m){
										if(scalings[m]->scalingId == meshes[i]->transformationIds[l]){
												break;
										}
								}
								tmp.val[0][0] = scalings[m]->sx;
								tmp.val[1][1] = scalings[m]->sy;
								tmp.val[2][2] = scalings[m]->sz;
								scaling = multiplyMatrixWithMatrix(scaling, tmp);
						}else{
								for(m=0 ; m<rotations.size() ; ++m){
										if(rotations[m]->rotationId == meshes[i]->transformationIds[l]){
												break;
										}
								}
								Matrix4 alfa, betha, reverse_alfa, reverse_betha, rotate;
								double a = rotations[m]->ux, b = rotations[m]->uy, c = rotations[m]->uz;
								double d = sqrt(pow(c, 2) + pow(b, 2));
								double sinalfa, cosalfa;
								cosalfa = cos(rotations[m]->angle*PI/180.0);
								sinalfa = sin(rotations[m]->angle*PI/180.0);
								alfa.val[0][0] = 1;
								alfa.val[1][1] = c/d;
								alfa.val[1][2] = -b/d;
								alfa.val[2][1] = b/d;
								alfa.val[2][2] = c/d;
								alfa.val[3][3] = 1;

								reverse_alfa.val[0][0] = 1;
								reverse_alfa.val[1][1] = c/d;
								reverse_alfa.val[1][2] = b/d;
								reverse_alfa.val[2][1] = -b/d;
								reverse_alfa.val[2][2] = c/d;
								reverse_alfa.val[3][3] = 1;

								betha.val[0][0] = d;
								betha.val[0][2] = -a;
								betha.val[1][1] = 1;
								betha.val[2][0] = a;
								betha.val[2][2] = d;
								betha.val[3][3] = 1;

								reverse_betha.val[0][0] = d;
								reverse_betha.val[0][2] = a;
								reverse_betha.val[1][1] = 1;
								reverse_betha.val[2][0] = -a;
								reverse_betha.val[2][2] = d;
								reverse_betha.val[3][3] = 1;

								rotate.val[0][0] = cosalfa;
								rotate.val[0][1] = -sinalfa;
								rotate.val[1][0] = sinalfa;
								rotate.val[1][1] = cosalfa;
								rotate.val[2][2] = 1;
								rotate.val[3][3] = 1;

								tmp = multiplyMatrixWithMatrix(reverse_betha, alfa);
								tmp = multiplyMatrixWithMatrix(rotate, tmp);
								tmp = multiplyMatrixWithMatrix(betha, tmp);
								tmp = multiplyMatrixWithMatrix(reverse_alfa, tmp);
								rotation = multiplyMatrixWithMatrix(rotation, tmp);
						}
				}
				//printMatrix4(transformation_matrix);
				//std::cout << meshes[i]->numberOfTriangles << std::endl;

				for(l=0 ; l<meshes[i]->numberOfTriangles ; ++l){
						for(m=0 ; m<3 ; ++m){
								values.x = vertices[meshes[i]->triangles[l].vertexIds[m]-1]->x;
								values.y = vertices[meshes[i]->triangles[l].vertexIds[m]-1]->y;
								values.z = vertices[meshes[i]->triangles[l].vertexIds[m]-1]->z;
								values.t = 1;
								//std::cout << values.x << " " << values.y << " " << values.z << std::endl;
								values = multiplyMatrixWithVec4(rotation, values);
								values = multiplyMatrixWithVec4(translation, values);
								values = multiplyMatrixWithVec4(scaling, values);
								//std::cout << values.x << " " << values.y << " " << values.z << std::endl;
								//std::cout << std::endl;
								values = multiplyMatrixWithVec4(camera_M, values);
								if(camera->projectionType){
										values = multiplyMatrixWithVec4(per, values);
										//std::cout << values.x << " " << values.y << " " << values.z << " " << values.t << std::endl;
										if(values.t != 1){
												values = multiplyWithScalar(values, values.t);
												values.t = 1.0;
										}
								}else{
										values = multiplyMatrixWithVec4(orth, values);
								}
								//std::cout << values.t << std::endl;
								result[m] = multiplyToVec3(vp, values);
								//std::cout << result.x << " " << result.y << " " << result.z << std::endl
						}
						Vec3 N;
						Vec3 center;
						center.x = (result[0].x + result[1].x + result[2].x)/3;
						center.y = (result[0].y + result[1].y + result[2].y)/3;
						center.z = (result[0].z + result[1].z + result[2].z)/3;
						N = normalizeVec3(crossProductVec3(subtractVec3(result[1], result[0]), subtractVec3(result[2], result[0])));
						/*std::cout << camera->cameraId << " " << camera->v.x << " " << camera->v.y << " " << camera->v.z << std::endl;
						std::cout << std::endl;*/
						if(!cullingEnabled || (cullingEnabled && ((dotProductVec3(N, center)>0 && camera->projectionType) || (dotProductVec3(N, center)<0 && !camera->projectionType)))){
								if(meshes[i]->type){
										x0 = result[0].x;
										x1 = result[1].x;
										x2 = result[2].x;
										y0 = result[0].y;
										y1 = result[1].y;
										y2 = result[2].y;
										//std::cout << result[0].z << " " << result[1].z << " " << result[2].z << " " << std::endl;

										for(hor=fmax(fmin(fmin(x0, x1), x2), 0) ; hor<fmin(fmax(fmax(x0, x1), x2), camera->horRes) ; ++hor){
												for(ver=fmax(fmin(fmin(y0, y1), y2), 0) ; ver<fmin(fmax(fmax(y0, y1), y2), camera->verRes) ; ++ver){
														bary1 = f12(hor,ver, y1, y2, x1, x2)/f12(x0, y0, y1, y2, x1, x2);
														bary2 = f20(hor, ver, y0, y2, x0, x2)/f20(x1, y1, y0, y2, x0, x2);
														bary3 = f01(hor, ver, y0, y1, x0, x1)/f01(x2, y2, y0, y1, x0, x1);

														if(bary1>=0 && bary2>=0 && bary3>=0){
																//std::cout << "hello" << std::endl;
																image[hor][ver].r = bary1*colorsOfVertices[meshes[i]->triangles[l].vertexIds[0]-1]->r;
																image[hor][ver].r += bary2*colorsOfVertices[meshes[i]->triangles[l].vertexIds[1]-1]->r;
																image[hor][ver].r += bary3*colorsOfVertices[meshes[i]->triangles[l].vertexIds[2]-1]->r;

																image[hor][ver].g = bary1*colorsOfVertices[meshes[i]->triangles[l].vertexIds[0]-1]->g;
																image[hor][ver].g += bary2*colorsOfVertices[meshes[i]->triangles[l].vertexIds[1]-1]->g;
																image[hor][ver].g += bary3*colorsOfVertices[meshes[i]->triangles[l].vertexIds[2]-1]->g;

																image[hor][ver].b = bary1*colorsOfVertices[meshes[i]->triangles[l].vertexIds[0]-1]->b;
																image[hor][ver].b += bary2*colorsOfVertices[meshes[i]->triangles[l].vertexIds[1]-1]->b;
																image[hor][ver].b += bary3*colorsOfVertices[meshes[i]->triangles[l].vertexIds[2]-1]->b;
														}
												}
										}

								}else{
										double tE, tL;
										double d;
										double slope;
										Color c;
										Color dc;
										/*for(m=0 ; m<3 ; ++m){
												tE = 0;
												tL = 1;
												xf = result[(m+1)%3].x;
												xi = result[m].x;
												yf = result[(m+1)%3].y;
												yi = result[m].y;
												zf = result[(m+1)%3].z;
												zi = result[m].z;
												dx = xf - xi;
												dy = yf - yi;
												dz = zf - zi;
												if(visible(dx, -1 - xi, tE, tL)){
														if(visible(-dx, xi - 1, tE, tL)){
																if(visible(dy, -1 - yi , tE, tL)){
																		if(visible(-dy, yi - 1, tE, tL)){
																				if(visible(dz, -1 - zi, tE, tL)){
																						if(visible(-dz, zi - 1, tE, tL)){
																								if(tL < 1){
																										result[(m+1)%3].x = xi + dx*tL;
																										result[(m+1)%3].y = yi + dy*tL;
																										result[(m+1)%3].z = zi + dz*tL;
																								}
																								if(tE > 0){
																										result[m].x = xi + dx*tE;
																										result[m].y = yi + dy*tE;
																										result[m].z = zi + dz*tE;
																								}
																						}
																				}
																		}
																}
														}
												}
										}*/
										for(m=0 ; m<3 ; ++m){
												if(result[(m+1)%3].x < result[m].x){
														xi = result[(m+1)%3].x;
														yi = result[(m+1)%3].y;
														xf = result[m].x;
														yf = result[m].y;
														arrtest1 = (m+1)%3;
														arrtest2 = m;
												}else{
														xi = result[m].x;
														yi = result[m].y;
														xf = result[(m+1)%3].x;
														yf = result[(m+1)%3].y;
														arrtest1 = m;
														arrtest2 = (m+1)%3;
												}
												//std::cout << yf << " " << yi << " " << xf << " " << xi << std::endl;
												if(xf != xi){
														slope = (yf - yi)/(xf - xi);
												}else{
														if(yf > yi){
																slope = HUGE_VAL;
														}else{
																slope = -HUGE_VAL;
														}
												}
												if(slope>0 && slope<=1){
														//std::cout << yf << " " << yi << " " << xf << " " << xi << std::endl;
														ver = yi;
														d = (yi - yf) + 0.5*(xf-xi);
														c.r = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->r;
														c.g = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->g;
														c.b = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->b;
														dc.r = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->r - c.r)/(xf-xi);
														dc.g = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->g - c.g)/(xf-xi);
														dc.b = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->b - c.b)/(xf-xi);
														if((xi<0 && xf<0) || (xi>=camera->horRes && xf>=camera->horRes)){
																continue;
														}
														for(hor=xi ; hor<fmin(xf, camera->horRes) ; ++hor){
																if(ver >= camera->verRes){
																		break;
																}
																if(hor>=0 && ver>=0){
																		image[hor][ver].r = c.r;
																		image[hor][ver].g = c.g;
																		image[hor][ver].b = c.b;
																}
																if(d < 0){
																		++ver;
																		d += (yi - yf) + (xf - xi);
																}else{
																		d += (yi - yf);
																}
																c.r += dc.r;
																c.g += dc.g;
																c.b += dc.b;
														}
												}else if(slope>1){
														hor = xi;
														d = (xi - xf) + 0.5*(yf-yi);
														c.r = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->r;
														c.g = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->g;
														c.b = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->b;
														dc.r = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->r - c.r)/(yf-yi);
														dc.g = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->g - c.g)/(yf-yi);
														dc.b = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->b - c.b)/(yf-yi);
														if((yi<0 && yf<0) || (yi>=camera->verRes && yf>=camera->verRes)){
																continue;
														}
														for(ver=yi ; ver<fmin(yf, camera->verRes) ; ++ver){
																if(hor >= camera->verRes){
																		break;
																}
																if(hor>=0 && ver>=0){
																		image[hor][ver].r = c.r;
																		image[hor][ver].g = c.g;
																		image[hor][ver].b = c.b;
																}
																if(d < 0){
																		++hor;
																		d += (xi - xf) + (yf - yi);
																}else{
																		d += (xi - xf);
																}
																c.r += dc.r;
																c.g += dc.g;
																c.b += dc.b;
														}
												}else if(slope<0 && slope>=-1){
														ver = yi;
														d = (yf - yi) + 0.5*(xf-xi);
														c.r = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->r;
														c.g = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->g;
														c.b = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->b;
														dc.r = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->r - c.r)/(xf-xi);
														dc.g = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->g - c.g)/(xf-xi);
														dc.b = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->b - c.b)/(xf-xi);
														if((xi<0 && xf<0) || (xi>=camera->horRes && xf>=camera->horRes)){
																continue;
														}
														for(hor=xi ; hor<fmin(xf, camera->horRes) ; ++hor){
																if(ver < 0){
																		break;
																}
																if(hor>=0 && ver>=0){
																		image[hor][ver].r = c.r;
																		image[hor][ver].g = c.g;
																		image[hor][ver].b = c.b;
																}
																if(d < 0){
																		--ver;
																		d += (yf - yi) + (xf - xi);
																}else{
																		d += (yf - yi);
																}
																c.r += dc.r;
																c.g += dc.g;
																c.b += dc.b;
														}
												}else if(slope<-1){
														hor = xf;
														d = (xf - xi) + 0.5*(yi-yf);
														c.r = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->r;
														c.g = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->g;
														c.b = colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest2]-1]->b;
														dc.r = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->r - c.r)/(yi-yf);
														dc.g = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->g - c.g)/(yi-yf);
														dc.b = (colorsOfVertices[meshes[i]->triangles[l].vertexIds[arrtest1]-1]->b - c.b)/(yi-yf);
														if((yi<0 && yf<0) || (yi>=camera->verRes && yf>=camera->verRes)){
																continue;
														}
														for(ver=yf ; ver<fmin(yi, camera->verRes) ; ++ver){
																if(hor < 0){
																		break;
																}
																if(hor>=0 && ver>=0){
																		image[hor][ver].r = c.r;
																		image[hor][ver].g = c.g;
																		image[hor][ver].b = c.b;
																}
																if(d > 0){
																		--hor;
																		d += (xf - xi) + (yf - yi);
																}else{
																		d += (xf - xi);
																}
																c.r += dc.r;
																c.g += dc.g;
																c.b += dc.b;
														}
												}
										}
								}
						}
				}
		}
}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL) {
		str = pElement->GetText();

		if (strcmp(str, "enabled") == 0) {
			cullingEnabled = true;
		}
		else {
			cullingEnabled = false;
		}
	}

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		// read projection type
		str = pCamera->Attribute("type");

		if (strcmp(str, "orthographic") == 0) {
			cam->projectionType = 0;
		}
		else {
			cam->projectionType = 1;
		}

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read meshes
	pElement = pRoot->FirstChildElement("Meshes");

	XMLElement *pMesh = pElement->FirstChildElement("Mesh");
	XMLElement *meshElement;
	while (pMesh != NULL)
	{
		Mesh *mesh = new Mesh();

		pMesh->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = pMesh->Attribute("type");

		if (strcmp(str, "wireframe") == 0) {
			mesh->type = 0;
		}
		else {
			mesh->type = 1;
		}

		// read mesh transformations
		XMLElement *pTransformations = pMesh->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *clone_str;
		int v1, v2, v3;
		XMLElement *pFaces = pMesh->FirstChildElement("Faces");
        str = pFaces->GetText();
		clone_str = strdup(str);

		row = strtok(clone_str, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);

			if (result != EOF) {
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		meshes.push_back(mesh);

		pMesh = pMesh->NextSiblingElement("Mesh");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		(void)system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}
