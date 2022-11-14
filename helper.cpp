#include "helper.h"

parser::Vec3f multS(parser::Vec3f v,float s)
{
	parser::Vec3f result;
	result.x = v.x*s;
	result.y = v.y*s;
	result.z = v.z*s;
	return result;
}

parser::Vec3f add(parser::Vec3f v1, parser::Vec3f v2)
{
	parser::Vec3f result;
	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;
	return result;
}

parser::Vec3f CProduct(parser::Vec3f v1, parser::Vec3f v2)
{
	parser::Vec3f result;
	result.x = v1.y*v2.z - v1.z*v2.y;
	result.y = v1.z*v2.x - v1.x*v2.z;
	result.z = v1.x*v2.y - v1.y*v2.x;
	return result;
}

float DProduct(parser::Vec3f v1, parser::Vec3f v2)
{
	float result;
	result = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
	return result;
}

void printVector(parser::Vec3f v)
{
	std::cout << v.x << " " << v.y << " " << v.z << std::endl;
}

parser::Vec3f normalize(parser::Vec3f v)
{
	parser::Vec3f result;
	float l;
	l = sqrt(DProduct(v, v));
	result = multS(v, 1/l);
	return result;
}

float det3x3(float **m)
{
	float result;
	result = m[0][0]*(m[1][1]*m[2][2] - m[2][1]*m[1][2]);
	result -= m[0][1]*(m[1][0]*m[2][2] - m[2][0]*m[1][2]);
	result += m[0][2]*(m[1][0]*m[2][1] - m[2][0]*m[1][1]);
	return result;
}


std::vector<parser::Vec3f> generateRay(int i, int j, int cam, parser::Vec3f q, parser::Vec3f u, parser::Vec3f v, parser::Scene scene)
{
		parser::Vec3f result;
		std::vector<parser::Vec3f> ray;
		float su, sv;
    parser::Vec3f s;

		su = (i+0.5)*(scene.cameras[cam].near_plane.y - scene.cameras[cam].near_plane.x)/scene.cameras[cam].image_width;
		sv = (j+0.5)*(scene.cameras[cam].near_plane.w - scene.cameras[cam].near_plane.z)/scene.cameras[cam].image_height;

    //std::cout << su << " " << sv << std::endl;

    s = add(q, add(multS(u,su), multS(v, -sv)));

    result = scene.cameras[cam].position;
    ray.push_back(result);
    result = add(s, multS(ray[0], -1));
    ray.push_back(result);
		return ray;
}

parser::Vec3f computeColor(int times, parser::Scene scene, std::vector<parser::Vec3f> ray)
{
    int i, j;
    parser::Vec3f c;
    float minT = HUGE_VALF;
    int minI1 = -1, minI2 = -1, minI3 = -1, minI4 = -1;
    float t, blinn;
    parser::Vec3f L, N, H;
    parser::Vec3f intersectPoint;
    parser::Vec3f dlight, slight;
		parser::Vec3f reflect;
		parser::Vec3f tmp;
		std::vector<parser::Vec3f> raytmp;
    float dist;

    c.x = (float)scene.background_color.x;
    c.y = (float)scene.background_color.y;
    c.z = (float)scene.background_color.z;


		//spheres
    for(i=0 ; i<scene.spheres.size() ; ++i){
        t = intersectSphere(scene.spheres[i], scene, ray);
        //std::cout << t << std::endl;
        if(t<minT && t>0){
            //std::cout << materials[spheres[i].material_id-1].ambient.x << " " << materials[spheres[i].material_id-1].ambient.y << " " << materials[spheres[i].material_id-1].ambient.z << std::endl;
            //c = materials[spheres[i].material_id-1].diffuse;
            //printVector(c);
            minI1 = i;
            minT = t;
            //std::cout << "hello" << std::endl;

        }
    }
    if(minI1 != -1){
    		intersectPoint = add(ray[0], multS(ray[1], minT));
        N = multS(add(intersectPoint, multS(scene.vertex_data[scene.spheres[minI1].center_vertex_id-1], -1)), 1/scene.spheres[minI1].radius);
				reflect = multS(N, DProduct(N, ray[1])*-2);
				reflect = normalize(add(ray[1], reflect));
				c.x = scene.ambient_light.x * scene.materials[scene.spheres[minI1].material_id-1].ambient.x;
				c.y = scene.ambient_light.y * scene.materials[scene.spheres[minI1].material_id-1].ambient.y;
				c.z = scene.ambient_light.z * scene.materials[scene.spheres[minI1].material_id-1].ambient.z;
				//printVector(c);
        for(i=0 ; i<scene.point_lights.size() ; ++i){
            L = normalize(add(scene.point_lights[i].position, multS(intersectPoint, -1)));
            H = normalize(add(L, multS(ray[1], -1)));
						if(times!=scene.max_recursion_depth && scene.materials[scene.spheres[minI1].material_id-1].is_mirror){
								raytmp.push_back(add(intersectPoint, multS(N, scene.shadow_ray_epsilon)));
								raytmp.push_back(reflect);
								tmp = computeColor(++times, scene, raytmp);
								//std::cout << times << std::endl;
								tmp.x = tmp.x * scene.materials[scene.spheres[minI1].material_id-1].mirror.x;
								tmp.y = tmp.y * scene.materials[scene.spheres[minI1].material_id-1].mirror.y;
								tmp.z = tmp.z * scene.materials[scene.spheres[minI1].material_id-1].mirror.z;
								c = add(c, tmp);
						}
						if(!noShadow(intersectPoint, L, scene, N, scene.point_lights[i].position)){
								continue;
						}
            if(DProduct(L, N) >= 0){
                dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, DProduct(L, N));
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.spheres[minI1].material_id-1].diffuse.x;
                dlight.y = dlight.y * scene.materials[scene.spheres[minI1].material_id-1].diffuse.y;
                dlight.z = dlight.z * scene.materials[scene.spheres[minI1].material_id-1].diffuse.z;

                c = add(c, dlight);
            }
            if(DProduct(H, N) >= 0){
            		blinn = DProduct(H, N);
            		blinn = pow(blinn, scene.materials[scene.spheres[minI1].material_id-1].phong_exponent);
            		dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, blinn);
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.spheres[minI1].material_id-1].specular.x;
                dlight.y = dlight.y * scene.materials[scene.spheres[minI1].material_id-1].specular.y;
                dlight.z = dlight.z * scene.materials[scene.spheres[minI1].material_id-1].specular.z;

                c = add(c, dlight);
            }
        }
    }


		//triangles
    parser::Vec3f p1, p2, p3;
    for(i=0 ; i<scene.triangles.size() ; ++i){
        t = intersectTri(scene.triangles[i], scene, ray);
        if(t<minT && t>=0){
                        //printVector(c);
            minI2 = i;
            minT = t;
        }
    }
    if(minI2 != -1){
				c.x = scene.ambient_light.x * scene.materials[scene.triangles[minI2].material_id-1].ambient.x;
				c.y = scene.ambient_light.y * scene.materials[scene.triangles[minI2].material_id-1].ambient.y;
				c.z = scene.ambient_light.z * scene.materials[scene.triangles[minI2].material_id-1].ambient.z;
				p1 = scene.vertex_data[scene.triangles[minI2].indices.v0_id-1];
				p2 = scene.vertex_data[scene.triangles[minI2].indices.v1_id-1];
				p3 = scene.vertex_data[scene.triangles[minI2].indices.v2_id-1];
    		intersectPoint = add(ray[0], multS(ray[1], minT));
    		N = normalize(CProduct(add(p2, multS(p1, -1)), add(p3, multS(p1, -1))));
				reflect = multS(N, DProduct(N, ray[1])*-2);
				reflect = normalize(add(ray[1], reflect));
    		for(i=0 ; i<scene.point_lights.size() ; ++i){
            L = normalize(add(scene.point_lights[i].position, multS(intersectPoint, -1)));
            H = normalize(add(L, multS(ray[1], -1)));
						if(times!=scene.max_recursion_depth && scene.materials[scene.triangles[minI2].material_id-1].is_mirror){
								raytmp.push_back(add(intersectPoint, multS(N, scene.shadow_ray_epsilon)));
								raytmp.push_back(reflect);
								tmp = computeColor(++times, scene, raytmp);
								tmp.x = tmp.x * scene.materials[scene.triangles[minI2].material_id-1].mirror.x;
								tmp.y = tmp.y * scene.materials[scene.triangles[minI2].material_id-1].mirror.y;
								tmp.z = tmp.z * scene.materials[scene.triangles[minI2].material_id-1].mirror.z;
								c = add(c, tmp);
						}
						if(!noShadow(intersectPoint, L, scene, N, scene.point_lights[i].position)){
								continue;
						}
            if(DProduct(L, N) >= 0){
                dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, DProduct(L, N));
                //std::cout << DProduct(L, N) << std::endl;
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.triangles[minI2].material_id-1].diffuse.x;
                dlight.y = dlight.y * scene.materials[scene.triangles[minI2].material_id-1].diffuse.y;
                dlight.z = dlight.z * scene.materials[scene.triangles[minI2].material_id-1].diffuse.z;

                c = add(c, dlight);
                //printVector(c);
            }
            if(DProduct(H, N) >= 0){
            		blinn = DProduct(H, N);
            		blinn = pow(blinn, scene.materials[scene.triangles[minI2].material_id-1].phong_exponent);
            		dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, blinn);
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.triangles[minI2].material_id-1].specular.x;
                dlight.y = dlight.y * scene.materials[scene.triangles[minI2].material_id-1].specular.y;
                dlight.z = dlight.z * scene.materials[scene.triangles[minI2].material_id-1].specular.z;

                c = add(c, dlight);
            }
            //printVector(c);
        }
    }




		//meshes
		parser::Triangle tri;
    for(i=0 ; i<scene.meshes.size() ; ++i){
        for(j=0 ; j<scene.meshes[i].faces.size() ; ++j){
            tri.material_id = scene.meshes[i].material_id;
            tri.indices = scene.meshes[i].faces[j];
            t = intersectTri(tri, scene, ray);
            if(t<minT && t>=0){
            		//printVector(c);
                minI3 = i;
                minT = t;
								minI4 = j;
            }
        }
    }
    if(minI3 != -1){
				p1 = scene.vertex_data[scene.meshes[minI3].faces[minI4].v0_id-1];
				p2 = scene.vertex_data[scene.meshes[minI3].faces[minI4].v1_id-1];
				p3 = scene.vertex_data[scene.meshes[minI3].faces[minI4].v2_id-1];
				tri.material_id = scene.meshes[minI3].material_id;
				tri.indices = scene.meshes[minI3].faces[minI4];
    		intersectPoint = add(ray[0], multS(ray[1], minT));
				c.x = scene.ambient_light.x * scene.materials[tri.material_id-1].ambient.x;
				c.y = scene.ambient_light.y * scene.materials[tri.material_id-1].ambient.y;
				c.z = scene.ambient_light.z * scene.materials[tri.material_id-1].ambient.z;
    		N = normalize(CProduct(add(p2, multS(p1, -1)), add(p3, multS(p1, -1))));
				reflect = multS(N, DProduct(N, ray[1])*-2);
				reflect = normalize(add(ray[1], reflect));
				for(i=0 ; i<scene.point_lights.size() ; ++i){
	      		L = normalize(add(scene.point_lights[i].position, multS(intersectPoint, -1)));
	        	H = normalize(add(L, multS(ray[1], -1)));
						if(times!=scene.max_recursion_depth && scene.materials[tri.material_id-1].is_mirror){
								raytmp.push_back(add(intersectPoint, multS(N, scene.shadow_ray_epsilon)));
								raytmp.push_back(reflect);
								tmp = computeColor(++times, scene, raytmp);
								//std::cout << times << std::endl;
								//printVector(tmp);
								tmp.x = tmp.x * scene.materials[tri.material_id-1].mirror.x;
								tmp.y = tmp.y * scene.materials[tri.material_id-1].mirror.y;
								tmp.z = tmp.z * scene.materials[tri.material_id-1].mirror.z;
								c = add(c, tmp);
						}
						if(!noShadow(intersectPoint, L, scene, N, scene.point_lights[i].position)){
								continue;
						}
	        	if(DProduct(L, N) >= 0){
	            	dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
	            	dlight = multS(scene.point_lights[i].intensity, DProduct(L, N));
	            	//std::cout << DProduct(L, N) << std::endl;
	            	dlight = multS(dlight, 1/dist);
	            	dlight.x = dlight.x * scene.materials[scene.meshes[minI3].material_id-1].diffuse.x;
	            	dlight.y = dlight.y * scene.materials[scene.meshes[minI3].material_id-1].diffuse.y;
	            	dlight.z = dlight.z * scene.materials[scene.meshes[minI3].material_id-1].diffuse.z;
	            	//printVector(dlight);
	            	c = add(c, dlight);
	            	//printVector(c);
	        	}
	        	if(DProduct(H, N) >= 0){
            		blinn = DProduct(H, N);
            		blinn = pow(blinn, scene.materials[scene.meshes[minI3].material_id-1].phong_exponent);
            		dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, blinn);
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.meshes[minI3].material_id-1].specular.x;
                dlight.y = dlight.y * scene.materials[scene.meshes[minI3].material_id-1].specular.y;
                dlight.z = dlight.z * scene.materials[scene.meshes[minI3].material_id-1].specular.z;

                c = add(c, dlight);
            }
	        //printVector(c);
    		}
		}
		/*
    c.x = c.x / 255.0;
    c.y = c.y / 255.0;
    c.z = c.z / 255.0;*/
    //printVector(c);
    return c;
}

float intersectSphere(parser::Sphere s, parser::Scene scene, std::vector<parser::Vec3f> ray)
{
    float A, B, C;
    float delta;
    parser::Vec3f c;

    c = scene.vertex_data[s.center_vertex_id-1];
    //std::cout << c.x << " " << c.y << " " << c.z << std::endl;
    float t, t1, t2;

    /*printVector(ray[0]);
    printVector(ray[1]);
    printVector(c);
    std::cout << std::endl;*/

    C = DProduct(add(ray[0],multS(c,-1)), add(ray[0],multS(c,-1))) - s.radius*s.radius;
    B = DProduct(ray[1], add(ray[0],multS(c,-1)))*2;
    A = DProduct(ray[1], ray[1]);

    //std::cout << A << " " << B << " " << C << " " << std::endl;

    delta = B*B - 4*A*C;

    //std::cout << delta << std::endl;

    if(delta < 0){
        return -1;
    }
    if(delta <= std::numeric_limits<float>::epsilon()){
        t = -B / (2*A);
    }else{
        delta = sqrt(delta);
        A = 2*A;
        t1 = (-B + delta) / A;
        t2 = (-B - delta) / A;
        if(t1 < t2){
            t = t1;
        }else{
            t = t2;
        }
    }
    return t;
}

float intersectTri(parser::Triangle t, parser::Scene scene, std::vector<parser::Vec3f> ray)
{
    int i=0;
    parser::Vec3f a, b, c;
    float beta, gamma, result;
    float **m, **A;
    float detA;

    a = scene.vertex_data[t.indices.v0_id-1];
    b = scene.vertex_data[t.indices.v1_id-1];
    c = scene.vertex_data[t.indices.v2_id-1];

    m = new float*[3];
    A = new float*[3];

    for(i=0 ; i<3 ; ++i){
        m[i] = new float[3];
        A[i] = new float[3];
    }
    A[0][0] = a.x - b.x;
    A[0][1] = a.x - c.x;
    A[0][2] = ray[1].x;
    A[1][0] = a.y - b.y;
    A[1][1] = a.y - c.y;
    A[1][2] = ray[1].y;
    A[2][0] = a.z - b.z;
    A[2][1] = a.z - c.z;
    A[2][2] = ray[1].z;

    detA = det3x3(A);

    m[0][0] = a.x - ray[0].x;
    m[0][1] = a.x - c.x;
    m[0][2] = ray[1].x;
    m[1][0] = a.y - ray[0].y;
    m[1][1] = a.y - c.y;
    m[1][2] = ray[1].y;
    m[2][0] = a.z - ray[0].z;
    m[2][1] = a.z - c.z;
    m[2][2] = ray[1].z;

    beta = det3x3(m) / detA;

    m[0][0] = a.x - b.x;
    m[0][1] = a.x - ray[0].x;
    m[0][2] = ray[1].x;
    m[1][0] = a.y - b.y;
    m[1][1] = a.y - ray[0].y;
    m[1][2] = ray[1].y;
    m[2][0] = a.z - b.z;
    m[2][1] = a.z - ray[0].z;
    m[2][2] = ray[1].z;

    gamma = det3x3(m) / detA;

    m[0][0] = a.x - b.x;
    m[0][1] = a.x - c.x;
    m[0][2] = a.x - ray[0].x;
    m[1][0] = a.y - b.y;
    m[1][1] = a.y - c.y;
    m[1][2] = a.y - ray[0].y;
    m[2][0] = a.z - b.z;
    m[2][1] = a.z - c.z;
    m[2][2] = a.z - ray[0].z;

    result = det3x3(m) / detA;

    if(beta+gamma<=1 && beta>=0 && gamma>=0){
        return result;
    }

    return -1;
}

bool noShadow(parser::Vec3f intersectPoint, parser::Vec3f L, parser::Scene scene, parser::Vec3f N, parser::Vec3f lightp)
{
		int i=0, j=0;
		float t;
		float minT;
		std::vector<parser::Vec3f> ray;
		ray.push_back(add(intersectPoint, multS(N, scene.shadow_ray_epsilon)));
		ray.push_back(L);
		if(L.x!=0){
				minT = (lightp.x - intersectPoint.x)/L.x;
		}else if(L.y!=0){
				minT = (lightp.y - intersectPoint.y)/L.y;
		}else{
				minT = (lightp.z - intersectPoint.z)/L.z;
		}
		for(i=0 ; i<scene.spheres.size() ; ++i){
        t = intersectSphere(scene.spheres[i], scene, ray);
				if(t>=0 && t<minT){
						return false;
				}
		}

		for(i=0 ; i<scene.triangles.size() ; ++i){
        t = intersectTri(scene.triangles[i], scene, ray);
				if(t>=0 && t<minT){
						return false;
				}
		}

		for(i=0 ; i<scene.meshes.size() ; ++i){
        for(j=0 ; j<scene.meshes[i].faces.size() ; ++j){
            parser::Triangle tri;
            tri.material_id = scene.meshes[i].material_id;
            tri.indices = scene.meshes[i].faces[j];
            t = intersectTri(tri, scene, ray);
						if(t>=0 && t<minT){
								return false;
						}
				}
		}

		return true;
}

/*parser::Vec3f mirror(int times, parser::Vec3f intersectPoint, parser::Vec3f go, parser::Scene scene)
{
		parser::Vec3f c;
		float t;
		int i, minI1=-1;
		c.x = (float)scene.background_color.x;
    c.y = (float)scene.background_color.y;
    c.z = (float)scene.background_color.z;
		for(i=0 ; i<scene.spheres.size() ; ++i){
        t = intersectSphere(scene.spheres[i], scene, ray);
        //std::cout << t << std::endl;
        if(t<minT && t>0){
            //std::cout << materials[spheres[i].material_id-1].ambient.x << " " << materials[spheres[i].material_id-1].ambient.y << " " << materials[spheres[i].material_id-1].ambient.z << std::endl;
            //c = materials[spheres[i].material_id-1].diffuse;
            c.x = scene.ambient_light.x * scene.materials[scene.spheres[i].material_id-1].ambient.x;
            c.y = scene.ambient_light.y * scene.materials[scene.spheres[i].material_id-1].ambient.y;
            c.z = scene.ambient_light.z * scene.materials[scene.spheres[i].material_id-1].ambient.z;
            //printVector(c);
            minI1 = i;
            minT = t;
            //std::cout << "hello" << std::endl;

        }
    }
		if(minI1 != -1){
				if(times == scene.max_recursion_depth){
						return c;
				}
				intersectPoint = add(ray[0], multS(ray[1], minT));
        N = multS(add(intersectPoint, multS(scene.vertex_data[scene.spheres[minI1].center_vertex_id-1], -1)), 1/scene.spheres[minI1].radius);
				reflect = multS(N, DProduct(N, ray[1])*2);
				reflect = multS(add(ray[1], reflect), -1);
				//printVector(c);
        for(i=0 ; i<scene.point_lights.size() ; ++i){
            L = normalize(add(scene.point_lights[i].position, multS(intersectPoint, -1)));
            H = normalize(add(L, multS(ray[1], -1)));
						if(!noShadow(intersectPoint, L, scene, N)){
								continue;
						}
						if(scene.materials[scene.spheres[minI1].material_id-1].is_mirror){
								c = add(c, mirror(0, intersectPoint, reflect, scene));
						}
            if(DProduct(L, N) >= 0){
                dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, DProduct(L, N));
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.spheres[minI1].material_id-1].diffuse.x;
                dlight.y = dlight.y * scene.materials[scene.spheres[minI1].material_id-1].diffuse.y;
                dlight.z = dlight.z * scene.materials[scene.spheres[minI1].material_id-1].diffuse.z;

                c = add(c, dlight);
            }
            if(DProduct(H, N) >= 0){
            		blinn = DProduct(H, N);
            		blinn = pow(blinn, scene.materials[scene.spheres[minI1].material_id-1].phong_exponent);
            		dist = DProduct(add(scene.point_lights[i].position, multS(intersectPoint, -1)), add(scene.point_lights[i].position, multS(intersectPoint, -1)));
                dlight = multS(scene.point_lights[i].intensity, blinn);
                dlight = multS(dlight, 1/dist);
                dlight.x = dlight.x * scene.materials[scene.spheres[minI1].material_id-1].specular.x;
                dlight.y = dlight.y * scene.materials[scene.spheres[minI1].material_id-1].specular.y;
                dlight.z = dlight.z * scene.materials[scene.spheres[minI1].material_id-1].specular.z;

                c = add(c, dlight);
            }
        }
		}

		return c;
}*/
