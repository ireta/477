#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "helper.h"

typedef unsigned char RGB[3];

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    //
    // Normally, you would be running your ray tracing
    // code here to produce the desired image.

    const RGB BAR_COLOR[8] =
    {
        { 255, 255, 255 },  // 100% White
        { 255, 255,   0 },  // Yellow
        {   0, 255, 255 },  // Cyan
        {   0, 255,   0 },  // Green
        { 255,   0, 255 },  // Magenta
        { 255,   0,   0 },  // Red
        {   0,   0, 255 },  // Blue
        {   0,   0,   0 },  // Black
    };
/*
    int width = 640, height = 480;
    int columnWidth = width / 8;

    unsigned char* image = new unsigned char [width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }
*/


    int cam=0;

    for(cam=0 ; cam<scene.cameras.size() ; ++cam){
        parser::Vec3f m, q, gaze, u, v, w;
        int i=0, j=0;
        int imgID=0;
        float dist = scene.cameras[cam].near_distance;
        int width = scene.cameras[cam].image_width, height = scene.cameras[cam].image_height;
        unsigned char* image = new unsigned char[width * height * 3];
        /*for(i=0 ; i<height ; ++i){
            for(j=0 ; j<width ; ++j){
                image[imgID++] = scene.background_color.x;
                image[imgID++] = scene.background_color.y;
                image[imgID++] = scene.background_color.z;
            }
        }*/

        v = scene.cameras[cam].up;
        gaze = scene.cameras[cam].gaze;
        w = multS(gaze, -1);
        u = CProduct(v, w);
        //u = parser::multS(u, -1);
        m = add(scene.cameras[cam].position, multS(gaze, dist));
        q = add(m, add(multS(u, scene.cameras[cam].near_plane.x), multS(v, scene.cameras[cam].near_plane.w)));
        //std::cout << height << " " << width << std::endl;
        imgID = 0;
        for(j=0 ; j<height ; ++j){
            for(i=0 ; i<width ; ++i){
                //parser::Vec3f pixel;
                parser::Vec3f raycolor;
                std::vector<parser::Vec3f> ray;
                float l, r, b, t;
                parser::Vec3f pos;
                l = scene.cameras[cam].near_plane.x;
                r = scene.cameras[cam].near_plane.y;
                b = scene.cameras[cam].near_plane.z;
                t = scene.cameras[cam].near_plane.w;
                pos = scene.cameras[cam].position;
                ray = generateRay(i, j, q, u, v, l, r, b, t, pos, width, height);
                //std::cout << l  << " " << r << " " << b << " " << t << std::endl;
                //std::cout << j  << " " << i << std::endl;
                //pixel = parser::add(scene.ray[0], parser::multS(scene.ray[1], 4));
                //std::cout << std::endl;
                raycolor = computeColor(0, scene, ray);
                //std::cout << raycolor.x << " " << raycolor.y << " " << raycolor.z << std::endl;
                //parser::printVector(pixel);
                if(raycolor.x > 255){
                    raycolor.x = 255;
                }
                if(raycolor.y > 255){
                    raycolor.y = 255;
                }
                if(raycolor.z > 255){
                    raycolor.z = 255;
                }
                //std::cout << raycolor.x << " " << raycolor.y << " " << raycolor.z << std::endl;
                image[imgID++] = raycolor.x;
                image[imgID++] = raycolor.y;
                image[imgID++] = raycolor.z;
            }
        }

        write_ppm(scene.cameras[cam].image_name.c_str(), image, width, height);
    }
}
