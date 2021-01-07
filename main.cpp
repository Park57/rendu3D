#include <vector>
#include <cmath>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <string>
#include "tgaimage.h"
#include "model.h"
using namespace std;

const int VERTICES = 1258;
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) { 
    bool steep = false; 
    if (std::abs(x0-x1)<std::abs(y0-y1)) { 
        std::swap(x0, y0); 
        std::swap(x1, y1); 
        steep = true; 
    } 
    if (x0>x1) { 
        std::swap(x0, x1); 
        std::swap(y0, y1); 
    } 
    int dx = x1-x0; 
    int dy = y1-y0; 
    int derror2 = std::abs(dy)*2; 
    int error2 = 0; 
    int y = y0; 
    for (int x=x0; x<=x1; x++) { 
        if (steep) { 
            image.set(y, x, color); 
        } else { 
            image.set(x, y, color); 
        } 
        error2 += derror2; 
        if (error2 > dx) { 
            y += (y1>y0?1:-1); 
            error2 -= dx*2; 
        } 
    } 
} 

int main(int argc, char** argv) { 
    TGAImage image(width, height, TGAImage::RGB);
    model = new Model("african_head.obj");
    for (int i=0; i<model->nverts(); i++) { // On parcourt tous les vertices
        vec3 vertice = model->vert(i);
        float x = vertice[0];
        float y = vertice[1];
        float z = vertice[2];
        image.set((x+1.)*width/2.,(y+1)*height/2.,white);
    }
    image.write_tga_file("output.tga");
    delete model;
    return 0;

}