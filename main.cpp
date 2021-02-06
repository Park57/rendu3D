#include <vector>
#include <limits>
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
const int width  = 650;
const int height = 650;

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
vec3 barycentric(vec3 A, vec3 B, vec3 C, vec3 P) {
    vec3 s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    vec3 u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return vec3(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return vec3(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void dtriangle(vec2 t0, vec2 t1, vec2 t2, TGAImage &image, TGAColor color) { 
    if (t0.y==t1.y && t0.y==t2.y) return; // On évite les duplications
    // on trie les sommets par ordre croissant
    if (t0.y>t1.y) std::swap(t0, t1); 
    if (t0.y>t2.y) std::swap(t0, t2); 
    if (t1.y>t2.y) std::swap(t1, t2); 
    int total_height = t2.y-t0.y;  // taille totale
    for (int i=0; i<total_height; i++) { 
       bool second_half = i>=t1.y-t0.y; 
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y; 
        float alpha = (float)i/total_height; 
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here 
        vec2 A = t0 + (t2-t0)*alpha; 
        vec2 B = second_half ? t1 + (t2-t1)*beta : t0 + (t1-t0)*beta; 
        if (A.x>B.x) std::swap(A, B); 
        for (int j=A.x; j<=B.x; j++) { 
            image.set(j, t0.y+i, color); // attention, due to int casts t0.y+i != A.y 
        } 
    } 
}
void dttriangle(vec3 *pts, float *zbuffer, TGAImage &image, TGAColor color) { 
    vec2 bboxmin(image.get_width()-1,  image.get_height()-1); 
    vec2 bboxmax(0, 0); 
    vec2 clamp(image.get_width()-1, image.get_height()-1); 
    for (int i=0; i<3; i++) { 
        for (int j=0; j<2; j++) { 
            bboxmin[j] = std::max(0.,std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
        } 
    } 
    vec3 P;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            vec3 bc_screen  = barycentric(pts[0], pts[1], pts[2], P);
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
            P.z = 0;
            for (int i=0; i<3; i++) P.z += pts[i][2]*bc_screen[i];
            if (zbuffer[int(P.x+P.y*width)]<P.z) {
                zbuffer[int(P.x+P.y*width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

vec3 world2screen(vec3 v) {
    return vec3(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z); //  On transforme les coord du monde reelle en coord de l'ecran 
}

int main(int argc, char** argv) { 

    float *zbuffer = new float[width*height]; //buffer de 650*650
    for (int i=width*height; i--; zbuffer[i] = -std::numeric_limits<double>::max());// On rempli le buffer de -1.79769e +308
    TGAImage image(width, height, TGAImage::RGB);
    vec3 light_dir(0,0,-1);
    model = new Model("african_head.obj");
    for (int i=0; i<model->nfaces()*3; i+=3) {  // On parcourt tous les triangles 
        std::vector<int> face = model->face(i);// face (i) return un vecteur d'entiers face = 24,25,26 pour la première boucle 
        vec3 world[3];
        for (int j=0; j<3; j++) world[j] = world2screen(model->vert(face[j])); // pour chaque tour de boucle on transforme les coord word en coord screen
        vec3 n = cross((world[2]-world[0]),(world[1]-world[0])); // cross product
        n.normalize(); // Norme du vecteur 
        float intensity = n*light_dir; // norme du vecteur * le vecteur d'intensité
        if(intensity>0) // si produit scalaire négatif on ne dessine pas le triangle*/
        dttriangle(world, zbuffer, image,TGAColor(intensity*255, intensity*255, intensity*255, 255));
    }
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
