#include "tgaimage.h"
#include "model.h"
#include <vector>
#include <iostream>
#include <limits>

Model *model = NULL;
int *zbuffer = NULL;
Vec3f light_dir(0, 0, -1); // vecteur de lumière

const int depth = 255;
const int width = 600;
const int height = 600;

extern Matrix ModelView;
Vec3f up(0,1,0);
Vec3f center(0,0,0);
Vec3f eye(1,1,3);

void lookat(Vec3f up, Vec3f center, Vec3f eye){
    Vec3f z = (eye-center).normalize(); // z' is given by the vector ce (do not forget to normalize it, it helps later)
    Vec3f x = cross(up,z).normalize(); //  x'? Simply by a cross product between u and z'
    Vec3f y = cross(x,z).normalize(); // y', such that it is orthogonal to already calculated x' and z'

    //Now it suffices to get any point with coordinates (x,y,z,1) in the model frame, multiply it by the matrix ModelView and we get the coordinates in the camera frame!
    Matrix M = Matrix::identity();
    Matrix T = Matrix::identity();
    for(int i = 0; i < 3; i++){
        M[0][i] = x[i];
        M[1][i] = y[i];
        M[2][i] = z[i];
        T[i][3] = -center[i];
    }
    ModelView = M*T;

}

Matrix viewPort(int x, int y, int w, int h){
    Matrix m = Matrix::identity();

    m[0][0] = w/2.f;
    m[1][1] = h/2.f;
    m[2][2] = depth/2.f;

    m[0][3] = x+w/2.f;
    m[1][3] = y+h/2.f;
    m[2][3] = m[2][2];
    return m;
}

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1))
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;
    int derror2 = std::abs(dy) * 2;
    int error2 = 0;
    int y = y0;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            image.set(y, x, color);
        }
        else
        {
            image.set(x, y, color);
        }
        error2 += derror2;
        if (error2 > dx)
        {
            y += (y1 > y0 ? 1 : -1);
            error2 -= dx * 2;
        }
    }
}

/*
* Calcul du barycentre G du triangle ABC 
*/
Vec3f barycentre(Vec3i A, Vec3i B, Vec3i C)
{
    float xG = (A.x + B.x + C.x) / 3;
    float yG = (A.y + B.y + C.y) / 3;
    float zG = (A.z + B.z + C.z) / 3;
    return Vec3f(xG, yG, zG);
}

Vec3f barycentric(Vec3i A, Vec3i B, Vec3i C, Vec3i P)
{
    Vec3i s[2];
    for (int i = 2; i--;)
    {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

/*void trianglee(Vec3i t0, Vec3i t1, Vec3i t2,TGAColor color, TGAImage &image, TGAImage &zbuffer){
    if(t0.y == t1.y && t0.y == t2.y) // Triangle dégénérés 
        return;
    // Trier les sommets du triangle par leurs coordonnées y
    if (t0.y>t1.y)
        std::swap(t0, t1);
    if (t0.y>t2.y) 
        std::swap(t0, t2);
    if (t1.y>t2.y) 
        std::swap(t1, t2);
    int taille_totale = t2.y - t0.y;  
}*/

void triangle(Vec3i t0, Vec3i t1, Vec3i t2, Vec2i uv0, Vec2i uv1, Vec2i uv2,TGAImage &image, TGAImage &zbuffer)
{
    if (t0.y==t1.y && t0.y==t2.y) return; // i dont care about degenerate triangles
    if (t0.y>t1.y) std::swap(t0, t1);
    if (t0.y>t2.y) std::swap(t0, t2);
    if (t1.y>t2.y) std::swap(t1, t2);
    int total_height = t2.y-t0.y;
    for (int i = 0; i < total_height; i++)
    {
        bool second_half = i>t1.y-t0.y || t1.y==t0.y;
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y;
        float alpha = (float)i / total_height;
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here
        Vec3i A = t0 + (t2-t0)*alpha;
        Vec2i A2 = uv0 +(uv2-uv0)*alpha;
        Vec2i B2 = second_half ? uv1 + (uv2 - uv1) * beta : uv0 + (uv1 - uv0) * beta;
        Vec3i B = second_half ? t1 + (t2-t1)*beta : t0 + (t1-t0)*beta;
        if (A.x > B.x)
            std::swap(A, B);
        if (A2.x > B2.x)
            std::swap(A2, B2);
        for (int j = A.x; j <= B.x; j++)
        {
            float phi = B.x == A.x ? 1. : (float)(j - A.x) / (float)(B.x - A.x);
            float phi2 = B2.x == A2.x ? 1. : (float)(j - A2.x) / (float)(B2.x - A2.x);
            Vec3i P = A + (B - A) * phi;

            Vec2i P2 =  A2 + (B2 - A2) * phi2;
            P2.x = j;
            P2.y = uv0.y + i;
            
            P.x = j;
            P.y = t0.y + i; // a hack to fill holes (due to int cast precision problems)
            int idx = j + (t0.y + i) * width;
            zbuffer.set(P.x, P.y, TGAColor(P.z));
            image.set(P2.x, P2.y, model->diffuse(P2)); // attention, due to int casts t0.y+i != A.y
        }
    }
}

int main(int argc, char **argv)
{
    model = new Model("obj/african_head.obj");

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    for (int i = 0; i < model->nfaces(); i++)
    { // On parcourt tous les triangles
        Vec3i screen_coords[3];
        Vec3f world_coords[3];
        Vec2i uv[3];
        std::vector<int> face = model->face(i);
        for (int j = 0; j < 3; j++)
        {
            uv[j] = model->uv(i, j);
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = Vec3i((v.x + 1.) * width / 2., (v.y + 1.) * height / 2., (v.z + 1.) * depth / 2.);
            world_coords[j] = v;
        }
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]); // cross product
        n.normalize();                                                                         // Normalisation du vecteur n
        float intensity = n * light_dir;
        if (intensity > 0)
        {
            triangle(screen_coords[0], screen_coords[1], screen_coords[2],uv[0], uv[1], uv[2], image, zbuffer);
        }
    }
    image.flip_vertically();
    image.write_tga_file("output.tga");
    zbuffer.flip_vertically();
    zbuffer.write_tga_file("zbuffer.tga");
    delete model;
    return 0;
}