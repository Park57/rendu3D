#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"
#include "tgaimage.h"

class Model {
private:
	std::vector<Vec3f> verts_;
	std::vector<std::vector<Vec3i> > faces_;
	std::vector<Vec2f> uv_; // tableau des coordonnées des textures 
	TGAImage diffusemap_;
	void load_texture(std::string filename, const char *suffix, TGAImage &img);
public:
	Model(const char *filename);
	~Model();
	int nverts();
	int nfaces();
	Vec3f vert(int i);
	std::vector<int> face(int index);
	Vec2i uv(int i, int nthvert);
	TGAColor diffuse(Vec2i uv);
};

#endif //__MODEL_H__