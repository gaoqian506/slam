
#ifndef __WW_CAMERA_HEADER__
#define __WW_CAMERA_HEADER__


#include "Image.h"
#include "Vectors.h"

namespace ww {


struct Intrinsic {
	Intrinsic() : f(500), cx(320), cy(240) {

	}
	double f;
	double cx;
	double cy;
};

class Camera {

public:
	Camera();
	~Camera();
	
	Vec3f unproject(const float& u, const float& v, const float& d);
	Vec3f project(const Vec3f& pt);
	void rotation_warp(Image*& out, bool inverse = true);


	Vec3d epi_point;
	Vec3d pos;
	Vec9d rotation;
	double movement;
	double radius;
	Intrinsic intrinsic;
	
	Image* original;
	Image* gray;
	Image* gradient[2];
	Image* depth;
	Image* points;
	Image* epi_line;
	Image* depth_weight;
	//Image* residual;
};


}

#endif
