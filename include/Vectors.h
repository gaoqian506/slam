#ifndef __WW_VECTORS_HEADER__
#define __WW_VECTORS_HEADER__

namespace ww {



struct Vec3d {

	Vec3d(const double& a, const double& b, const double& c) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
	}

	double val[3];
};



}

#endif
