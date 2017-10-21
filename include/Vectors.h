#ifndef __WW_VECTORS_HEADER__
#define __WW_VECTORS_HEADER__

#include <memory.h>

namespace ww {

struct Vec3d {

	Vec3d(const double& a = 0, const double& b = 0, const double& c = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
	}
	
	double& operator[](int idx) {
		return val[idx];
	}
	
	double length2() { return val[0]*val[0]+val[1]*val[1]+val[2]*val[2]; }
	double val[3];
};

struct Vec9d {

	Vec9d() {
		memset(val, 0, sizeof(val));
	}
	
	double& operator[](int idx) {
		return val[idx];
	}
	double val[9];
	
};


}

#endif


/*



	void operator=(double v) {
	
		for (int i = 0; i < 9; i++) {
			val[i] = v;
		}
	}


*/
