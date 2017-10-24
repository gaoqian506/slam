#ifndef __WW_VECTORS_HEADER__
#define __WW_VECTORS_HEADER__

#include <memory.h>

namespace ww {

struct Vec2f {

	Vec2f(const float& a = 0, const float& b = 0) {
		val[0] = a;
		val[1] = b;
	}
	
	float& operator[](int idx) {
		return val[idx];
	}
	
	float length2() { return val[0]*val[0]+val[1]*val[1]; }	
	float val[2];
};

struct Vec3f {

	Vec3f(const float& a = 0, const float& b = 0, const float& c = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
	}
	
	float& operator[](int idx) {
		return val[idx];
	}

	
	float length2() { return val[0]*val[0]+val[1]*val[1]+val[2]*val[2]; }
	float val[3];
};

struct Vec4f {

	Vec4f(const float& a = 0, const float& b = 0, const float& c = 0, const float& d = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
		val[3] = d;
	}
	
	float& operator[](int idx) {
		return val[idx];
	}
	
	Vec4f& operator/=(const float& d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		val[3] /= d;
		return *this;
	}
	
	float val[4];
};



struct Vec3d {

	Vec3d(const double& a = 0, const double& b = 0, const double& c = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
	}
	
	double& operator[](int idx) {
		return val[idx];
	}
	
	const double& operator[](int idx) const {
		return val[idx];
	}
	
	Vec3d& operator+=(const Vec3d& right) {
	
		val[0] += right[0];
		val[1] += right[0];
		val[2] += right[0];
		return *this;
	}
	Vec3d& operator/=(const double& d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		return *this;
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
	
	Vec9d& operator/=(const double& d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		val[3] /= d;
		val[4] /= d;
		val[5] /= d;
		val[6] /= d;
		val[7] /= d;
		val[8] /= d;
		return *this;
	}
	
	Vec9d& operator=(const double& d) {
	
		val[0] = d;
		val[1] = d;
		val[2] = d;
		val[3] = d;
		val[4] = d;
		val[5] = d;
		val[6] = d;
		val[7] = d;
		val[8] = d;
		return *this;
	}
	
	double* ptr() { return val; }
	
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
