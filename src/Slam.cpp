

#include "Slam.h"
#include "MatrixToolbox.h"
#include "SpaceToolbox.h"
#include "CvImage.h"
#include "Config.h"
#include <iostream>
#include <memory.h>
#include <cmath>
#include <stdio.h>


#define SAMPLE_2D( v0, v1, v2, v3, a, b) \
	( v0*(1-a)*(1-b)+v1*a*(1-b)+v2*(1-a)*b+v3*a*b)


namespace ww {


Slam::Slam(VideoSource* vs) : m_working(false), m_camera_count(0), m_key(NULL), m_frame(NULL),  m_changed(false), m_width(0), m_height(0) {

	std::cout << "Slam::Slam" << std::endl;
	
	m_source = vs;
	memset(m_cameras, 0, sizeof(Camera*) * MAX_STATIC_CAMERA_COUNT);
	
	m_mask = NULL;
	m_residual = NULL;
	m_gradient = NULL;
	m_depth = NULL;
	m_iuux = NULL;
	m_debug_image = NULL;

	m_pixel_info[0] = 0;
	
}

void Slam::start() {


	std::cout << "Slam::start" << std::endl;
	if (Config::manually_content) { return; }
	m_working = true;

	//for(int i = 0; i < 10; i++) {
	//	push();
	//}
	
	Image* image = NULL;
	Image* resized = NULL;
	while(m_source->read(image) && m_working) {
		image->resize(resized);
		initialize(resized);
		push(resized);
	}
	m_changed = false;
	if (image) { delete image; }
	if (resized) { delete resized; }
	
	std::cout << "leave Slam::start" << std::endl;

}

void Slam::stop() {


	std::cout << "Slam::stop" << std::endl;
	m_working = false;

}

Camera** Slam::get_cameras() {

	std::cout << "Slam::get_cameras" << std::endl;
	return m_cameras;

}

int Slam::get_camera_count() {

	std::cout << "Slam::get_camera_count" << std::endl;
	return m_camera_count;
}

Camera* Slam::get_current_frame() {
	std::cout << "Slam::get_current_frame" << std::endl;
	return m_frame;
}

bool Slam::changed() {

	return m_changed;
}

Image* Slam::get_debug_image(const int& idx) {

	Image* which = NULL;

	if (idx == 0) {
		which = m_key ? m_key->gray : NULL;
	}
	else if (idx == 1) {
		which = m_frame ? m_frame->gray : NULL;
	}
	else if (idx == 2) {
		which = m_residual;
	}
	else if (idx == 3) {
		which = m_mask;
	}
	else if (idx == 4) {
		which = m_key ? m_key->depth : NULL;
	}
	else {
		which = NULL;
	}

	if (which) {
		double min, max, scale;
		which->min_max(&min, &max);
		scale = 1./(max-min);
		which->convert_to(m_debug_image, Image::Float32, scale, -min/(max-min));
		return m_debug_image;
	}
	else {
		return NULL;
	}

	
		

	//m_residual;
	//assert(0);
	//return NULL;
}

void Slam::push_manauly() {

	std::cout << "Slam::push_manauly" << std::endl;

	//m_working = true;
	Image* image = NULL;
	Image* resized = NULL;
	if(m_source->read(image)) {
		image->resize(resized);
		initialize(resized);
		push(resized);
	}
	//m_changed = false;
	if (image) { delete image; }
	if (resized) { delete resized; }
	
	std::cout << "leave Slam::start" << std::endl;

}

void Slam::func_manualy(int idx) {

	switch (idx) {

	case 1:
		if (m_frame) {
			prepare_residual();
			m_frame->pos += calc_delta_t();
		}
		break;
	case 2:
		if (m_frame) {
			prepare_residual();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_delta_r());
		}
		break;
	case 3:
		if (m_frame) {
			prepare_residual();
			m_frame->pos += calc_delta_t();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_delta_r());
		}
		break;
	case 4:
		if (m_frame) {
			prepare_residual();
			update_keyframe(NULL);
		}
		break;
	}
}


char* Slam::pixel_info(const Vec2d& u) {
	
	
	if (u[0] < 0 || u[0] >= m_width || u[1] < 0 || u[1] >= m_height) {
		return NULL;
	}
	int idx = ((int)u[1]) * m_width + ((int)u[0]);
	sprintf(m_pixel_info, 
		"pos:%f, %f\n"
		"key:%f cur:%f res:%f mask:%d\n"
		"grad: %f, %f\n"
		"depth: %f\n"
		"t: %f, %f, %f\n"
		"R: %f, %f, %f\n"
		"   %f, %f, %f\n"
		"   %f, %f, %f\n",
		u[0], u[1],
		m_key ? *((float*)m_key->gray->at(idx)) : 0.0,
		m_frame ? *((float*)m_frame->gray->at(idx)) : 0.0,
		*((float*)m_residual->at(idx)),
		*((unsigned char*)m_mask->at(idx)),
		((Vec2f*)m_gradient->at(idx))[0][0],
		((Vec2f*)m_gradient->at(idx))[0][1],
		m_key ? *((float*)m_key->depth->at(idx)) : 0.0,
		m_frame ? m_frame->pos[0] : 0,
		m_frame ? m_frame->pos[1] : 0,
		m_frame ? m_frame->pos[2] : 0,
		m_frame ? m_frame->rotation[0] : 0,
		m_frame ? m_frame->rotation[1] : 0,
		m_frame ? m_frame->rotation[2] : 0,
		m_frame ? m_frame->rotation[3] : 0,
		m_frame ? m_frame->rotation[4] : 0,
		m_frame ? m_frame->rotation[5] : 0,
		m_frame ? m_frame->rotation[6] : 0,
		m_frame ? m_frame->rotation[7] : 0,
		m_frame ? m_frame->rotation[8] : 0
	);
	//printf("%s", m_pixel_info);
	return m_pixel_info;
}

void Slam::initialize(Image* image) {

	if (m_mask) { return; }

	m_width = image->width();
	m_height = image->height();

	int w = m_width;
	int h = m_height;
	
	m_mask = new CvImage(w, h, Image::UByte);
	//m_points = new CvImage(w, h, Image::Float32, 4);
	m_residual = new CvImage(w, h, Image::Float32);
	m_gradient = new CvImage(w, h, Image::Float32, 2);
	m_depth = new CvImage(w, h, Image::Float32);
	//m_iuux = new CvImage(w, h, Image::Float32, 3);
}

void Slam::push(Image* image) {

	std::cout << "Slam::push" << std::endl;
	preprocess(image);
	update_pose();
	update_keyframe(image);
	update_map();

//	m_changed = true;
//	if (m_display_delegate) {
//		m_display_delegate->display_with(this);
//	}
}


void Slam::preprocess(Image* image){

	std::cout << "Slam::preprocess" << std::endl;
	
	if (m_key) {
		if (!m_frame) {
			int w = image->width();
			int h = image->height();
			m_frame = new Camera();
			SpaceToolbox::init_intrinsic(m_frame->intrinsic, 45, w, h);
		
			m_frame->points = new CvImage(w, h, Image::Float32, 4);
		}
		image->gray(m_frame->gray);
		m_frame->gray->sobel_x(m_frame->gradient[0]);
		m_frame->gray->sobel_y(m_frame->gradient[1]);

	}
	
}
void Slam::update_pose(){

	std::cout << "Slam::update_pose" << std::endl;

	if (Config::manually_content) { return; }
	if (!m_key || !m_frame) { return; }
	
	//Vec3d delta_t, delta_r;
	double pre_res = 0, res;
	
	while(true) {
	
		prepare_residual();
		res = m_residual->abs_mean();
		if (res - pre_res < 0.0001) { break; }
		m_frame->pos += calc_delta_t();
		MatrixToolbox::update_rotation(m_frame->rotation, calc_delta_r());
		wipe_depth(m_frame->pos);
		//... collect other deltas
		pre_res = res;
	}

}


void Slam::update_keyframe(Image* image){

	std::cout << "Slam::update_keyframe" << std::endl;

	if (m_key == NULL) {
	
		int w = image->width();
		int h = image->height();
		m_key = new Camera();
		SpaceToolbox::init_intrinsic(m_key->intrinsic, 45, w, h);
		
		image->copy_to(m_key->original);
		image->gray(m_key->gray);
		m_key->depth = new CvImage(w, h, Image::Float32);

		m_key->depth->set(0.1);
		m_key->points = new CvImage(w, h, Image::Float32, 4);

		m_keyframes.push_back(m_key);
		m_cameras[m_camera_count] = m_key;
	
		m_camera_count++;
	}
	else {

		Vec2f* pGrad = (Vec2f*)m_gradient->data();
		float* pDg = (float*)m_residual->data();
		float* pDepth = (float*)m_depth->data();
		Vec4f* pPts = (Vec4f*)m_frame->points->data();
		unsigned char* pMask = (unsigned char*)m_mask->data();
		float* p_kd = (float*)m_key->depth->data();
		const Vec3d& t = m_frame->pos;
		int total = m_width * m_height;
		double a[3];
		double a2, temp;
		for (int i = 0; i < total; i++) {
			if (!pMask[i]) { continue; }
			temp = m_frame->intrinsic.f*pDepth[i]/p_kd[i];
			a[0] = pGrad[i][0]*temp;
			a[1] = pGrad[i][1]*temp;
			a[2] = -(a[0]*pPts[i][0]+a[1]*pPts[i][0]);
			a2 = a[0]*t[0]+a[1]*t[1]+a[2]*t[2];
			p_kd[i] += a2*pDg[i] / (100 + a2*a2);
		}
	}
	

}
void Slam::update_map(){

	std::cout << "Slam::update_map" << std::endl;
}

void Slam::prepare_residual() {

	if (!m_key || !m_frame) { return; }
	
	
	int width = m_frame->gray->width();
	int height = m_frame->gray->height();
	int total = width * height;

	const Intrinsic& intri0 = m_key->intrinsic;
	float invf0 = 1 / m_key->intrinsic.f;
	const Intrinsic& intri1 = m_frame->intrinsic;
	
	const Vec3d& t = m_frame->pos;
	const Vec9d& R = m_frame->rotation;
	
	int u, v;
	Vec3f m;
	Vec4f p0, p1;
	//unsigned char* pubyte;
	float* pfloat;
	//short* pShort;

	float* p_dkey = (float*)m_key->depth->data();
	Vec4f* p_pts = (Vec4f*)m_frame->points->data();
	Vec4f* p_key_pts = (Vec4f*)m_key->points->data();
	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_dg = (float*)m_residual->data();
	Vec2f* p_grad = (Vec2f*)m_gradient->data();
	float* p_depth = (float*)m_depth->data();
	//Vec3f* p_iuux = (Vec3f*)m_iuux->data();
	for (int i = 0; i < total; i++) {
	
		u = i % width;
		v = i / width;
		
		p0[0] = (u-intri0.cx)*invf0;
		p0[1] = (v-intri0.cy)*invf0;
		p0[2] = 1;
		p0[3] = p_dkey[i];
		
		p_key_pts[i] = p0;
		
		p1[0] = R[0]*p0[0]+R[1]*p0[1]+R[2]*p0[2]+t[0]*p0[3];
		p1[1] = R[3]*p0[0]+R[4]*p0[1]+R[5]*p0[2]+t[1]*p0[3];
		p1[2] = R[6]*p0[0]+R[7]*p0[1]+R[8]*p0[2]+t[2]*p0[3];
		p1[3] = p0[3];
		p1 /= p1[2];
		
		p_pts[i] = p1;
		
		m[0] = intri1.f * p1[0] + intri1.cx;
		m[1] = intri1.f * p1[1] + intri1.cy;
		
		if (m[0] > 0 && m[0] < width-1 && m[1] > 0 & m[1] < height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = (float*)(m_frame->gray->data()) + v * width + u;
			p_dg[i] = p_gkey[i] - SAMPLE_2D(pfloat[0], pfloat[1], pfloat[width], pfloat[width+1], m[0], m[1]);
			
			pfloat = (float*)(m_frame->gradient[0]->data()) + v * width + u;
			p_grad[i][0] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[width], pfloat[width+1], m[0], m[1]);
			pfloat = (float*)(m_frame->gradient[1]->data()) + v * width + u;
			p_grad[i][1] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[width], pfloat[width+1], m[0], m[1]);

			p_depth[i] = p1[3];
		
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			p_grad[i] = 0;
			p_depth[i] = 0;
			//p_iuux[i] = 0;
		}

	}


}

Vec3d Slam::calc_delta_t() {

	if (!m_key || !m_frame) { return Vec3d(); }

	Vec2f* pGrad = (Vec2f*)m_gradient->data();
	float* pDg = (float*)m_residual->data();
	float* pDepth = (float*)m_depth->data();
	Vec4f* pPts = (Vec4f*)m_frame->points->data();
	int total = m_frame->gray->width() * m_frame->gray->height();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double a[3];
	double w, temp;
	Vec9d A;
	Vec3d B;
	
	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }
		w = 1;//(std::abs(pGx[i])+std::abs(pGy[i]))*std::abs(pDg[i]);
		temp = m_frame->intrinsic.f*pDepth[i];
		a[0] = pGrad[i][0]*temp;
		a[1] = pGrad[i][1]*temp;
		a[2] = -(a[0]*pPts[i][0]+a[1]*pPts[i][0]);
		
		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*pDg[i];
		B[1] += w*a[1]*pDg[i];
		B[2] += w*a[2]*pDg[i];
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];
	
	//A /= total;
	//B /= total;

	A[0] += 1;
	A[4] += 1;
	A[8] += 1;
	
	Vec9d invA = MatrixToolbox::inv_matrix_3x3(A);
	return Vec3d(
		invA[0]*B[0]+invA[1]*B[1]+invA[2]*B[2],
		invA[3]*B[0]+invA[4]*B[1]+invA[5]*B[2],
		invA[6]*B[0]+invA[7]*B[1]+invA[8]*B[2]
	);
	

}

Vec3d Slam::calc_delta_r() {


	if (!m_key || !m_frame) { return Vec3d(); }

	Vec2f* pGrad = (Vec2f*)m_gradient->data();
	float* pDg = (float*)m_residual->data();
	float* pDepth = (float*)m_depth->data();
	Vec4f* pPts = (Vec4f*)m_frame->points->data();
	Vec4f* pPts0 = (Vec4f*)m_key->points->data();

	int total = m_frame->gray->width() * m_frame->gray->height();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double a[3];
	double w, temp;
	Vec3d iuux;
	Vec9d A;
	Vec3d B;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }
		w = 1;//(std::abs(pGx[i])+std::abs(pGy[i]))*std::abs(pDg[i]);
		
		temp = m_frame->intrinsic.f*pDepth[i];
		iuux[0] = temp*pGrad[i][0];
		iuux[1] = temp*pGrad[i][1];
		iuux[2] = -(iuux[0]*pPts[i][0]+iuux[1]*pPts[i][1]);

		
		a[0] = pPts[i][1]*iuux[2]-pPts[i][2]*iuux[1];
		a[1] = pPts[i][2]*iuux[0]-pPts[i][0]*iuux[2];
		a[2] = pPts[i][0]*iuux[1]-pPts[i][1]*iuux[0];
		
		
		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*pDg[i]*pDepth[i];
		B[1] += w*a[1]*pDg[i]*pDepth[i];
		B[2] += w*a[2]*pDg[i]*pDepth[i];
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];
	
	//A /= total;
	//B /= total;

	A[0] += 1;
	A[4] += 1;
	A[8] += 1;
	
	Vec9d invA = MatrixToolbox::inv_matrix_3x3(A);
	return Vec3d(
		invA[0]*B[0]+invA[1]*B[1]+invA[2]*B[2],
		invA[3]*B[0]+invA[4]*B[1]+invA[5]*B[2],
		invA[6]*B[0]+invA[7]*B[1]+invA[8]*B[2]
	);
	

}

void Slam::wipe_depth(const Vec3d& t) {
	
	int total = m_residual->width() * m_residual->height();
	Vec2f* pGrad = (Vec2f*)m_gradient->data();
	float* pd = (float*)m_key->depth->data();
	float* pdg = (float*)m_residual->data();
	unsigned char* pm = (unsigned char*)m_mask->data();
	float* pDepth = (float*)m_depth->data();
	Vec4f* pPts = (Vec4f*)m_frame->points->data();
	double lamda1 = 1, lamda2 = 1;
	double a, w, temp;
	Vec3d iuux;
	for (int i = 0; i < total; i++) {
	
		if (!pm[i]) { continue; }
		w = 1;
		temp = m_frame->intrinsic.f*pDepth[i];
		iuux[0] = temp*pGrad[i][0]*w;
		iuux[1] = temp*pGrad[i][1]*w;
		iuux[2] = -(iuux[0]*pPts[i][0]+iuux[1]*pPts[i][1]);
		a = iuux[0]*t[0]+iuux[1]*t[1]+iuux[2]*t[2];
		pd[i] = lamda1*a*pdg[i] / (a * a + lamda2);
	
	}
}



} // namespace




/***************************





	cv::minMaxIdx(image, &min, &max);
	scale = 1./(max-min);
	_image.convertTo(image, 
		CV_32FC(image.channels()), scale, -min/(max-min));		

	
	//MatrixToolbox::rectify_rotation(m_frame->rotation);

		
		
		//m_frame->gradient[0]->save("aaa.jpg");
		//m_frame->gradient[1]->save("bbb.jpg");
		//m_frame->gray->subtract(m_key->gray, m_frame->residual);
		
		
	
		// unprojects each pixel for calc delta
		int width = m_key->gray->width();
		int height = m_key->gray->height();
		
		
		int total = width * height;
		const Intrinsic& intri = m_key->intrinsic;
		float invf0 = 1 / intri.f;
		int u, v;
		Vec4f p;
		float* p_depth = (float*)m_key->depth->data();
		Vec4f* p_pts = (Vec4f*)m_key->points->data();

		for (int i = 0; i < total; i++) {
	
			u = i % width;
			v = i / width;
		
			p[0] = (u-intri.cx)*invf0;
			p[1] = (v-intri.cy)*invf0;
			p[2] = 1;
			p[3] = p_depth[i];
		
			p_pts[i] = p;
		}


		
	
	if (m_source->read(image)) {
		image->resize(resized);
		initialize(resized);
	}


		//if (delta_t.length2() < 0.0001) { break; }
		break;

		//image->convert_to(m_key->depth, Image::Float32);
				
		//memset(m_key->pos, 0, sizeof(m_key->pos));
		//m_key->image = image;
		//m_key->depth = default_depth; 

	short* pGx = (short*)m_frame->gradient[0]->data();
	short* pGy = (short*)m_frame->gradient[1]->data();
	short* pDg = (short*)m_frame->residual->data();
	float* pIz = (float*)m_key->depth->data();
	

	if (m_camera_count == 1) {
	
		m_cameras[m_camera_count] = m_frame;
		m_camera_count++;
	}
		
			p_grad[i][0] = m_frame->gradient[0]->sample(m[0], m[1]);
			p_grad[i][1] = m_frame->gradient[1]->sample(m[0], m[1]);
			
			p_dg[i] = p_gkey[i] - m_frame->gray->sample(m[0], m[1]);

m_frame->gray->sample(m[0], m[1]);

	m_mask->set(255);
	//m_mask->reset(width, height, Image::UnsignedChar);

	unsigned char* pGray = (unsigned char*)m_frame->gray->data();


	return Vec3d();
	
	//int width = image->width;
	//int height = image->height;
	
	//u = i % width;
	//v = i / width;
	
	
	
	
	//foreach(pixel) {
	
		// calc a;
		// calc b;
		// calc w;
		// A += w * a * a';
		// B += w * b;
	//}
	//return inv(A) * B;
	
	return Vec3d();


#include <unistd.h>
		//sleep(10);

void Slam::tick() {

	std::cout << "Slam::tick" << std::endl;


}


	Camera* camera1 = new Camera();
	camera1->pos[0] = 0;
	camera1->pos[1] = 0;
	camera1->pos[2] = 0;
	
	Camera* camera2 = new Camera();
	camera2->pos[0] = 100;
	camera2->pos[1] = 100;
	camera2->pos[2] = 100;
	
	m_keyframes.push_back(camera1);
	m_keyframes.push_back(camera2);
	
	m_cameras[0] = camera1;
	m_cameras[1] = camera2;
	
	m_camera_count = 2;



#include <stdio.h>
#include <stdio.h>

*********************************/

