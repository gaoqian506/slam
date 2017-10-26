

#include "Slam.h"
#include "MatrixToolbox.h"
#include "CvImage.h"
#include <iostream>
#include <memory.h>
#include <cmath>


#define SAMPLE_2D( v0, v1, v2, v3, a, b) \
	( v0*(1-a)*(1-b)+v1*(1-a)*b+v2*a*(1-b)+v3*a*b)


namespace ww {


Slam::Slam(VideoSource* vs) : m_working(false), m_camera_count(0), m_key(NULL), m_frame(NULL), m_changed(false) {

	std::cout << "Slam::Slam" << std::endl;
	
	m_source = vs;
	memset(m_cameras, 0, sizeof(Camera*) * MAX_STATIC_CAMERA_COUNT);
	

	
}

void Slam::start() {


	std::cout << "Slam::start" << std::endl;
	m_working = true;

	//for(int i = 0; i < 10; i++) {
	//	push();
	//}
	
	Image* image = NULL;
	
	if (m_source->read(image)) {
	
		initialize(image);
	}
	
	while(m_source->read(image) && m_working) {
		push(image);
	}
	
	if (image) { delete image; }
	
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

void Slam::initialize(Image* image) {

	int w = image->width();
	int h = image->height();
	
	m_mask = new CvImage(w, h, Image::UByte);
	m_points = new CvImage(w, h, Image::Float32, 4);
	m_residual = new CvImage(w, h, Image::Float32);
	m_gradient = new CvImage(w, h, Image::Float32, 2);
	m_depth = new CvImage(w, h, Image::Float32);

}

void Slam::push(Image* image) {

	std::cout << "Slam::push" << std::endl;
	preprocess(image);
	update_pose();
	update_keyframe(image);
	update_map();

	m_changed = true;
//	if (m_display_delegate) {
//		m_display_delegate->display_with(this);
//	}
}


void Slam::preprocess(Image* image){

	std::cout << "Slam::preprocess" << std::endl;
	
	if (m_key) {
		if (!m_frame) { m_frame = new Camera(); }
		image->gray(m_frame->gray);
		m_frame->gray->sobel_x(m_frame->gradient[0]);
		m_frame->gray->sobel_y(m_frame->gradient[1]);
		
		//m_frame->gradient[0]->save("aaa.jpg");
		//m_frame->gradient[1]->save("bbb.jpg");
		//m_frame->gray->subtract(m_key->gray, m_frame->residual);
	}
	
}
void Slam::update_pose(){

	std::cout << "Slam::update_pose" << std::endl;
	
	if (!m_key || !m_frame) { return; }
	
	Vec3d delta_t;
	
	while(true) {
	
		prepare_residual();
		delta_t = calc_delta_t();
		//... collect other deltas
		m_frame->pos += delta_t;
		//current_camera->pose += delta_t;
		
		if (delta_t.length2() < 0.0001) { break; }
	}
}


void Slam::update_keyframe(Image* image){

	std::cout << "Slam::update_keyframe" << std::endl;

	if (m_key == NULL) {
	
		m_key = new Camera();
		
		image->copy_to(m_key->original);
		image->gray(m_key->gray);
		m_key->depth = new CvImage(image->width(), image->height(), Image::Float32);

		m_key->depth->set(0.1);
		m_key->points = new CvImage(image->width(), image->height(), Image::Float32, 4);

		m_keyframes.push_back(m_key);
		m_cameras[m_camera_count] = m_key;
	
		m_camera_count++;
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
	
	Vec3d& trans = m_frame->pos;
	
	int u, v;
	Vec3f m;
	Vec4f p;
	//unsigned char* pubyte;
	float* pfloat;
	//short* pShort;

	float* p_dkey = (float*)m_key->depth->data();
	Vec4f* p_pts = (Vec4f*)m_points->data();
	Vec4f* p_key_pts = (Vec4f*)m_key->points->data();
	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_dg = (float*)m_residual->data();
	Vec2f* p_grad = (Vec2f*)m_gradient->data();
	float* p_depth = (float*)m_depth->data();
	for (int i = 0; i < total; i++) {
	
		u = i % width;
		v = i / width;
		
		p[0] = (u-intri0.cx)*invf0;
		p[1] = (v-intri0.cy)*invf0;
		p[2] = 1;
		p[3] = p_dkey[i];
		
		p_key_pts[i] = p;
		
		p[0] = p[0] + p[3] * trans[0];
		p[1] = p[1] + p[3] * trans[1];
		p[2] = p[2] + p[3] * trans[2];
		
		p /= p[2];
		
		p_pts[i] = p;
		
		m[0] = intri1.f * p[0] + intri1.cx;
		m[1] = intri1.f * p[1] + intri1.cy;
		
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

			p_depth[i] = p[3];
		
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			p_grad[i] = 0;
			p_depth[i] = 0;
		}

	}


}

Vec3d Slam::calc_delta_t() {

	if (!m_key || !m_frame) { return Vec3d(); }

	Vec2f* pGrad = (Vec2f*)m_gradient->data();
	float* pDg = (float*)m_residual->data();
	float* pDepth = (float*)m_depth->data();

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
		a[0] = w*pGrad[i][0]*temp;
		a[1] = w*pGrad[i][1]*temp;
		a[2] = -(a[0]+a[1])*pDepth[i];
		
		A[0] += a[0]*a[0];
		A[1] += a[0]*a[1];
		A[2] += a[0]*a[2];
		A[4] += a[1]*a[1];
		A[5] += a[1]*a[2];
		A[8] += a[2]*a[2];
		
		B[0] += a[0]*pDg[i];
		B[1] += a[1]*pDg[i];
		B[2] += a[2]*pDg[i];
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];
	
	A /= total;
	B /= total;
	
	Vec9d invA = MatrixToolbox::inv_matrix_3x3(A);
	return Vec3d(
		invA[0]*B[0]+invA[1]*B[1]+invA[2]*B[2],
		invA[3]*B[0]+invA[4]*B[1]+invA[5]*B[2],
		invA[6]*B[0]+invA[7]*B[1]+invA[8]*B[2]
	);
	

}



} // namespace


/***************************



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

