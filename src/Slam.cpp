

#include "Slam.h"
#include "Image.h"
#include "MatrixToolbox.h"
#include <iostream>
#include <memory.h>
#include <cmath>


namespace ww {


Slam::Slam(VideoSource* vs) : m_working(false), m_camera_count(0), m_key(NULL), m_frame(NULL) {

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

void Slam::push(Image* image) {

	std::cout << "Slam::push" << std::endl;
	preprocess(image);
	update_pose();
	update_keyframe(image);
	update_map();

	if (m_display_delegate) {
		m_display_delegate->display_with(this);
	}
}


void Slam::preprocess(Image* image){

	std::cout << "Slam::preprocess" << std::endl;
	
	if (m_key) {
		if (!m_frame) { m_frame = new Camera(); }
		image->gray(m_frame->gray);
		image->sobel_x(m_frame->gradient[0]);
		image->sobel_y(m_frame->gradient[1]);
		m_frame->gray->subtract(m_key->gray, m_frame->residual);
	}
	
}
void Slam::update_pose(){

	std::cout << "Slam::update_pose" << std::endl;
	
	Vec3d delta_t;
	
	while(true) {
	
		delta_t = calc_delta_t();
		//... collect other deltas
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
		image->convert_to(m_key->depth, Image::Float32);
		m_key->depth->set(0.1);
		
		//memset(m_key->pos, 0, sizeof(m_key->pos));
		//m_key->image = image;
		//m_key->depth = default_depth; 
		
		m_keyframes.push_back(m_key);
		m_cameras[m_camera_count] = m_key;
	
	m_camera_count++;
	}
}
void Slam::update_map(){

	std::cout << "Slam::update_map" << std::endl;
}

Vec3d Slam::calc_delta_t() {

	if (!m_key || !m_frame) { return Vec3d(); }

	short* pGx = (short*)m_frame->gradient[0]->data();
	short* pGy = (short*)m_frame->gradient[1]->data();
	short* pDg = (short*)m_frame->residual->data();
	float* pIz = (float*)m_key->depth->data();

	int total = m_frame->gray->width() * m_frame->gray->height();
	
	double a[3];
	double w, temp;
	Vec9d A;
	Vec3d B;
	
	for (int i = 0; i < total; i++) {

		w = (std::abs(pGx[i])+std::abs(pGy[i]))*std::abs(pDg[i]);
		temp = m_frame->intrinsic.f*pIz[i];
		a[0] = w*pGx[i]*temp;
		a[1] = w*pGy[i]*temp;
		a[2] = -w*(a[0]+a[1])*pIz[i]*temp;
		
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
	
	Vec9d invA = MatrixToolbox::inv_matrix_3x3(A);
	return Vec3d(
		invA[0]*B[0]+invA[1]*B[1]+invA[2]*B[2],
		invA[3]*B[0]+invA[4]*B[1]+invA[5]*B[2],
		invA[6]*B[0]+invA[7]*B[1]+invA[8]*B[2]
	);
	

}



} // namespace


/***************************




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

