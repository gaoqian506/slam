

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
	m_debug_weight = NULL;
	m_epi_line = NULL;
	m_warp = NULL;
	m_weight = NULL;
	m_of = NULL;
	memset(m_grad_grad, 0, sizeof(m_grad_grad));
	m_grad_residual = NULL;
	m_dut = NULL;
	//m_depth_weight = NULL;

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

Image* Slam::get_debug_image(int iid, int kid, Image** weight/* = 0*/) {

	Image *which1 = NULL, *which2 = NULL;

	Camera* key = NULL;
	if (kid < m_keyframes.size()) {
		key = m_cameras[kid];
	}

	if (iid == 0 && m_frame) {
		which1 = m_frame->gray;
		m_image_name = "current gray(1)";
	}
	else if (iid == 1) {
		which1 = key ? key->warp : m_warp;
		m_image_name = "warp(2)";
	}
	else if (iid == 2 && key) {
		which1 = key->gray;
		m_image_name = "key gray(3)";
	}
	else if (iid == 3) {
		which1 = key ? key->residual : m_residual;
		m_image_name = "residual(4)";
	}
	else if (iid == 4 && key) {
		which1 = key->gradient[0];
		which2 = key->gradient[1];
		if (weight) { *weight = key ? key->of_weight : NULL; }
		m_image_name = "key grad(5)";
	}
	else if (iid == 5 && m_frame) {
		which1 = m_frame->gradient[0];
		which2 = m_frame->gradient[1];
		if (weight) { *weight = key ? key->of_weight : NULL; }
		m_image_name = "frame grad(6)";
	}
	else if (iid == 6) {
		which1 = key ? key->warp_gradient : m_gradient;
		if (weight) { *weight = key ? key->of_weight : NULL; }
		m_image_name = "warp gradient(7)";
	}		
	else if (iid == 7) {
		which1 = key ? key->optical_flow : m_of;
		if (weight) { *weight = key ? key->of_weight : NULL; }
		m_image_name = "optical flow(8)";
	}
	else if (iid == 8) {
		which1 = key ? key->dut : m_dut;
		if (weight) { *weight = key ? key->epi_weight : NULL; }
		m_image_name = "dut(9)";
	}
	else if (iid == 9) {
		which1 = key ? key->of_weight : m_weight;
		m_image_name = "of weight(10)";
	}	

	else if (iid == 10) {
		which1 = key ? key->mask : m_mask;
		m_image_name = "mask(11)";
	}
	else {
		which1 = NULL;
	}


	double min, max, scale;

	if (weight && *weight) {
		(*weight)->min_max(&min, &max);
		scale = 1./(max-min);
		(*weight)->convert_to(m_debug_weight, Image::Float32, scale, -min/(max-min));
		*weight = m_debug_weight;
	}

	if (which1 && !which2 && which1->channels() == 1) {
		double min, max, scale;
		which1->min_max(&min, &max);
		scale = 1./(max-min);
		which1->convert_to(m_debug_image, Image::Float32, scale, -min/(max-min));
		return m_debug_image;
	}
	else if (which1 && which2) {
		which1->merge(which2, m_debug_image);
		return m_debug_image;
	}
	else if (which1) {
		which1->copy_to(m_debug_image);
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
		if (Config::method == Config::Epipolar) {
			push_epi(resized);
		}
		else { push(resized); }
		//preprocess(image);
		//update_keyframe(image);
	}
	//m_changed = false;
	if (image) { delete image; }
	if (resized) { delete resized; }
	
	//std::cout << "leave Slam::start" << std::endl;

}

void Slam::func_manualy(int idx) {

	switch (idx) {

	case 1:
		if (m_frame && Config::method == Config::Lsd) {
			prepare_residual();
			m_frame->pos += calc_delta_t();
		}
		if (m_frame && Config::method == Config::Epipolar) {
			prepare_residual_epi();
			m_frame->epi_point = calc_epi_point();
		}
		if (m_frame && Config::method == Config::Entropy) {
			prepare_residual_epi();
			m_frame->epi_point = calc_t_entropy();
		}
		if (m_frame && Config::method == Config::Entropy2 || m_frame && Config::method == Config::Entropy3 || m_frame && Config::method == Config::Entropy4) {
			prepare_residual_entropy2();
			m_frame->epi_point = calc_epi_point_entropy2();
		}
		if (m_frame && Config::method == Config::Entropy5) {
			prepare_residual_entropy2(true);
			m_frame->pos += calc_dt_entropy5();
		}
		if (m_frame && Config::method == Config::Lsd2) {
			prepare_residual_lsd2();
			m_frame->pos += calc_dt_lsd2();
		}
		if (m_frame && Config::method == Config::Lsd3) {
			prepare_residual_lsd3();
			m_frame->pos += calc_dt_lsd3();
		}
		if (m_frame && Config::method == Config::Epi2) {
			prepare_residual_epi2();
			m_frame->epi_point = calc_e_epi2();
		}
		if (m_frame && Config::method == Config::Lsd4) {
			prepare_residual_lsd4();
			m_frame->pos += calc_dt_lsd4();
		}
		if (m_frame && Config::method == Config::Of1) {
			prepare_residual_of1();
			calc_du_of1();
		}
		if (m_frame && Config::method == Config::Gof1) {
			prepare_residual_gof1();
			calc_du_gof1();
		}
		if (m_frame && Config::method == Config::Of2) {
			prepare_residual_of2();
			calc_du_of2();
		}		
		if (m_frame && Config::method == Config::Of3) {
			prepare_residual_of2();
			calc_du_of3();
			m_of->add(m_dut);
		}		
		break;
	case 2:
		if (m_frame && Config::method == Config::Lsd) {
			prepare_residual();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_delta_r());
		}
		if (m_frame && Config::method == Config::Entropy) {
			prepare_residual_epi();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy());
		}
		if (m_frame && Config::method == Config::Entropy2) {
			prepare_residual_entropy2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy2());
		}
		if (m_frame && Config::method == Config::Entropy3) {
			prepare_residual_entropy2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy3());
		}
		if (m_frame && Config::method == Config::Entropy4) {
			prepare_residual_entropy2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy4());
		}
		if (m_frame && Config::method == Config::Entropy5) {
			prepare_residual_entropy2(true);
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy4());
		}
		if (m_frame && Config::method == Config::Lsd2) {
			prepare_residual_lsd2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd2());
		}
		if (m_frame && Config::method == Config::Lsd3) {
			prepare_residual_lsd3();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd3());
		}
		if (m_frame && Config::method == Config::Epi2) {
			prepare_residual_epi2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_epi2());
		}
		if (m_frame && Config::method == Config::Lsd4) {
			prepare_residual_lsd4();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd4());
		}
		if (m_frame && Config::method == Config::Of1) {
			prepare_residual_of1();
			smooth_of_of1();
		}
		if (m_frame && Config::method == Config::Of2) {
			prepare_residual_of2();
			smooth_of_of2();
		}
		if (m_frame && Config::method == Config::Of3) {
			calc_e_dr_of3();
		}		
		break;
	case 3:
		if (m_frame && Config::method == Config::Lsd) {
			prepare_residual();
			m_frame->pos += calc_delta_t();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_delta_r());
		}
		if (m_frame && Config::method == Config::Entropy) {
			prepare_residual_epi();
			m_frame->epi_point = calc_t_entropy();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy());
		}
		if (m_frame && Config::method == Config::Entropy2) {
			prepare_residual_entropy2();
			m_frame->epi_point = calc_epi_point_entropy2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy2());
		}
		if (m_frame && Config::method == Config::Entropy3) {
			prepare_residual_entropy2();
			m_frame->epi_point = calc_epi_point_entropy2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy3());
		}
		if (m_frame && Config::method == Config::Entropy5) {
			prepare_residual_entropy2(true);
			m_frame->pos += calc_dt_entropy5();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_entropy4());
		}
		if (m_frame && Config::method == Config::Lsd2) {
			prepare_residual_lsd2();
			m_frame->pos += calc_dt_lsd2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd2());
		}
		if (m_frame && Config::method == Config::Lsd3) {
			prepare_residual_lsd3();
			m_frame->pos += calc_dt_lsd3();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd3());
		}
		if (m_frame && Config::method == Config::Lsd4) {
			prepare_residual_lsd4();
			m_frame->pos += calc_dt_lsd4();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd4());
		}
		if (m_frame && Config::method == Config::Of1) {
			prepare_residual_of1();
			calc_du_of1();
			prepare_residual_of1();
			smooth_of_of1();
		}
		if (m_frame && Config::method == Config::Of2) {
			prepare_residual_of2();
			calc_du_of2();
			prepare_residual_of2();
			smooth_of_of2();
		}	
		if (m_frame && Config::method == Config::Of3) {
			calc_t_of3();
		}			
		break;
	case 4:
		if (m_frame && Config::method == Config::Lsd) {
			prepare_residual();
			update_depth();
		}
		if (m_frame && Config::method == Config::Entropy2) {
			m_frame->pos = m_frame->epi_point * m_frame->movement;
			prepare_residual_entropy2(true);
			m_frame->movement += calc_dl_entropy2();
			m_frame->pos = m_frame->epi_point * m_frame->movement;
		}
		if (m_frame && Config::method == Config::Lsd2) {
			prepare_residual_lsd2();
			update_depth_lsd2();
			smooth_depth_lsd2();
		}
		if (m_frame && Config::method == Config::Lsd3) {
			prepare_residual_lsd3();
			update_depth_lsd2();
			smooth_depth_lsd2();
		}
		if (m_frame && Config::method == Config::Of2) {
			prepare_residual_of2();
			calc_du_of2();
			prepare_residual_of2();
			smooth_of_of2();
		}	
		if (m_frame && Config::method == Config::Of3) {
			update_depth_of3();
			unproject_points_of3();
		}		
		break;
	case 5:
		if (m_frame && Config::method == Config::Lsd2) {
			prepare_residual_lsd2();
			m_frame->pos += calc_dt_lsd2();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd2());
			update_depth_lsd2();
			smooth_depth_lsd2();
		}
		if (m_frame && Config::method == Config::Lsd3) {
			prepare_residual_lsd3();
			m_frame->pos += calc_dt_lsd3();
			MatrixToolbox::update_rotation(m_frame->rotation, calc_dr_lsd3());
			update_depth_lsd2();
			smooth_depth_lsd2();
		}
		break;

	case 9:
		create_keyframe(NULL);
		break;
	}
}


char* Slam::pixel_info(const Vec2d& u, int kid) {
	
	int idx = 0;
	if (u[0] >= 0 && u[0] < m_width && u[1] >= 0 && u[1] < m_height) {
		idx = ((int)u[1]) * m_width + ((int)u[0]);
	}

	Camera* key = NULL;
	if (kid < m_keyframes.size()) {
		key = m_cameras[kid];
	}

	Vec3f* pl = m_epi_line ? (Vec3f*)m_epi_line->data() : NULL;
	sprintf(m_pixel_info, 
		"image size: %d, %d\n"
		"active: %s\n"
		"pos:%f, %f\n"
		"key:%f cur:%f\n"
		"res:%f\n"
		"of weight:%f epi_weight:%f mask:%d\n"
		"grad res: %f, %f\n"
		"grad: %f, %f\n"
		"of: %f, %f\n"
		"dof: %f, %f\n"
		"depth: %f, %f\n"
		"e: %f, %f, %f\n"
		"t: %f, %f, %f\n"
		"R: %f, %f, %f\n"
		"   %f, %f, %f\n"
		"   %f, %f, %f\n",
		m_width, m_height,
		m_image_name.c_str(),
		u[0], u[1],
		m_key ? *((float*)m_key->gray->at(idx)) : 0.0,
		m_frame ? *((float*)m_frame->gray->at(idx)) : 0.0,
		key ? *((float*)key->residual->at(idx)) : 0,
		key ? *((float*)key->of_weight->at(idx)) : 0,
		key ? *((float*)key->epi_weight->at(idx)) : 0,
		key ? *((unsigned char*)key->mask->at(idx)) : 0,
		m_grad_residual ? ((Vec2f*)m_grad_residual->data())[idx][0] : 0,
		m_grad_residual ? ((Vec2f*)m_grad_residual->data())[idx][1] : 0,
		m_key ? ((float*)m_key->gradient[0]->at(idx))[0] : 0,
		m_key ? ((float*)m_key->gradient[1]->at(idx))[0] : 0,
		key ? ((Vec2f*)key->optical_flow->data())[idx][0] : 0,
		key ? ((Vec2f*)key->optical_flow->data())[idx][1] : 0,
		key ? ((Vec2f*)key->dut->data())[idx][0] : 0,
		key ? ((Vec2f*)key->dut->data())[idx][1] : 0,		
		//pl ? pl[idx][0] : 0,
		//pl ? pl[idx][1] : 0,
		//pl ? pl[idx][2] : 0,
		//((Vec2f*)m_gradient->at(idx))[0][0],
		//((Vec2f*)m_gradient->at(idx))[0][1],
		m_key ? *((float*)m_key->depth->at(idx)) : 0.0,
		m_key ? *((float*)m_key->depth_weight->at(idx)) : 0.0,
		m_frame ? m_frame->epi_point[0] : 0,
		m_frame ? m_frame->epi_point[1] : 0,
		m_frame ? m_frame->epi_point[2] : 0,
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

Image* Slam::get_optical_flow() {

	return m_of;

}

void Slam::build(BuildFlag flag/* = BuildAll*/) {

	switch(Config::method) {

	case Config::Of3:
		build_of3(flag);
		break;
	case Config::Of4:
		build_of4(flag);
		break;
	}

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
	m_epi_line = new CvImage(w, h, Image::Float32, 3);
	m_warp = new CvImage(w, h, Image::Float32);
	m_weight = new CvImage(w, h, Image::Float32);
	m_of = new CvImage(w, h, Image::Float32, 2);

	for (int i = 0; i < 4; i++) {
		m_grad_grad[i] = new CvImage(w, h, Image::Float32);
	}
	m_grad_residual = new CvImage(w, h, Image::Float32, 2);
	m_dut = new CvImage(w, h, Image::Float32, 2);
	//m_depth_weight = new CvImage(w, h, Image::Float32);
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

void Slam::push_epi(Image* image) {

	preprocess(image);
	calc_epipolar();
	calc_epi_trans();
	update_keyframe(image);
	// refine_map();
}


void Slam::preprocess(Image* image){

	std::cout << "Slam::preprocess" << std::endl;
	
	if (!m_frame) {
		m_frame = new Camera();
		SpaceToolbox::init_intrinsic(m_frame->intrinsic, 45, m_width, m_height);
		m_frame->points = new CvImage(m_width, m_height, Image::Float32, 4);

	}
	image->copy_to(m_frame->original);
	image->gray(m_frame->gray);
	m_frame->gray->sobel_x(m_frame->gradient[0]);
	m_frame->gray->sobel_y(m_frame->gradient[1]);
	if (Config::method == Config::Epipolar) {
		make_epi_line(m_frame);
	}
}

void Slam::make_epi_line(Camera* camera) {

	int total = m_width * m_height;
	int u, v;
	double f = camera->intrinsic.f;
	double f1 = 1.0 / f;
	double f2 = f1 * f1;
	Vec3f* pl = (Vec3f*)m_epi_line->data();
	float* piu = (float*)camera->gradient[0]->data();
	float* piv = (float*)camera->gradient[1]->data();
	for (int i = 0; i < total; i++) {

		u = i % m_width-camera->intrinsic.cx;
		v = i / m_width-camera->intrinsic.cy;

		pl[i][0] = -piv[i];
		pl[i][1] = +piu[i];
		pl[i][2] = (piv[i]*u-piu[i]*v)*f1;
		
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
		//wipe_depth(m_frame->pos);
		//... collect other deltas
		pre_res = res;
	}

}

void Slam::calc_epipolar() {

	if (Config::manually_content) { return; }
	if (!m_key || !m_frame) { return; }
	while(true) {
		prepare_residual_epi();
		m_frame->epi_point = calc_epi_point();
		MatrixToolbox::update_rotation(m_frame->rotation, calc_epi_dr());
		break;
	}


	

}
void Slam::calc_epi_trans() {

	if (Config::manually_content) { return; }
	if (!m_key || !m_frame) { return; }
	while(true) {
		//prepare_residual_epi();
		//m_frame->pos = calc_epi_pos();
		break;
	}


}


void Slam::update_keyframe(Image* image){

	std::cout << "Slam::update_keyframe" << std::endl;

	if (m_key == NULL) { //or need add a new keyframe
		create_keyframe(image);
	}
	else {
		// while(need upadte depth) {
			// prepare residual
			// update_depth
		//}
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
	float* p_warp = (float*)m_warp->data();
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

			p_warp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[width], pfloat[width+1], m[0], m[1]);
			p_dg[i] = p_gkey[i] - p_warp[i];
			
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
			p_warp[i] = 0;
			//p_iuux[i] = 0;
		}

	}


}

void Slam::prepare_residual_epi() {

	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;

	double KRKi[9];

	SpaceToolbox::make_KRKi(
		m_key->intrinsic,
		m_frame->rotation,
		m_frame->intrinsic,
		KRKi,
		true
	);
	
	int u, v;
	Vec3d m;
	float* pfloat;

	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_gframe = (float*)m_frame->gray->data();
	float* p_dg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();


	for (int i = 0; i < total; i++) {
	
		u = i % m_width;
		v = i / m_width;

		m[0] = KRKi[0]*u+KRKi[1]*v+KRKi[2];
		m[1] = KRKi[3]*u+KRKi[4]*v+KRKi[5];
		m[2] = KRKi[6]*u+KRKi[7]*v+KRKi[8];
		m /= m[2];
		
		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = p_gkey + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			p_dg[i] = pwarp[i] - p_gframe[i];
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			pwarp[i] = 0;
		}

	}

}


void Slam::prepare_residual_entropy2(bool use_trans/* = false*/) {

	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;

	double KRKi[9];

	SpaceToolbox::make_KRKi(
		m_key->intrinsic,
		m_frame->rotation,
		m_frame->intrinsic,
		KRKi
	);
	
	int u, v;
	Vec3d m;
	float* pfloat;

	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_gframe = (float*)m_frame->gray->data();
	float* p_dg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	float* pdepth = (float*)m_key->depth->data();
	Intrinsic in = m_key->intrinsic;
	Vec3d t = m_frame->pos;

	for (int i = 0; i < total; i++) {
	
		u = i % m_width;
		v = i / m_width;

		m[0] = KRKi[0]*u+KRKi[1]*v+KRKi[2];
		m[1] = KRKi[3]*u+KRKi[4]*v+KRKi[5];
		m[2] = KRKi[6]*u+KRKi[7]*v+KRKi[8];

		if (use_trans) {
			m[0] += pdepth[i]*(in.f*t[0]+in.cx*t[2]);
			m[1] += pdepth[i]*(in.f*t[1]+in.cy*t[2]);
			m[2] += pdepth[i]*(t[2]);
		}
		m /= m[2];
		
		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = p_gframe + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			p_dg[i] = p_gkey[i]-pwarp[i];
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			pwarp[i] = 0;
		}

	}

}


void Slam::prepare_residual_lsd2() {

	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	Vec3d m;
	Vec4f x0, x1;
	float* pfloat;

	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_gframe = (float*)m_frame->gray->data();
	float* p_dg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	float* pd = (float*)m_key->depth->data();
	Vec4f* px = (Vec4f*)m_key->points->data();

	Intrinsic in0 = m_key->intrinsic;
	Intrinsic in1 = m_frame->intrinsic;
	double f1 = 1.0/in0.f;
	Vec9d R = m_frame->rotation;
	Vec3d t = m_frame->pos;

	for (int i = 0; i < total; i++) {
	
		u = i % m_width - in0.cx;
		v = i / m_width - in0.cy;

		x0[0] = u*f1;
		x0[1] = v*f1;
		x0[2] = 1;
		x0[3] = pd[i];
		px[i] = x0;

		x1[0] = R[0]*x0[0]+R[1]*x0[1]+R[2]*x0[2]+t[0]*x0[3];
		x1[1] = R[3]*x0[0]+R[4]*x0[1]+R[5]*x0[2]+t[1]*x0[3];
		x1[2] = R[6]*x0[0]+R[7]*x0[1]+R[8]*x0[2]+t[2]*x0[3];
		x1[3] = x0[3];
		x1 /= x1[2];

		m[0] = in1.f*x1[0]+in1.cx;
		m[1] = in1.f*x1[1]+in1.cy;
		
		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = p_gframe + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			p_dg[i] = p_gkey[i]-pwarp[i];
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			pwarp[i] = 0;
		}
	}
}


void Slam::prepare_residual_lsd3() {

	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	Vec3d m;
	Vec4f x0, x1;
	float* pfloat;

	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_gframe = (float*)m_frame->gray->data();
	float* p_dg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	float* pw = (float*)m_weight->data();
	float* pd = (float*)m_key->depth->data();
	Vec4f* px = (Vec4f*)m_key->points->data();

	Intrinsic in0 = m_key->intrinsic;
	Intrinsic in1 = m_frame->intrinsic;
	double f1 = 1.0/in0.f;
	Vec9d R = m_frame->rotation;
	Vec3d t = m_frame->pos;

	Vec9d iR = MatrixToolbox::transpose_3x3(R);
	double it[3] = {
		-(iR[0]*t[0]+iR[1]*t[1]+iR[2]*t[2]),
		-(iR[3]*t[0]+iR[4]*t[1]+iR[5]*t[2]),
		-(iR[6]*t[0]+iR[7]*t[1]+iR[8]*t[2]),
	};
	R = iR;
	t = it;

	for (int i = 0; i < total; i++) {
	
		u = i % m_width - in0.cx;
		v = i / m_width - in0.cy;

		x0[0] = u*f1;
		x0[1] = v*f1;
		x0[2] = 1;
		x0[3] = pd[i];
		px[i] = x0;

		x1[0] = R[0]*x0[0]+R[1]*x0[1]+R[2]*x0[2]+t[0]*x0[3];
		x1[1] = R[3]*x0[0]+R[4]*x0[1]+R[5]*x0[2]+t[1]*x0[3];
		x1[2] = R[6]*x0[0]+R[7]*x0[1]+R[8]*x0[2]+t[2]*x0[3];
		x1[3] = x0[3];
		x1 /= x1[2];

		m[0] = in1.f*x1[0]+in1.cx;
		m[1] = in1.f*x1[1]+in1.cy;
		
		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = p_gframe + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			p_dg[i] = pwarp[i] - p_gkey[i];
			pw[i] = exp(-p_dg[i]*p_dg[i]/0.25);
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			pwarp[i] = 0;
			pw[i] = 0;
		}
	}
}


void Slam::prepare_residual_lsd4() {

	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	Vec3d m;
	Vec4f x0, x1;
	float* pfloat;

	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_gframe = (float*)m_frame->gray->data();
	float* p_dg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	float* pw = (float*)m_weight->data();
	float* pd = (float*)m_key->depth->data();
	Vec4f* px = (Vec4f*)m_key->points->data();

	Intrinsic in0 = m_key->intrinsic;
	Intrinsic in1 = m_frame->intrinsic;
	double f1 = 1.0/in0.f;
	Vec9d R = m_frame->rotation;
	Vec3d t = m_frame->pos;

	Vec9d iR = MatrixToolbox::transpose_3x3(R);
	double it[3] = {
		-(iR[0]*t[0]+iR[1]*t[1]+iR[2]*t[2]),
		-(iR[3]*t[0]+iR[4]*t[1]+iR[5]*t[2]),
		-(iR[6]*t[0]+iR[7]*t[1]+iR[8]*t[2]),
	};
	R = iR;
	t = it;

	for (int i = 0; i < total; i++) {
	
		u = i % m_width - in0.cx;
		v = i / m_width - in0.cy;

		x0[0] = u*f1;
		x0[1] = v*f1;
		x0[2] = 1;
		x0[3] = pd[i];
		px[i] = x0;

		x1[0] = R[0]*x0[0]+R[1]*x0[1]+R[2]*x0[2]+t[0]*x0[3];
		x1[1] = R[3]*x0[0]+R[4]*x0[1]+R[5]*x0[2]+t[1]*x0[3];
		x1[2] = R[6]*x0[0]+R[7]*x0[1]+R[8]*x0[2]+t[2]*x0[3];
		x1[3] = x0[3];
		x1 /= x1[2];

		m[0] = in1.f*x1[0]+in1.cx;
		m[1] = in1.f*x1[1]+in1.cy;
		
		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = p_gframe + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			p_dg[i] = pwarp[i] - p_gkey[i];
			//pw[i] = exp(-p_dg[i]*p_dg[i]/0.16);
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			pwarp[i] = 0;
			pw[i] = 0;
		}
	}
}

void Slam::prepare_residual_epi2(bool with_trans/* = false*/) {


	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	Vec3d m;
	Vec4f x0, x1;
	float* pfloat;

	unsigned char* p_mask = (unsigned char*)m_mask->data();
	float* p_gkey = (float*)m_key->gray->data();
	float* p_gframe = (float*)m_frame->gray->data();
	float* p_dg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	float* pw = (float*)m_weight->data();
	float* pd = (float*)m_key->depth->data();
	Vec4f* px = (Vec4f*)m_key->points->data();

	Intrinsic in0 = m_key->intrinsic;
	Intrinsic in1 = m_frame->intrinsic;
	double f1 = 1.0/in0.f;
	Vec9d R = m_frame->rotation;
	Vec3d t = m_frame->pos;

	Vec9d iR = MatrixToolbox::transpose_3x3(R);
	double it[3] = {
		-(iR[0]*t[0]+iR[1]*t[1]+iR[2]*t[2]),
		-(iR[3]*t[0]+iR[4]*t[1]+iR[5]*t[2]),
		-(iR[6]*t[0]+iR[7]*t[1]+iR[8]*t[2]),
	};
	R = iR;
	t = it;

	for (int i = 0; i < total; i++) {
	
		u = i % m_width - in0.cx;
		v = i / m_width - in0.cy;

		x0[0] = u*f1;
		x0[1] = v*f1;
		x0[2] = 1;
		x0[3] = pd[i];
		px[i] = x0;

		x1[0] = R[0]*x0[0]+R[1]*x0[1]+R[2]*x0[2];
		x1[1] = R[3]*x0[0]+R[4]*x0[1]+R[5]*x0[2];
		x1[2] = R[6]*x0[0]+R[7]*x0[1]+R[8]*x0[2];
		if (with_trans) {
			x1[0] += t[0]*x0[3];
			x1[1] += t[1]*x0[3];
			x1[2] += t[2]*x0[3];
		}
		x1[3] = x0[3];
		x1 /= x1[2];

		m[0] = in1.f*x1[0]+in1.cx;
		m[1] = in1.f*x1[1]+in1.cy;
		
		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			p_mask[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = p_gframe + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			p_dg[i] = pwarp[i] - p_gkey[i];
			//pw[i] = exp(-p_dg[i]*p_dg[i]/0.16);
		}
		else { 
			p_mask[i] = 0;
			p_dg[i] = 0;
			pwarp[i] = 0;
			pw[i] = 0;
		}
	}

}

void Slam::prepare_residual_of1() {


	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	double m[2];
	float* pfloat;

	unsigned char* pm = (unsigned char*)m_mask->data();
	float* pkg = (float*)m_key->gray->data();
	float* pcg = (float*)m_frame->gray->data();
	float* pdg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	Vec2f* pof = (Vec2f*)m_of->data();

	for (int i = 0; i < total; i++) {
	
		u = i % m_width;
		v = i / m_width;

		m[0] = u - pof[i][0];
		m[1] = v - pof[i][1];

		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			pm[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfloat = pcg + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfloat[0], pfloat[1], pfloat[m_width], pfloat[m_width+1], m[0], m[1]);
			pdg[i] = pwarp[i] - pkg[i];
			//pw[i] = exp(-p_dg[i]*p_dg[i]/0.16);
		}
		else { 
			pm[i] = 0;
			pdg[i] = 0;
			pwarp[i] = 0;
			//pw[i] = 0;
		}
	}

}


void Slam::prepare_residual_of2() {


	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	double m[2], w;
	float *pfi1, *pfiu, *pfiv;
	double min_w = Config::min_weight_of3;

	unsigned char* pm = (unsigned char*)m_mask->data();
	float* pi0 = (float*)m_key->gray->data();
	float* pi1 = (float*)m_frame->gray->data();
	float* pit = (float*)m_residual->data();
	float* pwi1 = (float*)m_warp->data();
	Vec2f* put = (Vec2f*)m_of->data();
	Vec2f* pwiu1 = (Vec2f*)m_gradient->data();
	float* pw = (float*)m_weight->data();

	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();
	float* piu1 = (float*)m_frame->gradient[0]->data();
	float* piv1 = (float*)m_frame->gradient[1]->data();


	for (int i = 0; i < total; i++) {
	
		u = i % m_width;
		v = i / m_width;

		m[0] = u + put[i][0];
		m[1] = v + put[i][1];

		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			pm[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;

			pfi1 = pi1 + v * m_width + u;
			pfiu = piu1 + v * m_width + u;
			pfiv = piv1 + v * m_width + u;
			pwi1[i] = SAMPLE_2D(pfi1[0], pfi1[1], pfi1[m_width], pfi1[m_width+1], m[0], m[1]);
			pit[i] = pwi1[i] - pi0[i];
			pwiu1[i][0] = SAMPLE_2D(pfiu[0], pfiu[1], pfiu[m_width], pfiu[m_width+1], m[0], m[1]);
			pwiu1[i][1] = SAMPLE_2D(pfiv[0], pfiv[1], pfiv[m_width], pfiv[m_width+1], m[0], m[1]);
			w = piu0[i]*pwiu1[i][0]+piv0[i]*pwiu1[i][1];
			if (w < min_w) { w = min_w; }
			pw[i] = w;
			//pw[i] = exp(-p_dg[i]*p_dg[i]/0.16);
		}
		else { 
			pm[i] = 0;
			pit[i] = 0;
			pwi1[i] = 0;
			pwiu1[i] = 0;
			pw[i] = 0.01;
		}
	}

}


void Slam::prepare_residual_gof1() {


	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	double m[2], cgu[2];
	float *pfg, *pfgu, *pfgv;

	unsigned char* pm = (unsigned char*)m_mask->data();
	float* pkg = (float*)m_key->gray->data();
	float* pcg = (float*)m_frame->gray->data();
	float* pdg = (float*)m_residual->data();
	float* pwarp = (float*)m_warp->data();
	Vec2f* pof = (Vec2f*)m_of->data();
	Vec2f* pdgu = (Vec2f*)m_grad_residual->data();

	float* pkgu = (float*)m_key->gradient[0]->data();
	float* pkgv = (float*)m_key->gradient[1]->data();
	float* pcgu = (float*)m_frame->gradient[0]->data();
	float* pcgv = (float*)m_frame->gradient[1]->data();


	for (int i = 0; i < total; i++) {
	
		u = i % m_width;
		v = i / m_width;

		m[0] = u - pof[i][0];
		m[1] = v - pof[i][1];

		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			pm[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;
			pfg = pcg + v * m_width + u;
			pfgu = pcgu + v * m_width + u;
			pfgv = pcgv + v * m_width + u;
			pwarp[i] = SAMPLE_2D(pfg[0], pfg[1], pfg[m_width], pfg[m_width+1], m[0], m[1]);
			pdg[i] = pwarp[i] - pkg[i];
			cgu[0] = SAMPLE_2D(pfgu[0], pfgu[1], pfgu[m_width], pfgu[m_width+1], m[0], m[1]);
			cgu[1] = SAMPLE_2D(pfgv[0], pfgv[1], pfgv[m_width], pfgv[m_width+1], m[0], m[1]);
			pdgu[i][0] =  cgu[0]- pkgu[i];
			pdgu[i][1] =  cgu[1]- pkgv[i];
			//pw[i] = exp(-p_dg[i]*p_dg[i]/0.16);
		}
		else { 
			pm[i] = 0;
			pdg[i] = 0;
			pwarp[i] = 0;
			pdgu[i] = 0;
			//pw[i] = 0;
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
		a[2] = -(a[0]*pPts[i][0]+a[1]*pPts[i][1]);
		
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

Vec3d Slam::calc_epi_point() {


	if (!m_key || !m_frame) { return Vec3d(); }

	//Vec2f* pGrad = (Vec2f*)m_gradient->data();
	float* pdg = (float*)m_residual->data();
	Vec3f* pl = (Vec3f*)m_epi_line->data();
	//float* pDepth = (float*)m_depth->data();
	//Vec4f* pPts = (Vec4f*)m_frame->points->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	//double a[3];
	double w;//, temp;
	
	double dg2;
	Vec9d A;
	//Vec3d B;
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }
		//w = 1;//(std::abs(pGx[i])+std::abs(pGy[i]))*std::abs(pDg[i]);
		//temp = m_frame->intrinsic.f*pDepth[i];
		w = pdg[i]*pdg[i];
		const Vec3f& a = pl[i];
		//a[0] = pGrad[i][0]*temp;
		//a[1] = pGrad[i][1]*temp;
		//a[2] = -(a[0]*pPts[i][0]+a[1]*pPts[i][1]);
		
		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		//B[0] += w*a[0]*pDg[i];
		//B[1] += w*a[1]*pDg[i];
		//B[2] += w*a[2]*pDg[i];
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];
	
	//A /= total;
	//B /= total;

	//A[0] += 1;
	//A[4] += 1;
	//A[8] += 1;

	return MatrixToolbox::min_eigen_vector_3x3(A);
	
}
Vec3d Slam::calc_epi_dr() {
	return Vec3d();
}

Vec3d Slam::calc_dr_entropy() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pDg = (float*)m_residual->data();
	float* piu = (float*)m_frame->gradient[0]->data();
	float* piv = (float*)m_frame->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double a[3], l[3], lxt[3], x[3];
	double f = m_frame->intrinsic.f;
	double f1 = 1.0 / f;
	double w, temp;
	Vec9d A;
	Vec3d B;
	const Vec3d& t = m_frame->epi_point;
	int u, v;
	x[2] = 1;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = piu[i];
		l[1] = piv[i];
		l[2] = -(x[0]*piu[i]+x[1]*piv[i]);

		lxt[0] = l[1]*t[2]-l[2]*t[1];
		lxt[1] = l[2]*t[0]-l[0]*t[2];
		lxt[2] = l[0]*t[1]-l[1]*t[0];

		w = lxt[0]*lxt[0]+lxt[1]*lxt[1]+lxt[2]*lxt[2];
		w *= 0.01;

		a[0] = f*(x[1]*l[2]-x[2]*l[1]);
		a[1] = f*(x[2]*l[0]-x[0]*l[2]);
		a[2] = f*(x[0]*l[1]-x[1]*l[0]);

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

Vec3d Slam::calc_dr_entropy2() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pDg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double a[3], l[3], lxt[3], x[3];
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	double w, temp;
	Vec9d A;
	Vec3d B;
	const Vec3d& ep = m_frame->epi_point;
	int u, v;
	x[2] = 1;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = piu[i];
		l[1] = piv[i];
		l[2] = -(x[0]*piu[i]+x[1]*piv[i]);

		lxt[0] = l[1]*ep[2]-l[2]*ep[1];
		lxt[1] = l[2]*ep[0]-l[0]*ep[2];
		lxt[2] = l[0]*ep[1]-l[1]*ep[0];

		w = 0.1*(lxt[0]*lxt[0]+lxt[1]*lxt[1]+lxt[2]*lxt[2]);
		w *= w;

		a[0] = f*(x[1]*l[2]-x[2]*l[1]);
		a[1] = f*(x[2]*l[0]-x[0]*l[2]);
		a[2] = f*(x[0]*l[1]-x[1]*l[0]);

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

Vec3d Slam::calc_dr_entropy3() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pDg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double l[3], le, x[3], b = 0;
	Vec3d a;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	double w, temp;
	const Vec3d& ep = m_frame->epi_point;
	int u, v;
	x[2] = 1;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = piu[i];
		l[1] = piv[i];
		l[2] = -(x[0]*piu[i]+x[1]*piv[i]);

		le = l[0]*ep[0]+l[1]*ep[1]+l[2]*ep[2];

		a[0] += le*f*(x[1]*l[2]-x[2]*l[1]);
		a[1] += le*f*(x[2]*l[0]-x[0]*l[2]);
		a[2] += le*f*(x[0]*l[1]-x[1]*l[0]);
		
		b += le*pDg[i];
	}

	double a2 = a[0]*a[0]+a[1]*a[1]+a[2]*a[2];
	return a * (b/(a2+1));
}


Vec3d Slam::calc_dr_entropy4() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pDg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double l[3], x[3];
	Vec3d a;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	int u, v;
	x[2] = 1;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = piu[i];
		l[1] = piv[i];
		l[2] = -(x[0]*piu[i]+x[1]*piv[i]);

		a[0] += pDg[i]*(x[1]*l[2]-x[2]*l[1]);
		a[1] += pDg[i]*(x[2]*l[0]-x[0]*l[2]);
		a[2] += pDg[i]*(x[0]*l[1]-x[1]*l[0]);

	}

	return a * (f1 / total * 10);
}

Vec3d Slam::calc_dr_lsd2() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pDg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	
	double l[3], x[3], W = 0;
	Vec3d a, r;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	int u, v;
	x[2] = 1;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = f * piu[i];
		l[1] = f * piv[i];
		l[2] = -(u*piu[i]+v*piv[i]);

		a[0] = pDg[i]*(x[1]*l[2]-x[2]*l[1]);
		a[1] = pDg[i]*(x[2]*l[0]-x[0]*l[2]);
		a[2] = pDg[i]*(x[0]*l[1]-x[1]*l[0]);

		r += a;
		W += a.length2();
	}

	return r /= W ;

}


Vec3d Slam::calc_dr_lsd3() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pdg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	float* pw = (float*)m_weight->data();
	
	double a[3], l[3], x[3], A[9], B[3], w, dg;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	int u, v;
	x[2] = 1;
	memset(A, 0, sizeof(A));
	memset(B, 0, sizeof(B));

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		dg = pdg[i];
		w = pw[i];

		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = f * piu[i];
		l[1] = f * piv[i];
		l[2] = -(u*piu[i]+v*piv[i]);

		a[0] = x[1]*l[2]-x[2]*l[1];
		a[1] = x[2]*l[0]-x[0]*l[2];
		a[2] = x[0]*l[1]-x[1]*l[0];

		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*dg;
		B[1] += w*a[1]*dg;
		B[2] += w*a[2]*dg;
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];

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

Vec3d Slam::calc_dr_lsd4() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pdg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	float* pw = (float*)m_weight->data();
	
	double a[3], l[3], x[3], A[9], B[3], w, dg;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	int u, v;
	x[2] = 1;
	memset(A, 0, sizeof(A));
	memset(B, 0, sizeof(B));
	const Vec3d& t = m_frame->pos;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		dg = pdg[i];
		//w = pw[i];

		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = f * piu[i];
		l[1] = f * piv[i];
		l[2] = -(u*piu[i]+v*piv[i]);

		a[0] = x[1]*l[2]-x[2]*l[1];
		a[1] = x[2]*l[0]-x[0]*l[2];
		a[2] = x[0]*l[1]-x[1]*l[0];
		w = a[0]*t[0]+a[1]*t[1]+a[2]*t[2];
		w *= w;
		pw[i] = w;

		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*dg;
		B[1] += w*a[1]*dg;
		B[2] += w*a[2]*dg;
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];

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

Vec3d Slam::calc_dr_epi2() {

	if (!m_key || !m_frame) { return Vec3d(); }

	float* pdg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	float* pw = (float*)m_weight->data();
	
	double a[3], l[3], x[3], A[9], B[3], w, dg;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	int u, v;
	x[2] = 1;
	memset(A, 0, sizeof(A));
	memset(B, 0, sizeof(B));
	const Vec3d& e = m_frame->epi_point;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cx;
		dg = pdg[i];

		x[0] = f1 * u;
		x[1] = f1 * v;

		l[0] = f * piu[i];
		l[1] = f * piv[i];
		l[2] = -(u*piu[i]+v*piv[i]);

		w = dg*(l[0]*e[0]+l[1]*e[1]+l[2]*e[2]);
		w = exp(-w*f1*5);
		pw[i] = w;
		//w = 1;

		a[0] = x[1]*l[2]-x[2]*l[1];
		a[1] = x[2]*l[0]-x[0]*l[2];
		a[2] = x[0]*l[1]-x[1]*l[0];

		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*dg;
		B[1] += w*a[1]*dg;
		B[2] += w*a[2]*dg;
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];

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


double Slam::calc_dl_entropy2() {

	if (!m_key || !m_frame) { return 0; }

	float* pDg = (float*)m_residual->data();
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	int total = m_width * m_height;
	unsigned char* pMask = (unsigned char*)m_mask->data();
	float* pd = (float*)m_key->depth->data();
	
	double a, l[3];
	double f = m_key->intrinsic.f;
	double w;
	double A = 0;
	double B = 0;
	const Vec3d& e = m_frame->epi_point;
	int u, v;

	for (int i = 0; i < total; i++) {
		if (!pMask[i]) { continue; }

		u = i%m_width-m_frame->intrinsic.cx;
		v = i/m_width-m_frame->intrinsic.cx;

		l[0] = pd[i]*f*piu[i]*e[0];
		l[1] = pd[i]*f*piv[i]*e[1];
		l[2] = -pd[i]*(u*piu[i]+v*piv[i])*e[2];
		a = l[0]+l[1]+l[2];

		w = 1.0;

		A += w*a*a;
		B += w*a*pDg[i];
	}

	A += 1;	
	return B / A;
}

Vec3d Slam::calc_t_entropy() {


	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v, count = 0;
	double f = m_frame->intrinsic.f;
	double f1 = 1.0 / f;
	float* piu = (float*)m_frame->gradient[0]->data();
	float* piv = (float*)m_frame->gradient[1]->data();

	float* pdg = (float*)m_residual->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	double w;
	Vec3d t;
	
	
	for (int i = 0; i < total; i++) {

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;

		if (!pMask[i]) { continue; }
		count++;
		w = pdg[i];
		t[0] += w * piu[i];
		t[1] += w * piv[i];
		t[2] -= w * (u*f1*piu[i]+v*f1*piv[i]);
	}
	t /= count;
	return t;

}

Vec3d Slam::calc_epi_point_entropy2() {

	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v, count = 0;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();

	float* pdg = (float*)m_residual->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	double w;
	Vec3d ep;
	
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;

		count++;
		w = pdg[i];
		ep[0] += w * piu[i];
		ep[1] += w * piv[i];
		ep[2] -= w * (u*f1*piu[i]+v*f1*piv[i]);
	}
	ep.normalize();
	return ep;
}

Vec3d Slam::calc_dt_entropy5() {

	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v, count = 0;
	double f = m_key->intrinsic.f;
	double f1 = 1.0 / f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();

	float* pdg = (float*)m_residual->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	double w;
	Vec3d a;
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;

		count++;
		w = pdg[i];
		a[0] += w * piu[i];
		a[1] += w * piv[i];
		a[2] -= w * (u*f1*piu[i]+v*f1*piv[i]);
	}

	return a /= count;
}

Vec3d Slam::calc_dt_lsd2() {

	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v, count = 0;
	double f = m_key->intrinsic.f;
	//double f1 = 1.0 / f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	float* pd = (float*)m_key->depth->data();
	float* pdg = (float*)m_residual->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();

	double W = 0;
	Vec3d a, X;
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;
		count++;
		a[0] = pdg[i] * pd[i] * f * piu[i];
		a[1] = pdg[i] * pd[i] * f * piv[i];
		a[2] = -pdg[i] * pd[i] * (u*piu[i]+v*piv[i]);

		X += a;

		W += a.length2();
	}

	return X /= (W+10000);
}


Vec3d Slam::calc_dt_lsd3() {

	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v, count = 0;
	double f = m_key->intrinsic.f;
	//double f1 = 1.0 / f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	float* pd = (float*)m_key->depth->data();
	float* pdg = (float*)m_residual->data();
	float* pw = (float*)m_weight->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();

	double w, A[9], B[3], dg;
	Vec3d a;
	memset(A, 0, sizeof(A));
	memset(B, 0, sizeof(B));
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;
		dg = pdg[i];
		w = pw[i];
		a[0] = pd[i] * f * piu[i];
		a[1] = pd[i] * f * piv[i];
		a[2] = -pd[i] * (u*piu[i]+v*piv[i]);

		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*dg;
		B[1] += w*a[1]*dg;
		B[2] += w*a[2]*dg;
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];

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


Vec3d Slam::calc_dt_lsd4() {

	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v, count = 0;
	double f = m_key->intrinsic.f;
	//double f1 = 1.0 / f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	float* pd = (float*)m_key->depth->data();
	float* pdg = (float*)m_residual->data();
	float* pw = (float*)m_weight->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();

	double w, A[9], B[3], dg;
	Vec3d a;
	memset(A, 0, sizeof(A));
	memset(B, 0, sizeof(B));
	Vec3d t = m_frame->pos;

	if (t.length2() < 0.001) { t[0] = 1; t[1] = 1; }
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;
		dg = pdg[i];
		//w = 1;
		a[0] = pd[i] * f * piu[i];
		a[1] = pd[i] * f * piv[i];
		a[2] = -pd[i] * (u*piu[i]+v*piv[i]);
		w = a[0]*t[0]+a[1]*t[1]+a[2]*t[2];
		w *= w;
		pw[i] = w;

		A[0] += w*a[0]*a[0];
		A[1] += w*a[0]*a[1];
		A[2] += w*a[0]*a[2];
		A[4] += w*a[1]*a[1];
		A[5] += w*a[1]*a[2];
		A[8] += w*a[2]*a[2];
		
		B[0] += w*a[0]*dg;
		B[1] += w*a[1]*dg;
		B[2] += w*a[2]*dg;
	}
	
	A[3] = A[1];
	A[6] = A[2];
	A[7] = A[5];

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


Vec3d Slam::calc_e_epi2() {

	if (!m_key || !m_frame) { return Vec3d(); }

	int total = m_width * m_height;
	int u, v;//, count = 0;
	//double f = m_key->intrinsic.f;
	double f1 = 1.0 / m_key->intrinsic.f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	float* pdg = (float*)m_residual->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	double w;
	Vec3d ep;
	
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;

		//count++;
		w = pdg[i];
		ep[0] += w * piu[i];
		ep[1] += w * piv[i];
		ep[2] -= w * (u*f1*piu[i]+v*f1*piv[i]);
	}
	ep.normalize();
	return ep;

}

void Slam::calc_du_of1() {


	if (!m_key || !m_frame) { return; }

	int total = m_width * m_height;
	int u, v;//, count = 0;
	Vec2f iu;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	//float* pd = (float*)m_key->depth->data();
	float* pdg = (float*)m_residual->data();
	Vec2f* pof = (Vec2f*)m_of->data();
	//float* pw = (float*)m_weight->data();
	unsigned char* pm = (unsigned char*)m_mask->data();

	for (int i = 0; i < total; i++) {

		if (!pm[i]) { continue; }

		iu[0] = piu[i];
		iu[1] = piv[i];
		pof[i] += iu * (pdg[i]/(iu.length2() + 0.01));
	}

}


void Slam::calc_du_of2() {


	if (!m_key || !m_frame) { return; }

	int total = m_width * m_height;
	int u, v;//, count = 0;
	Vec2f iu;
	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();
	Vec2f* pwiu1 = (Vec2f*)m_gradient->data();
	//float* pd = (float*)m_key->depth->data();
	float* pit = (float*)m_residual->data();
	Vec2f* put = (Vec2f*)m_of->data();
	//float* pw = (float*)m_weight->data();
	unsigned char* pm = (unsigned char*)m_mask->data();

	for (int i = 0; i < total; i++) {

		if (!pm[i]) { continue; }

		iu[0] = piu0[i] + pwiu1[i][0];
		iu[1] = piv0[i] + pwiu1[i][1];
		put[i] -= iu * (pit[i]*2.0/(iu.length2() + 1));
	}

}

bool Slam::calc_du_of3() {


	std::cout << "Slam::calc_du_of3" << std::endl;

	if (!m_key || !m_frame) { return true; }
	int total = m_width * m_height;
	float* pwit = (float*)m_residual->data();
	Vec2f* pdut = (Vec2f*)m_dut->data();
	Vec2f* put = (Vec2f*)m_of->data();

	Vec2f* pwiu1 = (Vec2f*)m_gradient->data();
	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();
	float* pw = (float*)m_weight->data();
	unsigned char* pm = (unsigned char*)m_mask->data();

	int u, v, u2, v2;
	float w, W, iu0[2], iu1[2];//, d;
	double A[4], iA[4], B[2], it, iuu[2], idet, iuiu[4];
	double lamda = Config::du_smooth_lamda_of3;
	double s = Config::stable_factor_of3;

	//double dgu, dgt;
	//const double& su = Config::sigma2_dgdu;
	//const double& st = Config::sigma2_dgdt;
	//Vec2f dut;
	int offid[9] = { 
		-m_width-1, -m_width, -m_width+1,
		-1, 0, 1,
		m_width-1, m_width, m_width+1
	};
	int offx[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
	int offy[9] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

	for (int i = 0; i < total; i++) {

		u = i % m_width;
		v = i / m_width;
		//W = 0;
		//dut = 0;
		memset(A, 0, sizeof(A));
		memset(B, 0, sizeof(B));

		for (int k = 0; k < 9; k++) {
			u2 = u + offx[k];
			v2 = v + offy[k];
			if (u2 >= 0 && u2 < m_width && v2 >= 0 && v2 < m_height) {

					iu0[0] = piu0[i+offid[k]];
					iu0[1] = piv0[i+offid[k]];
					iuu[0] = put[i+offid[k]][0]-put[i][0];
					iuu[1] = put[i+offid[k]][1]-put[i][1];
					w = pw[i+offid[k]];
					it = pwit[i+offid[k]];

					iuiu[0] = iu0[0]*iu0[0];
					iuiu[1] = iu0[0]*iu0[1];
					iuiu[2] = iu0[1]*iu0[0];
					iuiu[3] = iu0[1]*iu0[1];

					if (pm[i]) {
						A[0] += iuiu[0];
						A[1] += iuiu[1];
						//A[2] += iuiu[2];
						A[3] += iuiu[3];

						B[0] += iu0[0]*it;
						B[1] += iu0[1]*it;

					}

					if (Config::use_i1_constrain_of3) {
						
						iu1[0] = pwiu1[i+offid[k]][0];
						iu1[1] = pwiu1[i+offid[k]][1];
						A[0] += iu1[0]*iu1[0];
						A[1] += iu1[0]*iu1[1];
						//A[2] += iu1[1]*iu1[0];
						A[3] += iu1[1]*iu1[1];

						B[0] += iu1[0]*it;
						B[1] += iu1[1]*it;

					}


					iuiu[0] += s;
					iuiu[3] += s;

					A[0] += iuiu[0]*lamda*w;
					A[1] += iuiu[1]*lamda*w;
					//A[2] += iuiu[2]*lamda*w;
					A[3] += iuiu[3]*lamda*w;

					B[0] += -lamda*w*(iuiu[0]*iuu[0]+
						iuiu[1]*iuu[1]);
					B[1] += -lamda*w*(iuiu[2]*iuu[0]+
						iuiu[3]*iuu[1]);

			}
		}

		A[2] = A[1];

		//A[0] += s;
		//A[3] += s;

		idet = 1.0/(A[0]*A[3]-A[1]*A[2]);
		iA[0] = A[3]*idet;
		iA[1] = -A[2]*idet;
		iA[2] = -A[1]*idet;
		iA[3] = A[0] *idet;

		pdut[i][0] = -(iA[0]*B[0]+iA[1]*B[1]);
		pdut[i][1] = -(iA[2]*B[0]+iA[3]*B[1]);

	}

	//return is ok?
	return false;


}

bool Slam::calc_e_dr_of3(bool only_dr/* = false*/) {


	std::cout << "Slam::calc_e_dr_of3" << std::endl;

	if (!m_key || !m_frame) { return true; }
	int total = m_width * m_height;
	Vec2f* put = (Vec2f*)m_of->data();
	float* pw = (float*)m_weight->data();
	Vec2f* pdut = (Vec2f*)m_dut->data();

	 Intrinsic in = m_key->intrinsic;
	 double f1 = 1.0 / in.f;
	 double m0[3], m1[3], x0[3], x1[3], x10[3];
	 Vec9d iR = MatrixToolbox::inv_matrix_3x3(m_frame->rotation);
	 const double* R = iR.val;
	 const double* e = m_frame->epi_point.val;
	 double c, w, ae[3], ar[3], ar0[3];
	 double Ae[9], Ar[9], Br[3];

	 memset(Ae, 0, sizeof(Ae));
	 memset(Ar, 0, sizeof(Ar));
	 memset(Br, 0, sizeof(Br));
 	 m0[2] = 1;
 	 m1[2] = 1;
	 x10[2] = 1;
	 x0[2] = 1;
	 x1[2] = 1;

	for (int i = 0; i < total; i++) {

		m0[0] = i % m_width - in.cx;
		m0[1] = i / m_width - in.cy;
		m1[0] = m0[0] + put[i][0];
		m1[1] = m0[1] + put[i][1];
		x0[0] = m0[0]*f1;
		x0[1] = m0[1]*f1;
		x10[0] = m1[0]*f1;
		x10[1] = m1[1]*f1;
		x1[0] = R[0]*x10[0]+R[1]*x10[1]+R[2]*x10[2];
		x1[1] = R[3]*x10[0]+R[4]*x10[1]+R[5]*x10[2];
		x1[2] = R[6]*x10[0]+R[7]*x10[1]+R[8]*x10[2];

		pdut[i][0] = x1[0]*in.f/x1[2]-m0[0];
		pdut[i][1] = x1[1]*in.f/x1[2]-m0[1];

		ae[0] = x0[1]*x1[2]-x0[2]*x1[1];
		ae[1] = x0[2]*x1[0]-x0[0]*x1[2];
		ae[2] = x0[0]*x1[1]-x0[1]*x1[0];

		c = ae[0]*e[0]+ae[1]*e[1]+ae[2]*e[2];
		w = exp(-c*c*Config::epi_sigma2_of3);
		pw[i] = w;

		Ae[0] += w*ae[0]*ae[0];
		Ae[1] += w*ae[0]*ae[1];
		Ae[2] += w*ae[0]*ae[2];
		Ae[4] += w*ae[1]*ae[1];
		Ae[5] += w*ae[1]*ae[2];
		Ae[8] += w*ae[2]*ae[2];

		ar0[0] = x1[1]*e[2]-x1[2]*e[1];
		ar0[1] = x1[2]*e[0]-x1[0]*e[2];
		ar0[2] = x1[0]*e[1]-x1[1]*e[0];

		ar[0] = x0[1]*ar0[2]-x0[2]*ar0[1];
		ar[1] = x0[2]*ar0[0]-x0[0]*ar0[2];
		ar[2] = x0[0]*ar0[1]-x0[1]*ar0[0];

		Ar[0] += w*ar[0]*ar[0];
		Ar[1] += w*ar[0]*ar[1];
		Ar[2] += w*ar[0]*ar[2];
		Ar[4] += w*ar[1]*ar[1];
		Ar[5] += w*ar[1]*ar[2];
		Ar[8] += w*ar[2]*ar[2];

		Br[0] -= w*c*ar[0];
		Br[1] -= w*c*ar[1];
		Br[2] -= w*c*ar[2];
	}

	Ae[3] = Ae[1];
	Ae[6] = Ae[2];
	Ae[7] = Ae[5];

	Ar[3] = Ar[1];
	Ar[6] = Ar[2];
	Ar[7] = Ar[5];
	
	//A /= total;
	//B /= total;

	Ar[0] += 0.1;
	Ar[4] += 0.1;
	Ar[8] += 0.1;


	if (!only_dr) {
		m_frame->epi_point = MatrixToolbox::min_eigen_vector_3x3(Ae);	
	}
	
	Vec9d iAr = MatrixToolbox::inv_matrix_3x3(Ar);
	double dr[3] = {
			iAr[0]*Br[0]+iAr[1]*Br[1]+iAr[2]*Br[2],
			iAr[3]*Br[0]+iAr[4]*Br[1]+iAr[5]*Br[2],
			iAr[6]*Br[0]+iAr[7]*Br[1]+iAr[8]*Br[2],

	};
	MatrixToolbox::update_rotation(m_frame->rotation, dr);
	m_frame->rotation_warp(m_warp);

	return true;
}

bool Slam::calc_t_of3() {

	std::cout << "Slam::calc_t_of3" << std::endl;

	if (!m_key || !m_frame) { return true; }

	if (m_frame->epi_point.dot(m_frame->pos) < 0) {
		m_frame->epi_point *= -1;
	}

	double l = m_frame->pos.length();
	const double* e = m_frame->epi_point.val;

	int total = m_width * m_height;
	float* pw = (float*)m_weight->data();
	Vec2f* pdut = (Vec2f*)m_dut->data();
	float* pd = (float*)m_key->depth->data();

	 Intrinsic in = m_key->intrinsic;
	 double m0[3], m1[3];
	 double d, du[2], du0[2], w, a[2], AL = 0, A = 0;
	 double dl, dua;

	float* piu1 = (float*)m_frame->gradient[0]->data();
	float* piv1 = (float*)m_frame->gradient[1]->data();
	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();

	

	 m0[2] = 1;

	for (int i = 0; i < total; i++) {

		d = pd[i];

		m0[0] = i % m_width - in.cx;
		m0[1] = i / m_width - in.cy;
		dl = d*l;
		m1[0] = dl*in.f*e[0]+m0[0];
		m1[1] = dl*in.f*e[1]+m0[1];
		m1[2] = dl*e[2]+m0[2];
		m1[0] /= m1[2];
		m1[1] /= m1[2];

		du0[0] = pdut[i][0];
		du0[1] = pdut[i][1];
		du[0] = du0[0]-(m1[0]-m0[0]);
		du[1] = du0[1]-(m1[1]-m0[1]);

		piu0[i] = m1[0]-m0[0];
		piv0[i] = m1[1]-m0[1];
		piu1[i] = du[0];
		piv1[i] = du[1];

		a[0] = d*(in.f*e[0]-m0[0]*e[2]);
		a[1] = d*(in.f*e[1]-m0[0]*e[2]);

		w = du0[0]*a[0]+du0[1]*a[1];
		//if (w < 0) { w = 0; }
		pw[i] = w;

		dua = du[0]*a[0]+du[1]*a[1];

		AL += w*dua;
		A += w*(a[0]*a[0]+a[1]*a[1]);

	}

	l += AL/(A+0.001);
	if (l < 0) { 
		l *= -1; 
		m_frame->epi_point *= -1;
	}

	m_frame->pos = m_frame->epi_point * l;

	return true;

}

void Slam::update_depth_of3() {


	std::cout << "Slam::update_depth_of3" << std::endl;

	if (!m_key || !m_frame) { return; }

	const double* t = m_frame->pos.val;

	int total = m_width * m_height;
	Vec2f* pdut = (Vec2f*)m_dut->data();
	float* pwd = (float*)m_key->depth->data();
	float* pdw = (float*)m_key->depth_weight->data();


	 Intrinsic in = m_key->intrinsic;
	 double wd, dw, m0[2], a[2];

	for (int i = 0; i < total; i++) {

		m0[0] = i % m_width - in.cx;
		m0[1] = i / m_width - in.cy;
		a[0] = in.f*t[0]-m0[0]*t[2];
		a[1] = in.f*t[1]-m0[0]*t[2];

		wd = pdut[i][0]*a[0]+pdut[i][1]*a[1];
		dw = a[0]*a[0]+a[1]*a[1];

		pwd[i] += wd;
		pdw[i] += dw;
	}
}

void Slam::unproject_points_of3() {

	std::cout << "Slam::unproject_points_of3" << std::endl;

	if (!m_key || !m_frame) { return; }

	int total = m_width * m_height;
	float* pwd = (float*)m_key->depth->data();
	float* pdw = (float*)m_key->depth_weight->data();
	Vec4f* px = (Vec4f*)m_key->points->data();

	 Intrinsic in = m_key->intrinsic;
	 double f1 = 1.0/in.f;
	 Vec4f x;
	 double wd, dw, m0[2];

	for (int i = 0; i < total; i++) {

		m0[0] = i % m_width - in.cx;
		m0[1] = i / m_width - in.cy;
		dw = pdw[i];
		x[0] = dw * m0[0] * f1;
		x[1] = dw * m0[1] * f1;
		x[2] = dw;
		x[3] = pwd[i];
		px[i] = x;

	}

}


void Slam::calc_du_gof1() {


	if (!m_key || !m_frame) { return; }

	int total = m_width * m_height;
	int u, v;//, count = 0;
	Vec2f iu;
	//float* piu = (float*)m_key->gradient[0]->data();
	//float* piv = (float*)m_key->gradient[1]->data();
	//float* pdg = (float*)m_residual->data();
	Vec2f* pof = (Vec2f*)m_of->data();
	Vec2f* pdgu = (Vec2f*)m_grad_residual->data();
	unsigned char* pm = (unsigned char*)m_mask->data();
	float* guu = (float*)m_grad_grad[0]->data();
	float* guv = (float*)m_grad_grad[1]->data();
	float* gvu = (float*)m_grad_grad[2]->data();
	float* gvv = (float*)m_grad_grad[3]->data();
	double det, J[4], iJ[4], l = 1;

	for (int i = 0; i < total; i++) {

		if (!pm[i]) { continue; }

		J[0] = guu[i]+l;
		J[1] = guv[i];
		J[2] = gvu[i];
		J[3] = gvv[i]+l;
		det = J[0]*J[3]-J[1]*J[2];
		iJ[0] = J[3]/det;
		iJ[1] = -J[2]/det;
		iJ[2] = -J[1]/det;
		iJ[3] = J[0]/det;

		pof[i][0] += iJ[0]*pdgu[i][0]+iJ[1]*pdgu[i][1];
		pof[i][1] += iJ[2]*pdgu[i][0]+iJ[3]*pdgu[i][1];
	}

}

void Slam::smooth_of_of1() {

	std::cout << "Slam::smooth_of_of1" << std::endl;

	if (!m_key || !m_frame) { return; }
	int total = m_width * m_height;
	float* pg = (float*)m_key->gray->data();
	float* pdg = (float*)m_residual->data();
	Vec2f* pof = (Vec2f*)m_of->data();

	int u, v, u2, v2;
	float w, W;//, d;
	double dgu, dgt;
	const double& su = Config::sigma2_dgdu;
	const double& st = Config::sigma2_dgdt;
	Vec2f of;
	int offid[9] = { 
		-m_width-1, -m_width, -m_width+1,
		-1, 0, 1,
		m_width-1, m_width, m_width+1
	};
	int offx[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
	int offy[9] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

	for (int i = 0; i < total; i++) {

		u = i % m_width;
		v = i / m_width;
		W = 0;
		of = 0;
		for (int k = 0; k < 9; k++) {
			u2 = u + offx[k];
			v2 = v + offy[k];
			if (u2 >= 0 && u2 < m_width && v2 >= 0 && v2 < m_height) {
				dgu = pg[i+offid[k]]-pg[i];
				dgt = pdg[i+offid[k]];
				w = exp(-dgu*dgu*su-dgt*dgt*st);
				of += pof[i+offid[k]] * w;
				W += w;
			}
		}
		pof[i] = of/=W;
	}
}


void Slam::smooth_of_of2() {

	std::cout << "Slam::smooth_of_of2" << std::endl;

	if (!m_key || !m_frame) { return; }
	int total = m_width * m_height;
	//float* pi0 = (float*)m_key->gray->data();
	//float* pwit = (float*)m_residual->data();
	Vec2f* put = (Vec2f*)m_of->data();

	Vec2f* pwiu1 = (Vec2f*)m_gradient->data();
	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();
	float* pw = (float*)m_weight->data();

	int u, v, u2, v2;
	float w, W;//, iu0[2];//, d;
	//double A[4], iA[4], idet;
	//double dgu, dgt;
	//const double& su = Config::sigma2_dgdu;
	//const double& st = Config::sigma2_dgdt;
	Vec2f dut;
	int offid[9] = { 
		-m_width-1, -m_width, -m_width+1,
		-1, 0, 1,
		m_width-1, m_width, m_width+1
	};
	int offx[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
	int offy[9] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

	for (int i = 0; i < total; i++) {

		u = i % m_width;
		v = i / m_width;
		W = 0;
		dut = 0;
		//iu0[0] = piu0[i];
		//iu0[1] = piv0[i];

		for (int k = 0; k < 9; k++) {
			u2 = u + offx[k];
			v2 = v + offy[k];
			if (u2 >= 0 && u2 < m_width && v2 >= 0 && v2 < m_height) {
				w = pw[i+offid[k]];
				dut += (put[i+offid[k]]-put[i])*w;
				W += w;
			}
		}

		put[i] += (dut/=W);

		// A[0] = iu0[0]*iu0[0]+W;
		// A[1] = iu0[0]*iu0[1];
		// A[2] = A[1];
		// A[3] = iu0[1]*iu0[1]+W;
		// idet = 1.0/(A[0]*A[3]-A[1]*A[2]);
		// iA[0] = A[3]*idet;
		// iA[1] = -A[2]*idet;
		// iA[2] = -A[1]*idet;
		// iA[3] = A[0] *idet;

		// put[i][0] += iA[0]*dut[0]+iA[1]*dut[1];
		// put[i][1] += iA[2]*dut[0]+iA[3]*dut[1];
	}
}


void Slam::wipe_depth(const Vec3d& t) {
	

	assert(0);

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


void Slam::create_keyframe(Image* image) {

	Camera* pre_key = m_key;
	m_key = m_frame;
	m_frame = NULL;

	//SpaceToolbox::init_intrinsic(m_key->intrinsic, 45, m_width, m_height);
	
	//image->copy_to(m_key->original);
	//image->gray(m_key->gray);

	m_key->depth = new CvImage(m_width, m_height, Image::Float32);
	m_key->depth_weight = new CvImage(m_width, m_height, Image::Float32);

	m_key->mask = new CvImage(m_width, m_height, Image::UByte);
	m_key->residual = new CvImage(m_width, m_height, Image::Float32);
	m_key->warp_gradient = new CvImage(m_width, m_height, Image::Float32, 2);
	m_key->warp = new CvImage(m_width, m_height, Image::Float32);
	m_key->of_weight = new CvImage(m_width, m_height, Image::Float32);
	m_key->epi_weight = new CvImage(m_width, m_height, Image::Float32);
	m_key->optical_flow = new CvImage(m_width, m_height, Image::Float32, 2);
	m_key->dut = new CvImage(m_width, m_height, Image::Float32, 2);

	//if (pre_key) {
	//	pre_key->depth->set(0);
	//}
	m_key->depth->set(0.1);
	m_key->depth_weight->set(10);


	m_key->gradient[0]->sobel_x(m_grad_grad[0]);
	m_key->gradient[0]->sobel_y(m_grad_grad[1]);
	m_key->gradient[1]->sobel_x(m_grad_grad[2]);
	m_key->gradient[1]->sobel_y(m_grad_grad[3]);

	m_keyframes.push_back(m_key);
	m_cameras[m_camera_count] = m_key;
	m_camera_count++;

}
void Slam::update_depth() {


	Vec2f* pGrad = (Vec2f*)m_gradient->data();
	float* pDg = (float*)m_residual->data();
	float* pDepth = (float*)m_depth->data();
	Vec4f* pPts = (Vec4f*)m_frame->points->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	float* p_kd = (float*)m_key->depth->data();
	float* p_kg = (float*)m_key->gray->data();
	const Vec3d& t = m_frame->pos;
	int total = m_width * m_height;
	double iuux[3];
	double a, a2, temp;
	double ddi, dds, l1, l2, wij, gg, dd, wsum;
	double A = m_frame->intrinsic.f;
	int u, v, u2, v2, j;

	int offsetid[9] = { 
		-m_width-1, -m_width, -m_width+1,
		-1, 0, 1,
		m_width-1, m_width, m_width+1
	};
	int offsetx[9] = {
		-1, 0, 1, -1, 0, 1, -1, 0, 1
	};
	int offsety[9] = {
		-1, -1, -1, 0, 0, 0, 1, 1, 1
	};

	for (int i = 0; i < total; i++) {

		u = i % m_width;
		v = i / m_width;
		
		if (pMask[i]) {
			//temp = m_frame->intrinsic.f*pDepth[i]/p_kd[i];
			temp = m_frame->intrinsic.f;//*pDepth[i]/p_kd[i];
			iuux[0] = pGrad[i][0]*temp;
			iuux[1] = pGrad[i][1]*temp;
			iuux[2] = -(iuux[0]*pPts[i][0]+iuux[1]*pPts[i][1]);
			a = iuux[0]*t[0]+iuux[1]*t[1]+iuux[2]*t[2];
			a2 = a*a;
			ddi = a*pDg[i]/(A+a2);
		}
		else {
			a = 0;
			a2 = 0;
			ddi = 0;
		}
		dds = 0;
		wsum = 0;
		for (int k = 0; k < 9; k++) {
			u2 = u + offsetx[k];
			v2 = v + offsety[k];
			if (u2 >= 0 && u2 < m_width && v2 >=0 && v2 < m_height) {
				gg = p_kg[i+offsetid[k]]-p_kg[i];
				dd = p_kd[i+offsetid[k]]-p_kd[i];
				wij = exp(-gg*gg/0.01);
				dds += wij * dd;
				wsum += wij;
			}
		}
		dds /= wsum;
		
		l1 = a2/(A+a2);
		l2 = A/(A+a2);
		p_kd[i] += l1*ddi + l2*dds;
	}
}

void Slam::update_depth_lsd2() {

	if (!m_key || !m_frame) { return; }

	int total = m_width * m_height;
	int u, v;
	double f = m_key->intrinsic.f;
	//double f1 = 1.0 / f;
	float* piu = (float*)m_key->gradient[0]->data();
	float* piv = (float*)m_key->gradient[1]->data();
	float* pd = (float*)m_key->depth->data();
	float* pdg = (float*)m_residual->data();
	unsigned char* pMask = (unsigned char*)m_mask->data();
	double a, l[3], W = 0;
	const Vec3d& t = m_frame->pos;
	
	for (int i = 0; i < total; i++) {

		if (!pMask[i]) { continue; }

		u = i % m_width - m_frame->intrinsic.cx;
		v = i / m_width - m_frame->intrinsic.cy;
		l[0] = f * piu[i];
		l[1] = f * piv[i];
		l[2] = -(u*piu[i]+v*piv[i]);
		a = l[0]*t[0]+l[1]*t[1]+l[2]*t[2];
		pd[i] += a*pdg[i]/(a*a+f);
	}
}

void Slam::smooth_depth_lsd2() {
	
	std::cout << "Slam::smooth_depth_lsd2" << std::endl;

	if (!m_key || !m_frame) { return; }
	int total = m_width * m_height;
	float* pg = (float*)m_key->gray->data();
	float* pd = (float*)m_key->depth->data();
	int u, v, u2, v2;
	float dg, w, W, d;
	int offid[9] = { 
		-m_width-1, -m_width, -m_width+1,
		-1, 0, 1,
		m_width-1, m_width, m_width+1
	};
	int offx[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
	int offy[9] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

	for (int i = 0; i < total; i++) {

		u = i % m_width;
		v = i / m_width;
		W = 0;
		d = 0;
		for (int k = 0; k < 9; k++) {
			u2 = u + offx[k];
			v2 = v + offy[k];
			if (u2 >= 0 && u2 < m_width && v2 >= 0 && v2 < m_height) {
				dg = pg[i+offid[k]]-pg[i];
				w = 1;//exp(-dg*dg/0.25);
				d += w * pd[i+offid[k]];
				W += w;
			}
		}
		pd[i] = d/W;
	}
}

void Slam::build_of3(BuildFlag flag) {

	Image* image = NULL;
	Image* resized = NULL;
	int times = 0;
	int steps = 0;
	bool ok;
	while(true) {
		if ((flag & BuildReadFrame) && m_source->read(image)) {
			image->resize(resized);
			initialize(resized);
			preprocess(resized);
		}
		times = 0;
		while(flag & BuildOpticalFlow) {
			prepare_residual_of2();
			ok = calc_du_of3();
			m_of->add(m_dut);
			if (
				!(flag & BuildIterate) || ok || 
				times >= Config::max_iterate_times
			) { break; }
			times++;
		}
		while(flag & BuildEpipolar) {
			ok = calc_e_dr_of3();
			if (!(flag & BuildIterate) || ok) { break; }
		}
		while(flag & BuildTranslate) {
			ok = calc_t_of3();
			if (!(flag & BuildIterate) || ok) { break; }
		}
		if (flag & BuildDepth) {
			update_depth_of3();
			unproject_points_of3();
		}
		if (flag & BuildKeyframe) {
			update_keyframe(resized);
		}
		steps++;
		if (!(flag & BuildSequence) || 
			steps >= Config::build_steps)
		{ break; }
		if (m_display_delegate) {
			m_display_delegate->display_with(this);
		}
	}
	if (image) { delete image; }
	if (resized) { delete resized; }	

}
void Slam::build_of4(BuildFlag flag) {


	Image* image = NULL;
	Image* resized = NULL;
	int times = 0;
	int steps = 0;
	bool ok;
	while(true) {
		if ((flag & BuildReadFrame) && m_source->read(image)) {
			image->resize(resized);
			initialize(resized);
			preprocess(resized);
		}
		times = 0;
		while(flag & BuildOpticalFlow) {
			prepare_du_of4();
			ok = calc_du_of4();
			//->add(m_dut);
			if (!(flag & BuildIterate) || ok || 
				times >= Config::max_iterate_times
			) { break; }
			times++;
		}
		
		times = 0;		
		while(flag & BuildEpipolar) {
			ok = calc_e_dr_of4();
			if (!(flag & BuildIterate) || ok || 
				times >= Config::max_iterate_times
			) { break; }
			times++;
		}
		/*
		while(flag & BuildTranslate) {
			ok = calc_t_of3();
			if (!(flag & BuildIterate) || ok) { break; }
		}
		if (flag & BuildDepth) {
			update_depth_of3();
			unproject_points_of3();
		}
		*/
		if (flag & BuildKeyframe) {
			update_keyframe(resized);
		}
		steps++;
		if (!(flag & BuildSequence) || 
			steps >= Config::build_steps)
		{ break; }
		if (m_display_delegate) {
			m_display_delegate->display_with(this);
		}
	}
	if (image) { delete image; }
	if (resized) { delete resized; }	

}

void Slam::prepare_du_of4() {


	if (!m_key || !m_frame) { return; }
	
	int total = m_width * m_height;
	int u, v;
	double m[2], w;
	float *pfi1, *pfiu, *pfiv;
	double min_w = Config::min_weight_of3;

	unsigned char* pm = (unsigned char*)m_key->mask->data();
	float* pi0 = (float*)m_key->gray->data();
	float* pi1 = (float*)m_frame->gray->data();
	float* pit = (float*)m_key->residual->data();
	float* pwi1 = (float*)m_key->warp->data();
	Vec2f* put = (Vec2f*)m_key->optical_flow->data();
	Vec2f* pwiu1 = (Vec2f*)m_key->warp_gradient->data();
	float* pw = (float*)m_key->of_weight->data();

	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();
	float* piu1 = (float*)m_frame->gradient[0]->data();
	float* piv1 = (float*)m_frame->gradient[1]->data();


	for (int i = 0; i < total; i++) {
	
		u = i % m_width;
		v = i / m_width;

		m[0] = u + put[i][0];
		m[1] = v + put[i][1];

		if (m[0] >= 0 && m[0] < m_width-1 && m[1] >= 0 & m[1] < m_height-1) {
		
			pm[i] = 255;
			u = (int)m[0];
			v = (int)m[1];
			m[0] -= u;
			m[1] -= v;

			pfi1 = pi1 + v * m_width + u;
			pfiu = piu1 + v * m_width + u;
			pfiv = piv1 + v * m_width + u;
			pwi1[i] = SAMPLE_2D(pfi1[0], pfi1[1], pfi1[m_width], pfi1[m_width+1], m[0], m[1]);
			pit[i] = pwi1[i] - pi0[i];
			pwiu1[i][0] = SAMPLE_2D(pfiu[0], pfiu[1], pfiu[m_width], pfiu[m_width+1], m[0], m[1]);
			pwiu1[i][1] = SAMPLE_2D(pfiv[0], pfiv[1], pfiv[m_width], pfiv[m_width+1], m[0], m[1]);
			w = piu0[i]*pwiu1[i][0]+piv0[i]*pwiu1[i][1];
			if (w < 0) { w = 0; }
			pw[i] = w;
			//pw[i] = exp(-p_dg[i]*p_dg[i]/0.16);
		}
		else { 
			pm[i] = 0;
			pit[i] = 0;
			pwi1[i] = 0;
			pwiu1[i] = 0;
			pw[i] = 0;
		}
	}


}

bool Slam::calc_du_of4() {

	std::cout << "Slam::calc_du_of4" << std::endl;

	if (!m_key || !m_frame) { return true; }
	int total = m_width * m_height;
	float* pwit = (float*)m_key->residual->data();
	Vec2f* pdut = (Vec2f*)m_key->dut->data();
	Vec2f* put = (Vec2f*)m_key->optical_flow->data();

	Vec2f* pwiu1 = (Vec2f*)m_key->warp_gradient->data();
	float* piu0 = (float*)m_key->gradient[0]->data();
	float* piv0 = (float*)m_key->gradient[1]->data();
	float* pw = (float*)m_key->of_weight->data();
	unsigned char* pm = (unsigned char*)m_key->mask->data();

	int u, v, u2, v2;
	float w, W, iu0[2], iu1[2];//, d;
	double A[4], iA[4], B[2], it, iuu[2], idet, iuiu[4];
	double lamda = Config::du_smooth_lamda_of3;
	double s = Config::stable_factor_of3;

	int offid[9] = { 
		-m_width-1, -m_width, -m_width+1,
		-1, 0, 1,
		m_width-1, m_width, m_width+1
	};
	int offx[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
	int offy[9] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

	double a, b[2], c;

	for (int i = 0; i < total; i++) {


		u = i % m_width;
		v = i / m_width;

		iu0[0] = piu0[i];
		iu0[1] = piv0[i];
		it = pwit[i];
		c = iu0[0]*iu0[0]+iu0[1]*iu0[1];
		a = c;
		b[0] = it*iu0[0];
		b[1] = it*iu0[1];

		if (Config::use_i1_constrain_of3) {
			iu1[0] = pwiu1[i][0];
			iu1[1] = pwiu1[i][1];

			a += iu1[0]*iu1[0]+iu1[1]*iu1[1];
			b[0] += it*iu1[0];
			b[1] += it*iu1[1];			
		}


		for (int k = 0; k < 9; k++) {
			u2 = u + offx[k];
			v2 = v + offy[k];
			if (u2 >= 0 && u2 < m_width && v2 >= 0 && v2 < m_height) {

					iuu[0] = put[i+offid[k]][0]-put[i][0];
					iuu[1] = put[i+offid[k]][1]-put[i][1];
					w = pw[i+offid[k]] + Config::stable_factor_of3;
					a += w*(c+0.5);
					b[0] -= w*(c+0.5)*iuu[0];
					b[1] -= w*(c+0.5)*iuu[1];
			}
		}

		pdut[i][0] = -b[0]/(a+0.01);
		pdut[i][1] = -b[1]/(a+0.01);

	}

	//return is ok?
	m_key->optical_flow->add(m_key->dut);
	return false;
}

bool Slam::calc_e_dr_of4() {


	std::cout << "Slam::calc_e_dr_of4" << std::endl;

	if (!m_key || !m_frame) { return true; }
	int total = m_width * m_height;
	Vec2f* put = (Vec2f*)m_key->optical_flow->data();
	float* pwo = (float*)m_key->of_weight->data();
	float* pwe = (float*)m_key->epi_weight->data();
	Vec2f* pdut = (Vec2f*)m_key->dut->data();

	 Intrinsic in = m_key->intrinsic;
	 double f1 = 1.0 / in.f;
	 double m0[3], m1[3], x0[3], x1[3], x10[3], du[2];
	 Vec9d iR = MatrixToolbox::inv_matrix_3x3(m_frame->rotation);
	 const double* R = iR.val;
	 const double* e = m_frame->epi_point.val;
	 double c, wo, we, ae[3], ar[3], ar0[3];
	 double Ae[9], Ar[9], Br[3];

	 memset(Ae, 0, sizeof(Ae));
	 memset(Ar, 0, sizeof(Ar));
	 memset(Br, 0, sizeof(Br));
	 memset(ae, 0, sizeof(ae));
 	 m0[2] = 1;
 	 m1[2] = 1;
	 x10[2] = 1;
	 x0[2] = 1;
	 x1[2] = 1;

	for (int i = 0; i < total; i++) {

		m0[0] = i % m_width - in.cx;
		m0[1] = i / m_width - in.cy;
		m1[0] = m0[0] + put[i][0];
		m1[1] = m0[1] + put[i][1];
		x0[0] = m0[0]*f1;
		x0[1] = m0[1]*f1;
		x10[0] = m1[0]*f1;
		x10[1] = m1[1]*f1;
		x1[0] = R[0]*x10[0]+R[1]*x10[1]+R[2]*x10[2];
		x1[1] = R[3]*x10[0]+R[4]*x10[1]+R[5]*x10[2];
		x1[2] = R[6]*x10[0]+R[7]*x10[1]+R[8]*x10[2];
		x1[0] /= x1[2];
		x1[1] /= x1[2];
		x1[2] = 1;

		du[0] = x1[0]*in.f-m0[0];
		du[1] = x1[1]*in.f-m0[1];
		pdut[i][0] = du[0];
		pdut[i][1] = du[1];

		we = du[0]*e[0]+du[1]*e[1]-
			(du[0]*x0[0]+du[1]*x0[1])*e[2];
		we *= f1;
		pwe[i] = we;
		wo = pwo[i];

		ae[0] += wo*du[0];
		ae[1] += wo*du[1];
		ae[2] -= wo*(du[0]*x0[0]+du[1]*x0[1]);

		ar0[0] = x1[1]*e[2]-x1[2]*e[1];
		ar0[1] = x1[2]*e[0]-x1[0]*e[2];
		ar0[2] = x1[0]*e[1]-x1[1]*e[0];

		ar[0] = x0[1]*ar0[2]-x0[2]*ar0[1];
		ar[1] = x0[2]*ar0[0]-x0[0]*ar0[2];
		ar[2] = x0[0]*ar0[1]-x0[1]*ar0[0];

		Ar[0] += wo*ar[0]*ar[0];
		Ar[1] += wo*ar[0]*ar[1];
		Ar[2] += wo*ar[0]*ar[2];
		Ar[4] += wo*ar[1]*ar[1];
		Ar[5] += wo*ar[1]*ar[2];
		Ar[8] += wo*ar[2]*ar[2];

		c = x0[0]*ar0[0]+x0[1]*ar0[1]+x0[2]*ar0[2];

		Br[0] -= wo*c*ar[0];
		Br[1] -= wo*c*ar[1];
		Br[2] -= wo*c*ar[2];
	}


	Ar[3] = Ar[1];
	Ar[6] = Ar[2];
	Ar[7] = Ar[5];

	Ar[0] += 0.1;
	Ar[4] += 0.1;
	Ar[8] += 0.1;


	if (m_frame->epi_point.length2() < 0.5
	 || !Config::only_calc_epi_dr) {
		double n = sqrt(ae[0]*ae[0]+ae[1]*ae[1]+ae[2]*ae[2]);
		m_frame->epi_point[0] = ae[0]/n;
		m_frame->epi_point[1] = ae[1]/n;
		m_frame->epi_point[2] = ae[2]/n;		
	}

		
	Vec9d iAr = MatrixToolbox::inv_matrix_3x3(Ar);
	double dr[3] = {
			iAr[0]*Br[0]+iAr[1]*Br[1]+iAr[2]*Br[2],
			iAr[3]*Br[0]+iAr[4]*Br[1]+iAr[5]*Br[2],
			iAr[6]*Br[0]+iAr[7]*Br[1]+iAr[8]*Br[2],

	};
	MatrixToolbox::update_rotation(m_frame->rotation, dr);
	m_frame->rotation_warp(m_warp);

	return true;

}

} // namespace

