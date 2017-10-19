

#include "Slam.h"

#include <iostream>
#include <memory.h>


namespace ww {


Slam::Slam(VideoSource* vs) : m_working(false), m_camera_count(0) {

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
	while(/*vs->read_frame() && */m_working) {

		push();
		break;
	}
	
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

void Slam::push() {

	std::cout << "Slam::push" << std::endl;
	preprocess();
	update_pose();
	update_keyframe();
	update_map();

	if (m_display_delegate) {
		m_display_delegate->display_with(this);
	}
}


void Slam::preprocess(){

	std::cout << "Slam::preprocess" << std::endl;
}
void Slam::update_pose(){

	std::cout << "Slam::update_pose" << std::endl;
}
void Slam::update_keyframe(){

	std::cout << "Slam::update_keyframe" << std::endl;

	if (m_current_keyframe == NULL) {
	
		m_current_keyframe = new Camera();
		memset(m_current_keyframe->pos, 0, sizeof(m_current_keyframe->pos));
		//m_current_keyframe->image = image;
		//m_current_keyframe->depth = default_depth; 
		
		m_keyframes.push_back(m_current_keyframe);
		m_cameras[m_camera_count] = m_current_keyframe;
	
	m_camera_count++;
	}
}
void Slam::update_map(){

	std::cout << "Slam::update_map" << std::endl;
}



} // namespace


/***************************



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

