

#include "GlToolbox.h"
#include <iostream>
#include <assert.h>

namespace ww {


void GlToolbox::othorgonal(const Rectangle& rect) {

	std::cout << "GlToolbox::othorgonal" << std::endl;

	double rect_aspect = rect.width / rect.height;
	
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	double view_aspect = viewport[2] / viewport[3];
	//Rectangle viewport = gl_viewport_rect;
	
	//double view_aspect = viewport.width / viewport.height;
	
	double center[2] = { rect.left+rect.width*0.5, rect.bottom + rect.height*0.5 };
	if (rect_aspect > view_aspect) {
	
		double v_radius = rect.width * view_aspect * 0.5;
		double bottom = center[1] - v_radius;
		double top = center[1] + v_radius;

		glOrtho(rect.left, rect.left+rect.width, bottom, top, -1000, 1000);
	}
	else {
		// in vertical case
		double h_radius = rect.height / view_aspect * 0.5;
		double left = center[0] - h_radius;
		double right = center[0] + h_radius;
		glOrtho(left, right, rect.bottom, rect.bottom+rect.height, -1000, 1000);
	}

}


void GlToolbox::transform_to(const Vec3d& pose, const Vec9d& rotation) {


	double M[16] = {
		rotation[0], rotation[1], rotation[2], pose[0],
		rotation[3], rotation[4], rotation[5], pose[1],
		rotation[6], rotation[7], rotation[8], pose[2]
	};
	glMatrixMode(GL_MODELVIEW);
	glMultTransposeMatrixd(M);

}

void GlToolbox::transform_to(const double* pose, const double* rotation) {

	assert(0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


}

/********************************



	assert(0);

	glTranslated(pose[0], pose[1], pose[2]);

		glOthorgonal(rect.left, rect.left+rect.width, bottom, top);
		
		
*********************************/

