

#include "GlToolbox.h"
#include <iostream>
#include <assert.h>

namespace ww {


void GlToolbox::othorgonal(const Rectangle& rect) {

	std::cout << "GlToolbox::othorgonal" << std::endl;

	double rect_aspect = rect.width / rect.height;
	
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	double view_aspect = (double)viewport[2] / viewport[3];
	//Rectangle viewport = gl_viewport_rect;
	
	//double view_aspect = viewport.width / viewport.height;
	
	double center[2] = { rect.left+rect.width*0.5, rect.bottom + rect.height*0.5 };
	if (rect_aspect > view_aspect) {
	
		double v_radius = rect.width / view_aspect * 0.5;
		double bottom = center[1] - v_radius;
		double top = center[1] + v_radius;

		glOrtho(rect.left, rect.left+rect.width, bottom, top, -1000, 1000);
	}
	else {
		// in vertical case
		double h_radius = rect.height * view_aspect * 0.5;
		double left = center[0] - h_radius;
		double right = center[0] + h_radius;
		glOrtho(left, right, rect.bottom, rect.bottom+rect.height, -1000, 1000);
	}

}

void GlToolbox::orthogonal_pixel() {
	int vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);

	glOrtho(-vp[2]>>1, vp[2]>>1, vp[3]>>1, -vp[3]>>1, -1, 1);
	//glOrtho(-300, 300, -300, 300, -1, 1);
}


void GlToolbox::transform_to(const Vec3d& pose, const Vec9d& rotation) {


	double M[16] = {
		rotation[0], rotation[1], rotation[2], pose[0],
		rotation[3], rotation[4], rotation[5], pose[1],
		rotation[6], rotation[7], rotation[8], pose[2],
		0, 0, 0, 1
	};
	glMatrixMode(GL_MODELVIEW);
	glMultTransposeMatrixd(M);

}

void GlToolbox::transform_to(const double* pose, const double* rotation) {

	assert(0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void GlToolbox::setup_texture(GLuint m_gl_texture, Image* image) {

	//glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glBindTexture(GL_TEXTURE_2D, m_gl_texture);

	if (image && !image->empty()) {

		unsigned int internalFormat = 0, format = 0, type = 0;
		if (image->type() == Image::UByte && image->channels() == 4)
		{ internalFormat = GL_RGBA; format = GL_RGBA; type = GL_UNSIGNED_BYTE; }
		else if (image->type() == Image::UByte && image->channels() == 1) 
		{ internalFormat = GL_LUMINANCE;  format = GL_LUMINANCE; type = GL_UNSIGNED_BYTE; }
		else if (image->type() == Image::UByte && image->channels() == 3) 
		{ internalFormat = GL_RGB; format = GL_RGB; type = GL_UNSIGNED_BYTE; }
		else if (image->type() == Image::Short && image->channels() == 1)
		{ internalFormat = GL_LUMINANCE; format = GL_LUMINANCE; type = GL_UNSIGNED_SHORT; }
		else if (image->type() == Image::Float32 && image->channels() == 1)
		{ internalFormat = GL_LUMINANCE; format = GL_LUMINANCE; type = GL_FLOAT; }
		else if (image->type() == Image::Float32 && image->channels() == 1)
		{ internalFormat = GL_RGB; format = GL_RGB; type = GL_FLOAT; }
		else { assert(0); }


		int w = image->width();
		int h = image->height();

		//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		//glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		//glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
		//glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, type, image->data());
	}
	else {
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 0, 0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, 0);
	}

	glBindTexture(GL_TEXTURE_2D, 0);
}

} // namespace



/********************************



	assert(0);

	glTranslated(pose[0], pose[1], pose[2]);

		glOthorgonal(rect.left, rect.left+rect.width, bottom, top);
		
		
*********************************/

