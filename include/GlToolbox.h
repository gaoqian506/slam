#ifndef __WW_GL_TOOLBOX_HEADER__
#define __WW_GL_TOOLBOX_HEADER__

#include "Rectangle.h"
#include "Vectors.h"
#include "Image.h"
#include <GL/gl.h>


namespace ww {



class GlToolbox {

public:


enum Origin { Center, TopLeft };

static void othorgonal(const Rectangle& rect);
static void orthogonal_pixel(Origin ori = Center);
static void transform_to(const Vec3d& pose, const Vec9d& rotation, bool inverse = false);

static void transform_to(const double* pose, const double* rotation);



static void setup_texture(GLuint m_gl_texture, Image* image);
static Vec2d screen_to_image(const double& x, const double& y, const double& scale, const int& width, const int& height);

};


}

#endif
