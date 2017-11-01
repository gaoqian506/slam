#ifndef __WW_GL_TOOLBOX_HEADER__
#define __WW_GL_TOOLBOX_HEADER__

#include "Rectangle.h"
#include "Vectors.h"
#include "Image.h"
#include <GL/gl.h>


namespace ww {



class GlToolbox {

public:

static void othorgonal(const Rectangle& rect);
static void orthogonal_pixel();
static void transform_to(const Vec3d& pose, const Vec9d& rotation);

static void transform_to(const double* pose, const double* rotation);
static void setup_texture(GLuint m_gl_texture, Image* image);

};


}

#endif
