
#ifndef __WW_CV_IMAGE_HEADER__
#define __WW_CV_IMAGE_HEADER__

#include "Image.h"


namespace ww {



class CvImage : public Image {

public:

	cv::Mat& cv_mat() { return m_cv_mat; }

private:

	cv::Mat m_cv_mat;

};


}

#endif
