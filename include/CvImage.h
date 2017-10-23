
#ifndef __WW_CV_IMAGE_HEADER__
#define __WW_CV_IMAGE_HEADER__

#include "Image.h"
#include "opencv2/core/core.hpp"


namespace ww {



class CvImage : public Image {

public:

	CvImage();
	CvImage(const int& width, const int& height, const DataType& type, const int& channels = 1);

	cv::Mat& cv_mat() { return m_cv_mat; }
	const cv::Mat& cv_Mat() const { return m_cv_mat; }
	
	virtual void* data();
	virtual int width();
	virtual int height();
	
	virtual void gray(Image*& out);
	virtual void sobel_x(Image*& out);
	virtual void sobel_y(Image*& out);
	virtual void subtract(Image* b, Image*& out);
	virtual void copy_to(Image*& out);
	virtual void convert_to(Image*& out, DataType type);
	
	virtual void set(double v);
	virtual float sample(const float& a, const float& b) { return 0; }

private:

	cv::Mat m_cv_mat;

};


}

#endif
