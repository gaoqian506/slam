
#include "CvImage.h"

//#include "opencv2/improc/improc.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" 

namespace ww {

void* CvImage::data() {

	return m_cv_mat.data;

}
int CvImage::width() {

	return m_cv_mat.cols;

}
int CvImage::height() {
	return m_cv_mat.rows;
}

void CvImage::gray(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::cvtColor(m_cv_mat, cv_out->cv_mat(), CV_BGR2GRAY); 

}
void CvImage::sobel_x(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::Sobel(m_cv_mat, cv_out->cv_mat(), m_cv_mat.depth(), 1, 0);
}
void CvImage::sobel_y(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::Sobel(m_cv_mat, cv_out->cv_mat(), m_cv_mat.depth(), 0, 1);

}
void CvImage::subtract(Image* b, Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	CvImage* cv_b = static_cast<CvImage*>(b);
	cv::subtract(m_cv_mat, cv_b->cv_mat(), cv_out->cv_mat());
	
}
void CvImage::copy_to(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	m_cv_mat.copyTo(cv_out->cv_mat());

}
void CvImage::convert_to(Image*& out, DataType type) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	
	int t = CV_32F;
	switch (type) {
	
	case Float32:
		t = CV_32F;
		break;
	}
	m_cv_mat.convertTo(cv_out->cv_mat(), t);
}

void CvImage::set(double v) {

	m_cv_mat.setTo(v);
}


} // namespace
