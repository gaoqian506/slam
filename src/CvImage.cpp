
#include "CvImage.h"
#include "Config.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" 

namespace ww {

	CvImage::CvImage() {
		
	}
	
	CvImage::CvImage(const int& width, const int& height, const DataType& type, const int& channels) {
		switch(type) {
		case Float32:
			m_cv_mat = cv::Mat(height, width, CV_32FC(channels));
			break;
		case UByte:
			m_cv_mat = cv::Mat(height, width, CV_8UC(channels));
			break;
		
		}
	}

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
	cv::Mat gray;
	cv::cvtColor(m_cv_mat, gray/*cv_out->cv_mat()*/, CV_BGR2GRAY); 
	
	gray.convertTo(cv_out->cv_mat(), CV_32F, 1.0/255.0);

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
	//int t = cv_out->cv_mat().type();
	//cv_b = 0;
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

void CvImage::save(const char* path) {

	double minv = 0.0, maxv = 0.0;  
	double* minp = &minv;  
	double* maxp = &maxv;  
	minMaxIdx(m_cv_mat, &minv, &maxv); 
	cv::Mat stand = (m_cv_mat-minv) / (maxv-minv) * 255;
	cv::imwrite(path, stand);
}

void CvImage::resize(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	
	int w = m_cv_mat.cols;
	int h = m_cv_mat.rows;
	
	if (w > Config::max_width) {
	
		int th = (int)(((double)Config::max_width)/w*h);
	
	cv::resize(m_cv_mat, cv_out->cv_mat(), cv::Size(Config::max_width, th));
	}
	else {
		copy_to(out);
	}
}

double CvImage::average2() {

	return 0; 
}

Image::DataType CvImage::type() {

	return (DataType)(m_cv_mat.type() % 8);

}
int CvImage::channels() {
	return m_cv_mat.channels();
}

bool CvImage::empty() {

	return m_cv_mat.empty();

}

//float CvImage::sample(const float& a, const float& b) {

	//assert(m_cv_mat.type() == CV_32F);
	
	//return 0; 
 
 //}


} // namespace
