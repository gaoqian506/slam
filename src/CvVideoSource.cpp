

#include "CvVideoSource.h"
#include "CvImage.h"

namespace ww {


CvVideoSource::CvVideoSource(const char* path) {

	m_capture.open(path);
	assert(m_capture.isOpened());
	
	cv::namedWindow("video", CV_WINDOW_NORMAL);

}


bool CvVideoSource::read(Image*& image) {

	if (image == NULL) { 
		image = new CvImage(); 
	}

	CvImage* cv_image = static_cast<CvImage*>(image);
	
	assert(cv_image);
	
	if (m_capture.read(cv_image->cv_mat())) {
		cv::imshow("video", cv_image->cv_mat());  
		cv::waitKey(20);
		return true;	
	}
	else {
		return false;
	}


}


} // namespace

