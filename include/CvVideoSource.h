
#ifndef __WW_CV_VIDEO_SOURCE_HEADER__
#define __WW_CV_VIDEO_SOURCE_HEADER__


#include "VideoSource.h"
#include "opencv2/highgui/highgui.hpp"


namespace ww {



class CvVideoSource : public VideoSource {

public:
	CvVideoSource(const char* path);

	virtual bool read(Image* image);

private:
	cv::VideoCapture m_capture;
};


}

#endif
