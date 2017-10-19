
#ifndef __WW_SLAM_HEADER__
#define __WW_SLAM_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"

namespace ww {



class Slam : public ViewContent {

public:
	Slam(const VideoSource* vs);
	
	virtual void tick();
	virtual void start();
	virtual void stop();
	
	void push();

private:
	bool m_working;

};


}

#endif
