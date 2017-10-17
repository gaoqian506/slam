

#ifndef __VIEW_HEADER__
#define __VIEW_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"

namespace ww {



class View {

public:
	View(const ViewContent* vc);
	
	void run();


};


}

#endif
