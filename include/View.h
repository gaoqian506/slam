

#ifndef __WW_VIEW_HEADER__
#define __WW_VIEW_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"
#include <pthread.h>

namespace ww {



class View : public DisplayDelegate {

public:
	View(ViewContent* vc);
	~View();
	
	void run();
	void tick();
	void display();
	void start_content();
	void draw_cameras();
	void draw_points();
	void draw_mesh();
	
	virtual void display_with(ViewContent* cv);
	
private:
	ViewContent* m_content;
	pthread_t m_thread_id;


};


}

#endif
