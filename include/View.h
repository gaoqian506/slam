

#ifndef __WW_VIEW_HEADER__
#define __WW_VIEW_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"
#include "Camera.h"
#include "Rectangle.h"
#include <pthread.h>

namespace ww {



class View : public DisplayDelegate {

public:

	enum DisplayAspect {
		DisplaySpace = 0,
		DisplayImage, // 1
		// ...
	};

	View(ViewContent* vc);
	~View();
	
	void run();
	void tick();
	void display();
	void keyboard(unsigned char key,int x,int y);
	void start_content();
	void draw_cameras();
	void draw_points();
	void draw_mesh();
	void draw_camera_instance(Camera* camera);
	ViewContent* content() { return m_content; }
	
	virtual void display_with(ViewContent* cv);
	
private:

	Rectangle get_scene_bounding_box(Camera** cameras, int count);
	

	ViewContent* m_content;
	pthread_t m_thread_id;
	DisplayAspect m_display_aspect;
	


};


}

#endif
