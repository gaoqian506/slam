
#ifndef __WW_VIEW_CONTENT_HEADER__
#define __WW_VIEW_CONTENT_HEADER__

#include <iostream>

namespace ww {

class ViewContent;
class DisplayDelegate;


class DisplayDelegate {

public:
	virtual void display_with(ViewContent* cv) {
		std::cout << "DisplayDelegate::display_with" << std::endl;
	}
};



class ViewContent {

public:
	ViewContent() : m_display_delegate(0) {
		std::cout << "ViewContent::ViewContent" << std::endl;
	}
	virtual void tick() = 0;
	virtual void start() = 0;
	virtual void stop() = 0;
	
	void set_display_delegate(DisplayDelegate* dd) {
		std::cout << "ViewContent::set_display_delegate" << std::endl;
		m_display_delegate = dd;
	}
	
protected:
	DisplayDelegate* m_display_delegate;
	

};


}

#endif
