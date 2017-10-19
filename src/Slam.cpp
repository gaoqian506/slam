

#include "Slam.h"

#include <iostream>


namespace ww {


Slam::Slam(const VideoSource* vs) : m_working(false) {

	std::cout << "Slam::Slam" << std::endl;
}


void Slam::tick() {

	std::cout << "Slam::tick" << std::endl;


}

void Slam::start() {


	std::cout << "Slam::start" << std::endl;
	m_working = true;

	for(int i = 0; i < 10; i++) {
		push();
	}
	while(m_working) {
		push();
		break;
	}
	
	std::cout << "leave Slam::start" << std::endl;

}

void Slam::stop() {


	std::cout << "Slam::stop" << std::endl;
	m_working = false;

}

void Slam::push() {

	std::cout << "Slam::push" << std::endl;

	if (m_display_delegate) {
		m_display_delegate->display_with(this);
	}
}



} // namespace


/*



#include <stdio.h>
#include <stdio.h>

*/

