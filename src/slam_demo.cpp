

#include "CvVideoSource.h"
#include "Slam.h"
#include "View.h"
#include <GL/glut.h>




int main(int argc, char** argv) {

	glutInit(&argc, argv);
	
	if (argc == 1) {
		std::cout << "please set video name." << std::endl;
		return 0;
	}

	ww::VideoSource* vs = new ww::CvVideoSource(argv[1]);
	ww::ViewContent* vc = new ww::Slam(vs);
	ww::View view(vc);
	view.run();
	
	delete vs;
	delete vc;
	

	return 0;

}
