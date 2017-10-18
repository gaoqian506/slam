

#include "VideoFile.h"
#include "Slam.h"
#include "View.h"
#include <GL/glut.h>


int main(int argc, char** argv) {

	glutInit(&argc, argv);

	ww::VideoSource* vs = new ww::VideoFile(argv[1]);
	ww::ViewContent* vc = new ww::Slam(vs);
	ww::View view(vc);
	view.run();
	
	delete vs;
	delete vc;
	

	return 0;

}
