

#include "View.h"
#include <GL/glut.h>

namespace ww {


void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);
    //glBegin(GL_POINTS);
    //glEnd();
    glRectf(-0.5f,-0.5f,0.5f,0.5f);
    glFlush();

    return;
}


View::View(const ViewContent* vc) {

	//glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
	glutInitWindowPosition(0, 0); 
	glutInitWindowSize(300, 300);
	glutCreateWindow("OpenGL 3D View");
	glutDisplayFunc(display);

}

void View::run() {
	glutMainLoop();

}



} // namespace

/*






int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowPosition(0, 0); 
    glutInitWindowSize(300, 300);
    glutCreateWindow("OpenGL 3D View");
    glutDisplayFunc(display);
    glutMainLoop();

    return 0;
}

*/





