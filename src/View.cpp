

#include "View.h"
#include <GL/glut.h>
#include <iostream>


namespace ww {

void* g_host = 0;

void g_display(void) {
	if (g_host) { ((View*)g_host)->display(); }
}

void* g_thread(void*) {

	std::cout << "g_thread" << std::endl;
	
	 ((View*)g_host)->start_content();
	
}


View::View(ViewContent* vc) {

	std::cout << "View::View" << std::endl;

	m_content = vc;
	vc->set_display_delegate(this);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(0, 0); 
	glutInitWindowSize(300, 300);
	glutCreateWindow("OpenGL 3D View");
	glutDisplayFunc(g_display);

}

View::~View() {

	std::cout << "View::~View" << std::endl;
	m_content->stop();
	pthread_join(m_thread_id, 0);

}

void View::run() {

	std::cout << "View::run" << std::endl;
	g_host = this;
	

	pthread_create(&m_thread_id, NULL, g_thread, NULL);
	glutMainLoop();

}

void View::display() {

	draw_cameras();
	draw_points();
	draw_mesh();

	glutSwapBuffers();
}

void View::draw_cameras() {
	std::cout << "View::draw_cameras" << std::endl;
}

void View::draw_points() {
	std::cout << "View::draw_points" << std::endl;
}

void View::draw_mesh() {
	std::cout << "View::draw_mesh" << std::endl;
}

void View::start_content() {

	std::cout << "View::start_content" << std::endl;
	m_content->start();

}

void View::display_with(ViewContent* cv) {

	std::cout << "View::display_with" << std::endl;
	glutPostRedisplay();

	
}


} // namespace

/*



	std::cout << "View::display" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT);
	//glBegin(GL_POINTS);
	//glEnd();
	glRectf(-0.5f,-0.5f,0.5f,0.5f);



void View::tick() {

	std::cout << "View::tick" << std::endl;

	//m_content->tick();
	display();



}



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


#include "controller.h"
#include <iostream>			// cout
#include <stdio.h>			// fopen
#include "opencv2/highgui/highgui.hpp"	//cv::imread
#include "config.h"
#include "toolbox.h"

huma::Controller* g_host;


namespace huma {

	void main_loop(void) {

		g_host->display();
	}

	void mouse_func(int button, int state, int x, int y) {
		g_host->mouse(button, state, x, y);

	}
	void mouse_move_func(int x,int y) {
		g_host->mouse_move(x, y);

	}
	void passive_mouse_move_func(int x, int y) {
		g_host->passive_mouse_move(x, y);
	}
	void keyboard_func(unsigned char key,int x,int y) {
		g_host->keyboard(key, x, y);
	}

	void Special_keyboard_func(int key, int x, int y) {
		g_host->Specialkeyboard(key, x, y);
	}

	void menu_func(int id) {
		g_host->menu(id);
	}

	void reshape_func(int w, int h) {
		g_host->reshape(w, h);
	}
//----------------------------------------------------------------

	Controller::Controller() {

		g_host = this;
		init_glut(0, 0);
		_cmd_cursor = 0;
		_cmd_buffer[0] = 0;
		_purpose = Normal;
		memset(_rectangle, 0, sizeof(_rectangle));
		memset(_image_trans, 0, sizeof(_image_trans));
		memset(_image_ids, 0, sizeof(_image_ids));
		_image_trans[0] = 1;
		_pixel_value[0] = 0;
		command("test");
		Config::instance()->min_pyramid_size();

	}

	void Controller::run() {
		glutMainLoop();
	}

	void Controller::display() {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		draw_image();
		//draw_rectangle();
		//draw_polygon();
		
		int x = 18, y = 15;
		print_text(_cmd_buffer, x, y);
		print_text(_pixel_value, x, y);


		glutSwapBuffers();
	}

	void Controller::menu(int id) {
		set_purpose((Purpose)id);
	}

	void Controller::keyboard(unsigned char key,int x,int y) {

		switch(key) {

		//Enter
		case 13:	
			command(_cmd_buffer);
			_cmd_cursor = 0;
			_cmd_buffer[_cmd_cursor] = 0;

		break;
		//Backspace
		case 8:
			_cmd_cursor = _cmd_cursor > 0 ? _cmd_cursor-1 : 0;
			_cmd_buffer[_cmd_cursor] = 0;
		break;
		//Zero
		case 48:
			_image_ids[0] = 0;
			_image_ids[1] = 0;
			select_image();
		break;
		//Escape...
		case 27:
			set_purpose(Normal);
		break;
		default:
			_cmd_buffer[_cmd_cursor++] = key;
			_cmd_buffer[_cmd_cursor] = 0;

		}

		display();
	}

	void Controller::Specialkeyboard(int key, int x, int y) {
		switch(key) {
		case GLUT_KEY_LEFT:
			_image_ids[0]--;
			if (_image_ids[0] < 0) { _image_ids[0] = 0; }
			_image_ids[1] = 0;
		break;
		case GLUT_KEY_RIGHT:

			//if (check_layer(_image_ids[0]+1, _image_ids[1])) {
				_image_ids[0]++;
				_image_ids[1] = 0;
			//}else {
			//	std::cout<< "No this layer!" << "\n";
			//}
		break;
		case GLUT_KEY_UP:
			//if (check_layer(_image_ids[0], _image_ids[1]+1)) {
				_image_ids[1]++;
			//}else {
			//	std::cout<< "No this layer!" << "\n";
			//}
		break;
		case GLUT_KEY_DOWN:
			_image_ids[1]--;
			if (_image_ids[1] < 0) { _image_ids[1] = 0; }
		break;
		}
		select_image();
	}

	void Controller::mouse(int button, int state, int x, int y) {

		if (button == 3 || button == 4) {
			double ds = button == 3 ? 1.25 : 0.8;
			double dx = x - _win_size[0]*0.5;
			double dy = _win_size[1]*0.5 - y;
			_image_trans[0] *= ds;
			_image_trans[1] = ds*(_image_trans[1]-dx)+dx;
			_image_trans[2] = ds*(_image_trans[2]-dy)+dy;
		}
		
		switch (_purpose) {

		case PreRectangle:
			if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
				cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
				_rectangle[0] = p[0]; _rectangle[1] = p[1];
				_rectangle[2] = _rectangle[0]; _rectangle[3] = _rectangle[1]; 
				set_purpose(DoingRectangle);
			}
		break;
		case PrePolygon:
			if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
				if (_polygon.size() > 3) {
					cv::Vec2d p = image_to_screen(_polygon.front());
					if (cv::norm(p - cv::Vec2d(x, y)) < 10) {
						_polygon.back() = _polygon.front();
						set_purpose(Normal);
						break;
					}
				}
				cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
				_polygon.push_back(p);
			}
		break;
		case DoingRectangle:
			if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
				set_purpose(Normal);
			}
		break;
		case Normal:

		break;
			
		}
		display();

	}

	void Controller::mouse_move(int x,int y) {

		if (!_image.empty()) {

			cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
			p[0] = (_image.cols>>1)+p[0];
			p[1] = (_image.rows>>1)-p[1];

                        if (p[1] >=0 && p[1] < _image.rows && p[0] >= 0 && p[0] < _image.cols) {

                                if (_image.type() == CV_32F && _image.channels() == 1) {
                                        sprintf(_pixel_value, "%.2f,%.2f:%f", p[0], p[1] , _image.at<float>((int)p[1], (int)p[0]));
                                }
                                else if (_image.type() == CV_8U && _image.channels() == 1) {
                                        sprintf(_pixel_value, "%.2f,%.2f:%d", p[0], p[1] , _image.at<unsigned char>((int)p[1], (int)p[0]));
                                }
                                else if (_image.type() == CV_8S && _image.channels() == 1) {
                                        sprintf(_pixel_value, "%.2f,%.2f:%d", p[0], p[1] , _image.at<char>((int)p[1], (int)p[0]));
                                }
			}
		}

		switch (_purpose) {

		case DoingRectangle:
			cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
			_rectangle[2] = p[0]; _rectangle[3] = p[1];
		break;
			
		}
		display();
		
	}

	void Controller::passive_mouse_move(int x,int y) {

		switch (_purpose) {
		case PrePolygon:
			_polygon.back() = screen_to_image(cv::Vec2d(x, y));
			display();
		break;
			
			
		}

	}

	void Controller::reshape(int w, int h) {
		_win_size[0] = w;
		_win_size[1] = h;
		glViewport(0, 0, w, h);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-w*0.5, w*0.5, -h*0.5, h*0.5, -1, 1);
	}


//-------------------------------------------------------------------------------
	void Controller::init_glut(int argc, char** argv) {

		_gl_texture = 0;
		glutInit(&argc, argv);


		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
		glutInitWindowSize(800, 600);
		glutCreateWindow("mahu");
		glutDisplayFunc(main_loop); //glutIdleFunc glutDisplayFunc
		glutKeyboardFunc(keyboard_func);
		glutSpecialFunc(Special_keyboard_func);
		glutMouseFunc(mouse_func);
		glutMotionFunc(mouse_move_func);
		glutPassiveMotionFunc(passive_mouse_move_func);
		glutReshapeFunc(reshape_func);
		//glutSetIconTitle("favicon.ico");
		//glutSetWindowTitle("favicon.ico");
		glClearColor(0.3, 0.3, 0.3, 1);

		//assert(glewInit() == GLEW_OK);

		// menus
		 int menu;
		 menu = glutCreateMenu(menu_func);
		 glutAddMenuEntry("Polygon", PrePolygon);
		 glutAddMenuEntry("Rectangle", PreRectangle);
		 glutAddMenuEntry("ResetImage", ResetImage);
		 glutAttachMenu(GLUT_RIGHT_BUTTON);

		//std::cout << "controller's glut initialized.\n";

	}

	void Controller::setup_gl_texture( cv::Mat image ) {
		if (image.empty()) { return; }
		image.copyTo(_image);


		if (_gl_texture == 0) { glGenTextures(1, &_gl_texture);	}

		glBindTexture(GL_TEXTURE_2D, _gl_texture);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

		double min, max, scale;
		cv::minMaxIdx(image, &min, &max);
		scale = 1./(max-min);
		_image.convertTo(image, 
			CV_32FC(image.channels()), scale, -min/(max-min));				

		unsigned int internalFormat = 0, format = 0, type = 0;

		if (image.type() == CV_32F)
		{ internalFormat = GL_LUMINANCE; format = GL_LUMINANCE; type = GL_FLOAT; }
		else if (image.type() == CV_32FC3)
		{ internalFormat = GL_RGB; format = GL_BGR; type = GL_FLOAT; }
		else if (image.type() == CV_32FC4)
		{ internalFormat = GL_RGBA; format = GL_RGBA; type = GL_FLOAT; }
		else if (image.type() == CV_8UC3)
		{ internalFormat = GL_RGB; format = GL_RGB; type = GL_UNSIGNED_BYTE; }
		else { assert(0); }

		int w = image.cols;
		int h = image.rows;
		_image_size[0] = w;
		_image_size[1] = h;

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, type, image.data);

		glBindTexture(GL_TEXTURE_2D, 0);
	}



	
	void Controller::draw_image() {

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		
		if(_gl_texture){

			glTranslated(_image_trans[1], _image_trans[2], 0);
			glScaled(_image_trans[0], _image_trans[0], 1); 

			glBindTexture(GL_TEXTURE_2D, _gl_texture);
			glEnable(GL_TEXTURE_2D);

			glBegin(GL_QUADS);

			glTexCoord2f(0.0f, 1.0f); glVertex2d(-0.5*_image_size[0], -0.5*_image_size[1]);
			glTexCoord2f(1.0f, 1.0f); glVertex2d(+0.5*_image_size[0], -0.5*_image_size[1]);
			glTexCoord2f(1.0f, 0.0f); glVertex2d(+0.5*_image_size[0], +0.5*_image_size[1]);
			glTexCoord2f(0.0f, 0.0f); glVertex2d(-0.5*_image_size[0], +0.5*_image_size[1]);


			glEnd();

			glDisable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, 0);

		}

		glColor3d(1,0,0);
		glBegin(GL_LINE_LOOP);
		glVertex2d(_rectangle[0], _rectangle[1]);
		glVertex2d(_rectangle[2], _rectangle[1]);
		glVertex2d(_rectangle[2], _rectangle[3]);
		glVertex2d(_rectangle[0], _rectangle[3]);
		glEnd();

		glColor3d(1,0,0);
		glBegin(GL_LINE_STRIP);
		for (std::vector<cv::Vec2d>::iterator itr = _polygon.begin(); 
			itr != _polygon.end(); itr++) {
			glVertex2dv(itr->val);
			

		}
		glEnd();

		glPopMatrix();

	}

	void Controller::print_text(const char* text, int& x, int& y) {

		glColor3d(0,1,0);
		glRasterPos2d(x-_win_size[0]*0.5,_win_size[1]*0.5-y);
		for (int i = 0; i < strlen(text); i++) {
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, text[i]);
		}

		y += 10;

	}

	void Controller::command(const char* cmd) {

		std::cout << "\ncommand:" << cmd << "\n";

		if (int k = Toolbox::leftcmp("open ", cmd)) {
			open_image(cmd+k);
			strcpy(_image_name, cmd+k);
			//select_image();
		}
		else if (strcmp("test", cmd) == 0) {

			if(FILE *fp = fopen("command/commands.cmd", "r")) {
				char buffer[1024];
				while(fgets(buffer, 1024, fp)) {
					if (Toolbox::leftcmp("#", buffer)) { continue; }
					buffer[strlen(buffer)-1] = 0;
					//std::cout << "test command:" << buffer << "\n";
					command(buffer);

				}
				fclose(fp);
			}
			
		}
		else if (strcmp("exit", cmd) == 0 || strcmp("q", cmd) == 0) {
			exit(0);
		}
		else if (int k = Toolbox::leftcmp("go ", cmd)) {//"object "
			char cmd2[256] = "object 1";
			//sprintf(cmd2, "object %s", cmd+k);
			int n = atoi(cmd+k);
			std::cout << "will train " << n << "times!\n";
			for (int i = 0; i < n; i++) {
				std::cout << "step:------------------" << i << "----------------\n";
				command(cmd2);
				command("test");


			}
		}
		else if (int k = Toolbox::leftcmp("object ", cmd)) {//"object "
			std::vector<cv::Vec2d> poly = center_to_corner(_polygon);
			_detector.set(cmd+k, poly);
		}
		else {
			std::cout << "invalid command:" << cmd << "!\n";
		}

		select_image();

	}

	void Controller::open_image(const char* name) {

		cv::Mat img = cv::imread(name,CV_LOAD_IMAGE_COLOR);
		if(!img.empty()) {
			_detector.set_image(img);
			select_image();
		}else {
			std::cout<< "open image failed! image name:" << name << ".\n";
		}
	}

	void Controller::set_purpose(Purpose p) {

		switch (p) {
		case Normal:
			glutSetCursor(GLUT_CURSOR_RIGHT_ARROW);
		break;
		case PreRectangle:
			memset(_rectangle, 0, sizeof(_rectangle));
			glutSetCursor(GLUT_CURSOR_CROSSHAIR);
		break;
		case PrePolygon:
			_polygon.clear();
			_polygon.push_back(cv::Vec2d(0, 0));
			glutSetCursor(GLUT_CURSOR_CROSSHAIR);
		break;
		case ResetImage:
			memset(_image_trans, 0, sizeof(_image_trans));
			_image_trans[0] = 1;
			p = Normal;
		break;


		}
		//std::cout << "purpose:" << p << ".\n";
		_purpose = p;
		display();
	}

	cv::Vec2d Controller::screen_to_image(cv::Vec2d p) {

		double a = p[0] - _win_size[0]*0.5;
		double b = _win_size[1]*0.5 - p[1];

		return cv::Vec2d(
				(a-_image_trans[1])/_image_trans[0],
				(b-_image_trans[2])/_image_trans[0]
			);

	}

	cv::Vec2d Controller::image_to_screen(cv::Vec2d p) {

		double a = _image_trans[0] * p[0] + _image_trans[1];
		double b = _image_trans[0] * p[1] + _image_trans[2];
		return cv::Vec2d(a+_win_size[0]*0.5, _win_size[1]*0.5-b);
	}

	std::vector<cv::Vec2d> Controller::center_to_corner(std::vector<cv::Vec2d> ps) {


		std::vector<cv::Vec2d> out;
		for (std::vector<cv::Vec2d>::iterator itr = ps.begin();
			itr != ps.end(); itr++) {
			out.push_back(
				cv::Vec2d(
					(_image_size[0]>>1)+itr->val[0],
					(_image_size[1]>>1)-itr->val[1]
				)
			);
		}
		return out;
	}

	void Controller::select_image() {
		
		std::cout << "debug_image(" << _image_ids[0] <<"," << _image_ids[1] << "):";

		Layer* layer = _detector.get_layer(_image_ids[0]);
		if (layer) {
			std::cout << layer->name();
			cv::Mat image = layer->get_image(_image_ids[1]);
			if (!image.empty()) {
				setup_gl_texture(image);
			}
		}

		std::cout << "\n";

		display();

	}

	bool Controller::check_layer(int idx1, int idx2) {

		Layer* layer = _detector.get_layer(idx1);
		if (layer) {

			cv::Mat image = layer->get_image(idx2);
			if (!image.empty()) {
				return true;
			}
		}

		return false;

	}
}

*/




