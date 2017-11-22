
#ifndef __WW_SLAM_HEADER__
#define __WW_SLAM_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"
#include "Vectors.h"
#include <vector>

#define MAX_STATIC_CAMERA_COUNT 100
#define MAX_PIXEL_INFO 1024

namespace ww {

class Slam : public ViewContent {

public:
	Slam(VideoSource* vs);

	virtual void start();
	virtual void stop();
	virtual Camera** get_cameras();
	virtual int get_camera_count();
	virtual Camera* get_current_frame();
	virtual bool changed();
	virtual Image* get_debug_image(const int& idx);
	virtual void push_manauly();
	virtual void func_manualy(int idx);
	virtual char* pixel_info(const Vec2d& u);


private:

	void initialize(Image* image);
	void push(Image* image);
	void push_epi(Image* image);
	void preprocess(Image* image);
	void make_epi_line(Camera* camera);
	void update_pose();
	void calc_epipolar();
	void calc_epi_trans();
	void update_keyframe(Image* image);
	void update_map();

	void prepare_residual();
	void prepare_residual_epi();
	Vec3d calc_delta_t();
	Vec3d calc_delta_r();
	Vec3d calc_epi_point();
	Vec3d calc_epi_dr();
	Vec3d calc_dr_entropy();
	Vec3d calc_t_entropy();
	void wipe_depth(const Vec3d& t);
	void create_keyframe(Image* image);
	void update_depth();


	VideoSource* m_source;
	bool m_working;
	Camera* m_cameras[MAX_STATIC_CAMERA_COUNT];
	int m_camera_count;
	std::vector<Camera*> m_keyframes;
	Camera* m_key;
	Camera* m_frame;
	Image* m_mask;
	//Image* m_points;
	Image* m_residual;
	Image* m_gradient;
	Image* m_depth;
	Image* m_iuux;
	Image* m_debug_image;
	Image* m_epi_line;
	Image* m_warp;
	bool m_changed;
	char m_pixel_info[MAX_PIXEL_INFO];
	int m_width;
	int m_height;
	std::string m_image_name;

};


} // namespace

#endif


/*


	
	virtual void tick();
	
	
*/
