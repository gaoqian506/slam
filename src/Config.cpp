#include "Config.h"

namespace ww {


	int Config::max_width = 320;
	bool Config::manually_content = true;
	//bool Config::epipolar_mode = true;
	//Config::Method Config::method = Config::Entropy5;
	//Config::Method Config::method = Config::Lsd3;
	//Config::Method Config::method = Config::Epi2;
	Config::Method Config::method = Config::Of6;
	//Config::Method Config::method = Config::Gof1;
	int Config::of_skip = 5;
	int Config::field_skip = 1;
	double Config::sigma2_dgdu = 1.0/(0.2*0.2);
	double Config::sigma2_dgdt = 1.0/(0.5*0.5);
	double Config::du_smooth_lamda_of3 = 1;
	double Config::stable_factor_of3 = 0.01;
	double Config::min_weight_of3 = 0.1;
	bool Config::use_i1_constrain_of3 = true;
	double Config::epi_sigma2_of3 = 10000;
	int Config::max_iterate_times = 100;
	int Config::build_steps = 10;
	bool Config::only_calc_epi_dr = false;
	bool Config::use_of_smooth_of4 = true;
	double Config::min_depth_weight = 0.001;

	bool Config::use_i1_constrain_lsd5 = true;
	double Config::min_smooth_weight_lsd5 = 0.1;
	double Config::smooth_lamda_lsd5 = 1;
	int Config::max_iterate_times_lsd5 = 10;

	double Config::du_smooth_weight_of5 = 0.01;
	double Config::du_smooth_lamda_of5 = 1;
	int Config::max_iterations_of5 = 20;

	bool Config::image_switch = true;
	int Config::win_size[2] = { 500, 500 };
	//bool Config::use_canonical_intrinsic = true;
}
