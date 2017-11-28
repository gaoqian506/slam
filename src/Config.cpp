#include "Config.h"

namespace ww {


	int Config::max_width = 80;
	bool Config::manually_content = true;
	//bool Config::epipolar_mode = true;
	//Config::Method Config::method = Config::Entropy5;
	//Config::Method Config::method = Config::Lsd3;
	//Config::Method Config::method = Config::Epi2;
	Config::Method Config::method = Config::Of3;
	//Config::Method Config::method = Config::Gof1;
	int Config::of_skip = 1;
	int Config::field_skip = 1;
	double Config::sigma2_dgdu = 1.0/(0.2*0.2);
	double Config::sigma2_dgdt = 1.0/(0.5*0.5);
	double Config::du_smooth_lamda_of3 = 9;
	double Config::stable_factor_of3 = 0.1;
	double Config::min_weight_of3 = 0.1;
	bool Config::use_i1_constrain_of3 = true;
}
