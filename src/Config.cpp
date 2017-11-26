#include "Config.h"

namespace ww {


	int Config::max_width = 200;
	bool Config::manually_content = true;
	//bool Config::epipolar_mode = true;
	//Config::Method Config::method = Config::Entropy5;
	//Config::Method Config::method = Config::Lsd3;
	//Config::Method Config::method = Config::Epi2;
	Config::Method Config::method = Config::Of1;
	int Config::of_skip = 1;
}
