#include "Config.h"

namespace ww {


	int Config::max_width = 200;
	bool Config::manually_content = true;
	//bool Config::epipolar_mode = true;
	//Config::Method Config::method = Config::Entropy5;
	Config::Method Config::method = Config::Lsd3;
}
