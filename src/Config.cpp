#include "Config.h"

namespace ww {


	int Config::max_width = 160;
	bool Config::manually_content = true;
	//bool Config::epipolar_mode = true;
	Config::Method Config::method = Config::Entropy2;
	//Config::Method Config::method = Config::Entropy;
	//Config::Method Config::method = Config::Lsd;
}
