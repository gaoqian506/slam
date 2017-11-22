#ifndef __WW_CONFIG_HEADER__
#define __WW_CONFIG_HEADER__

namespace ww {



class Config {

public:

	enum Method { Lsd, Epipolar, Entropy };

	static int max_width;
	static bool manually_content;
	//static bool epipolar_mode;
	static Method method;


};


}

#endif
