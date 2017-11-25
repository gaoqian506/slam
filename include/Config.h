#ifndef __WW_CONFIG_HEADER__
#define __WW_CONFIG_HEADER__

namespace ww {



class Config {

public:

	enum Method { 
		Lsd, 
		Epipolar, 
		Entropy, 	// E(e) = dg*dot(l, e)
					// E(r) = cross(l, e)*(Ir*dr-dg)
					// base on current frame

		Entropy2, 	// E(e) = dg*dot(l, e)
					// E(r) = cross(l, e)*(Ir*dr-dg)
					// base on key frame

		Entropy3, 	// E = (dg-Ir*dr)*dot(l, e)

		Entropy4, 	// E(e) = (dg)*dot(It, e)
					// E(dr) = (dg)*dot(Ir, dr)

		Entropy5, 	// E(dt) = (dg)*dot(It, dt)
					// E(dr) = (dg)*dot(Ir, dr)

		Lsd2,	 	// E(dt) = dot(It, dt)-dg
					// E(dr) = dot(Ir, dr)-dg

		Lsd3,	 	// E(dt) = (dot(It, dt)-dg)^2
					// E(dr) = (dot(Ir, dr)-dg)^2

		Epi2,		// e = dot(dgl, e)
					// E(dr) = w(dot(Ir, dr)-dg)^2
					// w = exp(-dot(l,e)/sigma)
		Lsd4,	 	// E(dt) = (dot(It, dt)-dg)^2
					// E(dr) = w(dot(Ir, dr)-dg)^2
					// w = dot(Ir,t);		
	};

	static int max_width;
	static bool manually_content;
	//static bool epipolar_mode;
	static Method method;


};


}

#endif
