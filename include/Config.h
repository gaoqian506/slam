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
		Of1,		// E = Iudu-dg
					// of[i] = mean(of[i]);
		Gof1,		// gradient-based optical flow
					// E = Gudu-dg (g:gradient)
		Of2,		// IuUt + It = 0;
					// Ut = sum(wUt)/W, W = dot(Iu0, Iu1)
		Of3,		// E(dut) = (iu0dut-it)^2+(iu1dut-it)^+
					//		lamda w(iu0(dut-duu)^2)
	};

	static int max_width;
	static bool manually_content;
	//static bool epipolar_mode;
	static Method method;
	static int of_skip;
	static int field_skip;
	static double sigma2_dgdu; //0.2*0.2;
	static double sigma2_dgdt; //0.2*0.2;
	static double du_smooth_lamda_of3;
	static double stable_factor_of3;
	static double min_weight_of3;
	static bool use_i1_constrain_of3;
	static double epi_sigma2_of3;


};


}

#endif
