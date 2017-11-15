
#include "MatrixToolbox.h"
#include "opencv2/core/core.hpp"

namespace ww {


Vec9d MatrixToolbox::inv_matrix_3x3(Vec9d& from) {
/*
	from = 0;
	from[0] = 500;
	from[2] = 320;
	from[4] = 500;
	from[5] = 240;
	from[8] = 1;
*/

	Vec9d to;
	cv::Mat A = cv::Mat(3, 3, CV_64F, from.ptr());
	cv::Mat invA = cv::Mat(3, 3, CV_64F, to.ptr());
	cv::invert(A, invA);
	
	//Vec9d test;
	//cv::Mat T = A * invA;
	//memcpy(test.ptr(), T.data, sizeof(test));
	//cv::Mat T2 = A.inv();
	return to;
}

void MatrixToolbox::update_rotation(Vec9d& R, const Vec3d& eula) {

	//double dR[9] = {
	//	1, -eula[2], eula[1], 
	//	eula[2], 1, -eula[0],
	//	-eula[1], eula[0], 1
	//};

	//[ cos(b)*cos(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c), sin(a)*sin(c) + cos(a)*cos(c)*sin(b)]
//[ cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(a)*sin(b)*sin(c) - cos(c)*sin(a)]
//[       -sin(b),                        cos(b)*sin(a),                        cos(a)*cos(b)]
	double dR[9] = {
		cos(eula[1])*cos(eula[0])-cos(eula[2])*sin(eula[1])*sin(eula[0]), 
		-cos(eula[2])*cos(eula[0])*sin(eula[1])-cos(eula[1])*sin(eula[0]),
		sin(eula[1])*sin(eula[2]),
		cos(eula[0])*sin(eula[1])+cos(eula[1])*cos(eula[2])*sin(eula[0]),
		cos(eula[1])*cos(eula[2])*cos(eula[0])-sin(eula[1])*sin(eula[0]), 
		-cos(eula[1])*sin(eula[2]),
		sin(eula[2])*sin(eula[0]),
		cos(eula[0])*sin(eula[2]),
		cos(eula[2])
	};


	cv::Mat cvR = cv::Mat(3, 3, CV_64F, R.val);
	cv::Mat cvdR = cv::Mat(3, 3, CV_64F, dR);
	cvR = cvdR  * cvR;
	
	cv::SVD svd(cvR);
	cvR = svd.u*svd.vt;
}
void MatrixToolbox::rectify_rotation(Vec9d& R) {
	assert(0);
}

} // namespace


/*

	
	//A.inverse(invA);
	
	
*/
