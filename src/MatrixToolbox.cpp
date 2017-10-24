
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
	
	Vec9d test;
	cv::Mat T = A * invA;
	memcpy(test.ptr(), T.data, sizeof(test));
	cv::Mat T2 = A.inv();
	return to;
}

} // namespace


/*

	
	//A.inverse(invA);
	
	
*/
