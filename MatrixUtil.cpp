#include "MatrixUtil.h"

using namespace std;
using namespace cv;
using namespace Eigen;

MatrixXd MatrixUtil::convertGray2MatrixXd(const Mat& gray)
{
	MatrixXd m(gray.rows, gray.cols);
	unsigned char* ptr = gray.data;
	for (int i = 0; i < gray.rows; i++) {
		for (int j = 0; j < gray.cols; j++) {
			m(i, j) = (double)ptr[i * gray.step + j * gray.elemSize()];
		}
	}
	return m;
}

