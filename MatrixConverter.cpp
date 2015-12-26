#include "MatrixConverter.h"

using namespace std;
using namespace Eigen;

MatrixXd MatrixConverter::convert2MatrixXd(const MatrixXi& src)
{
	MatrixXd dst(src.rows(), src.cols());
	for (int i = 0; i < src.rows(); i++)
	for (int j = 0; j < src.cols(); j++)
		dst(i, j) = (double)src(i, j);
	return dst;
}

vector<MatrixXd> MatrixConverter::convert2MatrixXd(const vector<MatrixXi>& src)
{
	vector<MatrixXd> dst;
	for (unsigned i = 0; i < src.size(); i++)
		dst.push_back(convert2MatrixXd(src[i]));
	return dst;
}

