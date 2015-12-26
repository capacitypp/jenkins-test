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

vector<MatrixXd> MatrixConverter::convert2MatrixXd(const vector<MatrixXi*>& src)
{
	vector<MatrixXd> dst;
	for (unsigned i = 0; i < src.size(); i++)
		dst.push_back(convert2MatrixXd(*src[i]));
	return dst;
}

vector<MatrixXi*> MatrixConverter::convert2MatrixPointer(vector<MatrixXi>& src)
{
	vector<MatrixXi*> positions;
	for (unsigned i = 0; i < src.size(); i++) {
		MatrixXi& position = src[i];
		positions.push_back(&position);
	}

	return positions;
}

vector<MatrixXd*> MatrixConverter::convert2MatrixPointer(vector<MatrixXd>& src)
{
	vector<MatrixXd*> positions;
	for (unsigned i = 0; i < src.size(); i++) {
		MatrixXd& position = src[i];
		positions.push_back(&position);
	}

	return positions;
}

