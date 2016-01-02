#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "MatrixUtil.h"
#include "PositionUtil.h"
#include "MatrixConverter.h"
#include "RobustImageMatching.h"
#include "Combination.h"
#include "Timer.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv)
{
	if (argc != 5) {
		cerr << argv[0] << " <image path 1> <image path 2> <positions path 1> <positions path 2>" << endl;
		return 1;
	}

	Timer timer;

	string imagePath1(argv[1]);
	string imagePath2(argv[2]);
	Mat image1 = imread(imagePath1);
	Mat image2 = imread(imagePath2);

	Mat gray_image1, gray_image2;
	cvtColor(image1, gray_image1, CV_RGB2GRAY);
	cvtColor(image2, gray_image2, CV_RGB2GRAY);
	MatrixXd gray1 = MatrixUtil::convertGray2MatrixXd(gray_image1);
	MatrixXd gray2 = MatrixUtil::convertGray2MatrixXd(gray_image2);

	string positionsPath1(argv[3]);
	string positionsPath2(argv[4]);
	vector<MatrixXi> positions1 = PositionUtil::readPositions(positionsPath1);
	vector<MatrixXi> positions2 = PositionUtil::readPositions(positionsPath2);

	cout << "positions1 size : " << positions1.size() << endl;
	cout << "positions2 size : " << positions2.size() << endl;

	vector<MatrixXi*> positionPtrs1 = MatrixConverter::convert2MatrixPointer(positions1);
	vector<MatrixXi*> positionPtrs2 = MatrixConverter::convert2MatrixPointer(positions2);

	cout << "remove duplicate positions" << endl;
	positionPtrs1 = RobustImageMatching::removeDuplicatePositions(positionPtrs1);
	positionPtrs2 = RobustImageMatching::removeDuplicatePositions(positionPtrs2);

	cout << "positions1 size : " << positionPtrs1.size() << endl;
	cout << "positions2 size : " << positionPtrs2.size() << endl;

	int w = 9;

	cout << "remove protruding positions" << endl;
	positionPtrs1 = RobustImageMatching::removeProtrudingPositions(positionPtrs1, w, gray1.cols(), gray1.rows());
	positionPtrs2 = RobustImageMatching::removeProtrudingPositions(positionPtrs2, w, gray2.cols(), gray2.rows());

	cout << "positions1 size : " << positionPtrs1.size() << endl;
	cout << "positions2 size : " << positionPtrs2.size() << endl;

	vector<MatrixXd> Tps = RobustImageMatching::computeTs(gray1, positionPtrs1, w);
	vector<MatrixXd> Tqs = RobustImageMatching::computeTs(gray2, positionPtrs2, w);

	vector<Combination> combinations = Combination::generateCombinations(positionPtrs1.size(), positionPtrs2.size());
	vector<CombinationPointer> combinationPtrs = CombinationPointer::convert2CombinationPointer(combinations);

	cout << "combinations size : " << combinations.size() << endl;

	vector<double> Js = RobustImageMatching::computeJs(combinationPtrs, Tps, Tqs);

	RobustImageMatching::setJs(combinationPtrs, Js);

	vector<MatrixXd> positionDoubles1 = MatrixConverter::convert2MatrixXd(positionPtrs1);
	vector<MatrixXd> positionDoubles2 = MatrixConverter::convert2MatrixXd(positionPtrs2);

	cout << "$B=*N;(B : " << timer.get() << " sec" << endl;

	return 0;
}

