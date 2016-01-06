#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "MatrixUtil.h"
#include "PositionUtil.h"
#include "MatrixConverter.h"
#include "RobustImageMatching.h"
#include "Combination.h"
#include "Timer.h"
#include "CvUtil.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv)
{
	if (argc < 5) {
		cerr << argv[0] << " <image path 1> <image path 2> <positions path 1> <positions path 2> [w=9] [k=3] [d=3] [f0=600]" << endl;
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
	if (argc >= 6)
		w = stoi(argv[5]);
	cout << "w : " << w << endl;

	cout << "remove protruding positions" << endl;
	positionPtrs1 = RobustImageMatching::removeProtrudingPositions(positionPtrs1, w, gray1.cols(), gray1.rows());
	positionPtrs2 = RobustImageMatching::removeProtrudingPositions(positionPtrs2, w, gray2.cols(), gray2.rows());

	vector<MatrixXd> positionDoubles1 = MatrixConverter::convert2MatrixXd(positionPtrs1);
	vector<MatrixXd> positionDoubles2 = MatrixConverter::convert2MatrixXd(positionPtrs2);
	vector<MatrixXd*> positionDoublePtrs1 = MatrixConverter::convert2MatrixPointer(positionDoubles1);
	vector<MatrixXd*> positionDoublePtrs2 = MatrixConverter::convert2MatrixPointer(positionDoubles2);


	cout << "positions1 size : " << positionPtrs1.size() << endl;
	cout << "positions2 size : " << positionPtrs2.size() << endl;

	vector<MatrixXd> Tps = RobustImageMatching::computeTs(gray1, positionPtrs1, w);
	vector<MatrixXd> Tqs = RobustImageMatching::computeTs(gray2, positionPtrs2, w);

	vector<Combination> combinations = Combination::generateCombinations(positionPtrs1.size(), positionPtrs2.size());
	vector<CombinationPointer> combinationPtrs = CombinationPointer::convert2CombinationPointer(combinations);

	cout << "combinations size : " << combinations.size() << endl;

	Timer timer1;
	cout << "compute Js..." << flush;
	vector<double> Js = RobustImageMatching::computeJs(combinationPtrs, Tps, Tqs);
	cout << "done(" << timer1.get() << ")." << endl;

	int L = (positionPtrs1.size() < positionPtrs2.size()) ? positionPtrs1.size() : positionPtrs2.size();
	double JBar = RobustImageMatching::computeJBar(Js, L);

	timer1.get();
	cout << "solve phi..." << flush;
	double s;
	try {
		s = RobustImageMatching::solvePhi(Js, JBar);
	}
	catch (const NotConvergedException& e) {
		return 2;
	}
	cout << "done(" << timer1.get() << ")." << endl;

	vector<double> P0s = RobustImageMatching::computeP0s(Js, s);

	double k = 3.0;
	if (argc >= 7)
		k = stod(argv[6]);
	cout << "k : " << k << endl;

	vector<CombinationPointer> localCorrespondence = RobustImageMatching::getLocalCorrespondence(combinationPtrs, P0s, k);

	cout << "local correspondence size : " << localCorrespondence.size() << endl;

	timer1.get();
	cout << "compute P1s..." << flush;
	vector<double> P1s;
	try {
		P1s = RobustImageMatching::computeP1s(combinationPtrs, localCorrespondence, positionDoublePtrs1, positionDoublePtrs2);
	}
	catch (const InvalidDataNumException& e) {
		return 3;
	}
	catch (const InvalidDeterminantException& e) {
		return 4;
	}
	cout << "done(" << timer1.get() << ")." << endl;

	vector<CombinationPointer> spatialCorrespondence = RobustImageMatching::getSpatialCorrespondence(combinationPtrs, P0s, P1s, k);

	cout << "spatial correspondence size : " << spatialCorrespondence.size() << endl;

	double f0 = 600.0;
	if (argc >= 9)
		f0 = stod(argv[8]);
	cout << "f0 : " << f0 << endl;

	vector<MatrixXd> xs1 = RobustImageMatching::computeXs(positionDoublePtrs1, f0);
	vector<MatrixXd> xs2 = RobustImageMatching::computeXs(positionDoublePtrs2, f0);
	vector<MatrixXd*> xPtrs1 = MatrixConverter::convert2MatrixPointer(xs1);
	vector<MatrixXd*> xPtrs2 = MatrixConverter::convert2MatrixPointer(xs2);

	timer1.get();
	cout << "compute H..." << flush;
	MatrixXd H;
	try {
		H = RobustImageMatching::computeH(spatialCorrespondence, xPtrs1, xPtrs2);
	}
	catch (const EigenValueException& e) {
		return 5;
	}
	catch (const NotConvergedException& e) {
		return 6;
	}
	cout << "done(" << timer1.get() << ")." << endl;

	vector<double> Ds = RobustImageMatching::computeDs(combinationPtrs, xPtrs1, xPtrs2, H);

	double DBar = RobustImageMatching::computeJBar(Ds, L);

	timer1.get();
	cout << "solve phi..." << flush;
	try {
		s = RobustImageMatching::solvePhi(Ds, DBar);
	}
	catch (const NotConvergedException& e) {
		return 7;
	}
	cout << "done(" << timer1.get() << ")." << endl;

	vector<double> P2s = RobustImageMatching::computeP0s(Ds, s);

	vector<CombinationPointer> globalCorrespondence = RobustImageMatching::getGlobalCorrespondence(combinationPtrs, P0s, P1s, P2s, k);

	cout << "global correspondence size : " << globalCorrespondence.size() << endl;

	double d = 3.0;
	if (argc >= 8)
		d = stod(argv[7]);
	cout << "d : " << d << endl;

	cout << "get ransac correspondence..." << flush;
	vector<CombinationPointer> ransacCorrespondence;
	try {
		ransacCorrespondence = RobustImageMatching::getRansacCorrespondence(combinationPtrs, globalCorrespondence, xPtrs1, xPtrs2, P0s, P1s, P2s, k, d, f0);
	}
	catch (const InvalidDataNumException& e) {
		return 8;
	}
	cout << "done(" << timer1.get() << ")." << endl;

	cout << "ransac correspondence size : " << ransacCorrespondence.size() << endl;

	CombinationPointer::write("robustMatches.dat", ransacCorrespondence, positionPtrs1, positionPtrs2);

	cout << "終了 : " << timer.get() << " sec" << endl;

	Mat image = CvUtil::drawCorrespondence(image1, image2, ransacCorrespondence, positionPtrs1, positionPtrs2);
	imwrite("robustMatches.jpg", image);

	return 0;
}

