#ifndef ___Class_RobustImageMatching
#define ___Class_RobustImageMatching

#include <vector>

#include <Eigen/Core>

#include "Combination.h"
#include "Exception.h"

class RobustImageMatching {
	static bool isDuplicatePosition(const Eigen::MatrixXi& position1, const Eigen::MatrixXi& position2);
	static bool isDuplicatePosition(const std::vector<Eigen::MatrixXi*>& positionPtrs, const Eigen::MatrixXi& position);
	static bool isProtrudingPosition(const Eigen::MatrixXi& position, int w, int width, int height);
	static Eigen::MatrixXd computeT(const Eigen::MatrixXd& gray, const Eigen::MatrixXi& position, int w);
	static double computeJ(const Eigen::MatrixXd& Tp, const Eigen::MatrixXd& Tq);
	static bool searchCombination(const std::vector<CombinationPointer>& combinationPtrs, const CombinationPointer& combinationPtr);
	static void evaluatePhiAndDiffPhi(const std::vector<double>& JsJBar, const std::vector<double>& Js, double* phi, double* diffPhi, double s);
	static std::vector<Eigen::MatrixXd> computeFlow(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*>& positionDoublePtrs1, const std::vector<Eigen::MatrixXd*>& positionDoublePtrs2);
	static Eigen::MatrixXd computeX(const Eigen::MatrixXd& positionDouble, double f0);
	static Eigen::MatrixXd computeTensorM(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*>& xPtrs1, const std::vector<Eigen::MatrixXd*>& xPtrs2, const std::vector<Eigen::MatrixXd>& Ws);
	static Eigen::MatrixXd tensorProduct(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b);
	static int epsilon(int i, int j, int k);
	static Eigen::MatrixXd computeTensorN(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*>& xPtrs1, const std::vector<Eigen::MatrixXd*>& xPtrs2, const std::vector<Eigen::MatrixXd>& Ws, const std::vector<Eigen::MatrixXd>& V0s);
	static Eigen::MatrixXd getEpipolarMatrix(const Eigen::MatrixXd& x1, const Eigen::MatrixXd& x2);
	static std::vector<Eigen::MatrixXd> getEpipolarMatrixs(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*>& xPtrs1, const std::vector<Eigen::MatrixXd*>& xPtrs2);
	static std::vector<int> getRandomIndexes(int max, int n);
	static Eigen::MatrixXd solveF(const std::vector<Eigen::MatrixXd>& epipolarMatrixs, const std::vector<int>& indexes);
	static std::vector<double> computeDFs(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*>& xPtrs1, const std::vector<Eigen::MatrixXd*>& xPtrs2, const Eigen::MatrixXd& F);
	static std::vector<CombinationPointer> takeOutCombinations(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<double>& DFs, double threshold);
public:
	static std::vector<Eigen::MatrixXi*> removeDuplicatePositions(const std::vector<Eigen::MatrixXi*>& positionPtrs);
	static std::vector<Eigen::MatrixXi*> removeProtrudingPositions(const std::vector<Eigen::MatrixXi*>& positionPtrs, int w, int width, int height);
	static std::vector<Eigen::MatrixXd> computeTs(const Eigen::MatrixXd& gray, const std::vector<Eigen::MatrixXi*>& positionPtrs, int w);
	static std::vector<double> computeJs(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd>& Tps, const std::vector<Eigen::MatrixXd>& Tqs);
	static void setJs(std::vector<CombinationPointer>& combinationPtrs, const std::vector<double>& Js);
	static std::vector<CombinationPointer> one2OneReduction(const std::vector<CombinationPointer>& src);
	static double computeJBar(const std::vector<double>& srcJs, unsigned L);
	static double solvePhi(const std::vector<double>& Js, double JBar) throw(NotConvergedException);
	static std::vector<double> computeP0s(const std::vector<double>& Js, double s);
	static std::vector<CombinationPointer> takeOutCombinations(const std::vector<CombinationPointer>& combinationPtrs, double threshold);
	static std::vector<CombinationPointer> getLocalCorrespondence(const std::vector<CombinationPointer>& srcCombinationPtrs, const std::vector<double>& P0s, double k);
	static std::vector<double> computeP1s(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<CombinationPointer>& localCorrespondence, const std::vector<Eigen::MatrixXd*>& positionDoublePtrs1, const std::vector<Eigen::MatrixXd*>& positionDoublePtrs2) throw(InvalidDataNumException, InvalidDeterminantException);
	static std::vector<CombinationPointer> getSpatialCorrespondence(const std::vector<CombinationPointer>& srcCombinationPtrs, const std::vector<double>& P0s, const std::vector<double>& P1s, double k);
	static std::vector<Eigen::MatrixXd> computeXs(const std::vector<Eigen::MatrixXd*>& positionDoublePtrs, double f0);
	static Eigen::MatrixXd computeH(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*> xPtrs1, const std::vector<Eigen::MatrixXd*> xPtrs2) throw(EigenValueException, NotConvergedException);
	static std::vector<double> computeDs(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd*>& xPtrs1, const std::vector<Eigen::MatrixXd*>& xPtrs2, const Eigen::MatrixXd& H);
	static std::vector<CombinationPointer> getGlobalCorrespondence(const std::vector<CombinationPointer>& srcCombinationPtrs, const std::vector<double>& P0s, const std::vector<double>& P1s, const std::vector<double>& P2s, double k);
	static std::vector<CombinationPointer> getRansacCorrespondence(const std::vector<CombinationPointer>& srcCombinationPtrs, const std::vector<CombinationPointer>& globalCorrespondence, const std::vector<Eigen::MatrixXd*>& xPtrs1, const std::vector<Eigen::MatrixXd*>& xPtrs2, const std::vector<double>& P0s, const std::vector<double>& P1s, const std::vector<double>& P2s, double k, double d, double f0) throw(InvalidDataNumException);
};

#endif

