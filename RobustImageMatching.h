#ifndef ___Class_RobustImageMatching
#define ___Class_RobustImageMatching

#include <vector>

#include <Eigen/Core>

#include "Combination.h"

class RobustImageMatching {
	static bool isDuplicatePosition(const Eigen::MatrixXi& position1, const Eigen::MatrixXi& position2);
	static bool isDuplicatePosition(const std::vector<Eigen::MatrixXi*>& positionPtrs, const Eigen::MatrixXi& position);
	static bool isProtrudingPosition(const Eigen::MatrixXi& position, int w, int width, int height);
	static Eigen::MatrixXd computeT(const Eigen::MatrixXd& gray, const Eigen::MatrixXi& position, int w);
	static double computeJ(const Eigen::MatrixXd& Tp, const Eigen::MatrixXd& Tq);
	static bool searchCombination(const std::vector<CombinationPointer>& combinationPtrs, const CombinationPointer& combinationPtr);
public:
	static std::vector<Eigen::MatrixXi*> removeDuplicatePositions(const std::vector<Eigen::MatrixXi*>& positionPtrs);
	static std::vector<Eigen::MatrixXi*> removeProtrudingPositions(const std::vector<Eigen::MatrixXi*>& positionPtrs, int w, int width, int height);
	static std::vector<Eigen::MatrixXd> computeTs(const Eigen::MatrixXd& gray, const std::vector<Eigen::MatrixXi*>& positionPtrs, int w);
	static std::vector<double> computeJs(const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXd>& Tps, const std::vector<Eigen::MatrixXd>& Tqs);
	static void setJs(std::vector<CombinationPointer>& combinationPtrs, const std::vector<double>& Js);
	static std::vector<CombinationPointer> one2OneReduction(const std::vector<CombinationPointer>& src);
};

#endif

