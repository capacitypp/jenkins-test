#ifndef ___Class_RobustImageMatching
#define ___Class_RobustImageMatching

#include <vector>

#include <Eigen/Core>

class RobustImageMatching {
	static bool isDuplicatePosition(const Eigen::MatrixXi& position1, const Eigen::MatrixXi& position2);
	static bool isDuplicatePosition(const std::vector<Eigen::MatrixXi*>& positionPtrs, const Eigen::MatrixXi& position);
public:
	static std::vector<Eigen::MatrixXi*> removeDuplicatePositions(const std::vector<Eigen::MatrixXi*>& positionPtrs);
};

#endif

