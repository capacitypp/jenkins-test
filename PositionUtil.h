#ifndef ___Class_PositionUtil
#define ___Class_PositionUtil

#include <vector>
#include <string>

#include <Eigen/Core>

class PositionUtil {
public:
	static std::vector<Eigen::MatrixXi> readPositions(const std::string& fpath);
};

#endif

