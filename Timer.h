#ifndef ___Class_Timer
#define ___Class_Timer

#include <chrono>

class Timer {
	std::chrono::system_clock::time_point before;
public:
	Timer(void) : before(std::chrono::system_clock::now()) { }
	double get(void) {
		std::chrono::system_clock::time_point current = std::chrono::system_clock::now();
		double time = std::chrono::duration_cast<std::chrono::milliseconds>(current - before).count();
		before = current;
		return time / 1000.0;
	}
};

#endif

