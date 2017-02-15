OBJS=Main.o MatrixUtil.o PositionUtil.o MatrixConverter.o RobustImageMatching.o Combination.o EigenValue.o CvUtil.o
OPENCV=/usr/local/opencv/opencv-3.1.0
INCLUDE=-I./include -I./ -I$(OPENCV)/include
CPPFLAGS=$(INCLUDE)
LDFLAGS=-L/usr/local/lib -L$(OPENCV)/lib
CXXFLAGS=-std=c++11 -DEIGEN_NO_DEBUG -fopenmp
LIBS=$(shell ls $(OPENCV)/lib | grep so$$ | sed 's/.so//g' | sed 's/^lib/-l/g')

all : $(OBJS)
	$(CXX) -o RobustImageMatching -fopenmp $(LDFLAGS) $(LIBS) $(OBJS)

clean : 
	$(RM) *.o
	$(RM) RobustImageMatching 

