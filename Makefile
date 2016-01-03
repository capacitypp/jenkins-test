OBJS=Main.o MatrixUtil.o PositionUtil.o MatrixConverter.o RobustImageMatching.o Combination.o EigenValue.o
INCLUDE=-I./include -I./
CPPFLAGS=$(INCLUDE)
LDFLAGS=-L/usr/local/lib
CXXFLAGS=-std=c++11
LIBS=-lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_adas -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_datasets -lopencv_face -lopencv_latentsvm -lopencv_objdetect -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_surface_matching -lopencv_text -lopencv_tracking -lopencv_xfeatures2d -lopencv_calib3d -lopencv_features2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_flann -lopencv_ximgproc -lopencv_xobjdetect -lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core -lopencv_hal 

all : $(OBJS)
	$(CXX) -o RobustImageMatching $(LDFLAGS) $(LIBS) $(OBJS)

clean : 
	$(RM) *.o
	$(RM) RobustImageMatching 

