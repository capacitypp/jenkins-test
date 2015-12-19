OBJS=Main.o
INCLUDE=-I./include -I./
CPPFLAGS=$(INCLUDE)

all : $(OBJS)
	$(CXX) -o Main $(LDFLAGS) $(OBJS)

clean : 
	$(RM) *.o
	$(RM) Main

