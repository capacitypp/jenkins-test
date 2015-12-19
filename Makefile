OBJS=Main.o

all : $(OBJS)
	$(CXX) -o Main $(LDFLAGS) $(OBJS)

clean : 
	$(RM) *.o
	$(RM) Main

