

INCLUDE_DIR = -Iinclude
LIBS = -lGL -lGLU -lglut -lpthread
FLAGS = -g
SRCS=$(wildcard  src/*.cpp)
OBJS=$(SRCS:%.cpp=%.o)
TARGET = slam_demo


$(TARGET) : $(OBJS)
	g++ $(OBJS) $(FLAGS) $(LIBS) -o $(TARGET)

$(OBJS) : %.o : %.cpp
	g++ -c $(INCLUDE_DIR) $(LIBS) $(FLAGS) $< -o $@



clean:
	rm -f $(OBJS) $(TARGET)
	
run:
	./$(TARGET)
	
	
debug:
	gdb ./$(TARGET)
