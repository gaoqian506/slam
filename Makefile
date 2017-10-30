

INCLUDE_DIR = -Iinclude
LIBS = -lGL -lGLU -lglut -lpthread -lopencv_highgui -lopencv_core -lopencv_imgproc
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

test1:
	./$(TARGET) data/test1.mp4
	
test2:
	./$(TARGET) data/test2.mp4
	
	
debug:
	gdb ./$(TARGET)
