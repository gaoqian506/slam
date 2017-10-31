

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

test_longxuan:
	./$(TARGET) data/videos/test1.mp4
	
test_texie:
	./$(TARGET) data/videos/test2.mp4
	
test_move_left_8x8:
	./$(TARGET) data/images/move_left_8x8/001.png data/images/move_left_8x8/002.png
	
	
debug:
	gdb ./$(TARGET)

