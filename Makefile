

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

longxuan:
	./$(TARGET) data/videos/longxuan.mp4
	
texie:
	./$(TARGET) data/videos/texie.mp4

shanghai:
	./$(TARGET) data/videos/shanghai.mp4
	
move_left_8x8:
	./$(TARGET) data/images/move_left_8x8_001.png data/images/move_left_8x8_002.png

move_left_64x64:
	./$(TARGET) data/images/move_left_64x64_001.png data/images/move_left_64x64_002.png
	
tsukuba:
	./$(TARGET) data/images/tsukuba_l.png data/images/tsukuba_r.png
	
debug:
	gdb ./$(TARGET)

