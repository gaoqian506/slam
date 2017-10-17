

INCLUDE_DIR = -Iinclude
FLAGS = -g
SRCS=$(wildcard  src/*.cpp)
OBJS=$(SRCS:%.cpp=%.o)
TARGET = slam_demo


$(TARGET) : $(OBJS)
	g++ $(OBJS) $(FLAGS) -o $(TARGET)

$(OBJS) : %.o : %.cpp
	g++ -c $(INCLUDE_DIR) $(FLAGS) $< -o $@



clean:
	rm -f $(OBJS) $(TARGET)
