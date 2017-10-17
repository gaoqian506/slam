

INCLUDE_DIR = -Iinclude
SRCS=$(wildcard  src/*.cpp)
OBJS=$(SRCS:%.cpp=%.o)
TARGET = slam_demo


$(TARGET) : $(OBJS)
	g++ $(OBJS) -o $(TARGET)

$(OBJS) : %.o : %.cpp
	g++ -c $(INCLUDE_DIR) $< -o $@



clean:
	rm -f $(OBJS) $(TARGET)
