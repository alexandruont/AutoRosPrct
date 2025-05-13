CXX = g++
CXXFLAGS = -Wall -Wextra -O2 -Isrc/ -Isrc/ChasisController $(shell pkg-config --cflags roscpp geometry_msgs)
SRC = RosX3System.cpp \
      src/CommonData.cpp \
      src/Networking/NetworkBase.cpp \
      src/Networking/tcpClient/client.cpp \
      src/Networking/tcpClient/common.cpp \
      src/Networking/tcpClient/pipe_ret_t.cpp \
      src/Networking/tcpClient/tcp_client.cpp \
      src/ChasisController/ChasisHandler.cpp \
      src/ChasisController/Task_Executer.cpp

OBJ = $(SRC:.cpp=.o)
LIBS = -pthread $(shell pkg-config --libs roscpp geometry_msgs)
EXEC = RosX3System

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(EXEC) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(EXEC)