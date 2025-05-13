# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

# Include directories
INCLUDE_DIRS = -I/usr/local/cuda/include -I/usr/include/opencv4

# Library directories
LIB_DIRS = -L/usr/local/cuda/lib64 -L/usr/lib

# Libraries
LIBS = -lnvinfer -lnvonnxparser -lnvparsers -lcudart \
       -lopencv_core -lopencv_imgcodecs -lopencv_highgui \
       -lopencv_imgproc -lopencv_flann -lopencv_videoio

# Source files
SRCS = RunningAI.cpp

# Output executable
OUTPUT = RunningAI

all: $(OUTPUT)

$(OUTPUT): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) $(LIB_DIRS) $(SRCS) -o $(OUTPUT) $(LIBS)

clean:
	rm -f $(OUTPUT)
