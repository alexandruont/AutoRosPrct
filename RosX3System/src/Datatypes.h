#ifndef DATATYPES_H
#define DATATYPES_H

#include "Framework.h"

enum TaskType{
    Get = 0, // Get any kind of information
    Post = 1, // Post some information. Used for file like data. Mainly used by the robot
    Set = 2, // Set is for updating the arm position and robot speed
    Specs = 3, // Sends a request for robot specifications(like number of cameras)

	NULL_TASK = 20
};

enum InfoType{
    None = 0,
    Camera = 1,
    Arm = 2,
    Speed = 3,
    ImageSize = 4
};

struct ArrayData {
	char* data = nullptr;
	size_t size = 0;
};

struct Task {
	TaskType type;
	InfoType infoType;
	ArrayData data;
	void*(*callback);
};
#pragma pack(push, 1)
struct Request{
	TaskType type;
	InfoType infoType;
	uint32_t size;
	Request() = default;
	Request(const Task& t){
		this->type = t.type;
		this->infoType = t.infoType;
		this->size = t.data.size;
	}
};
#pragma pack(pop)


struct SpecStruct{
	int numberOfCameras = 2;
	int camera1[2] = { 256, 256 };
	int camera2[2] = { 256, 256 };
	int numberOfJoints = 6;
};
#endif // !FRAMEWORK_H