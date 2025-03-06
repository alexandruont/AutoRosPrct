#ifndef DATATYPES_H
#define DATATYPES_H

#include "Framework.h"

enum TaskType {
	GET = 0,
	POST = 1,
	COMMAND = 2,
	CLOSE = 3,
	DEBUG_LOG = 4,
	NULL_TASK = 5
};

enum InfoType {
	LIDAR = 0,
	CAMERA_1 = 1,
	CAMERA_2 = 2,
	LOCAL_MAP = 3,
	ARM_STATE = 4,
	DATA_STREAM = 5,
	NULL_INFO = 6
};

struct ArrayData {
	char* data = nullptr;
	size_t size = 0;
};
#pragma pack(push, 1)
struct Request{
	TaskType type;
	InfoType infoType;
	size_t size;
};
#pragma pack(pop)

struct Task {
	TaskType type;
	InfoType infoType;
	ArrayData data;
	void*(*callback);
};
#endif // !FRAMEWORK_H