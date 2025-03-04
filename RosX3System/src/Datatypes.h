#ifndef DATATYPES_H
#define DATATYPES_H

#include "Framework.h"

enum class TaskType {
	GET,
	UPDATE,
	COMMAND,
	CLOSE,
	DEBUG_LOG
};

enum class InfoType {
	LIDAR,
	CAMERA,
	LOCAL_MAP,
	ARM_STATE,
	DATA_STREAM
};

struct ArrayData {
	char* data = nullptr;
	size_t size = 0;
};

struct Request{
	TaskType type;
	InfoType infoType;
	size_t size;
};

struct Task {
	TaskType type;
	InfoType infoType;
	ArrayData data;
	void*(*callback);
};
#endif // !FRAMEWORK_H