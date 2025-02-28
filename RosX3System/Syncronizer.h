#pragma once
#include "Datatypes.h"
class Syncronizer
{
	std::queue<Request> tasks;
public:
	Syncronizer() = default;
	/*void addTask(Request Request);
	Request getTask();
	bool taskAvailable();
	~Syncronizer();*/
};

