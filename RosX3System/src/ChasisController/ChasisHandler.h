#ifndef CHASIS_HANDLER_H
#define CHASIS_HANDLER_H
#include "Datatypes.h"
#include "CommonData.h"
#include "Framework.h"

void initChasisHandler(int argc, char** argv);
void closeChasisHandler();

extern RP::vec3 chMovement;
#endif // CHASIS_HANDLER_H
