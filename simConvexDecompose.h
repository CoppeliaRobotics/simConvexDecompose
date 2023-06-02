#pragma once

#include <simLib/simExp.h>

SIM_DLLEXPORT int simInit(const char*);
SIM_DLLEXPORT void simCleanup();
SIM_DLLEXPORT void simMsg(int,int*,void*);
SIM_DLLEXPORT void simHACD(void*);
SIM_DLLEXPORT void simVHACD(void*);
