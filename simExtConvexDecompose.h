#pragma once

#include <simLib/simExp.h>

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);
SIM_DLLEXPORT void simHACD(void* data);
SIM_DLLEXPORT void simVHACD(void* data);
