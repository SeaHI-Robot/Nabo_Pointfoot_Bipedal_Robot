/*=========== ***doc description @ yyp*** ===========
项目最终封装，仅提供c接口
=====================================================*/
#pragma once
#include"nabo_config.h"

namespace Nabo{

extern "C" void setDt(double dt);
extern "C" void run(inputStruct &in,outputStruct &out);

}//namespace
