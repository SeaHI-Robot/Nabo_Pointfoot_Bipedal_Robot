/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
项目最终封装，仅提供c接口
*/

#pragma once
#include"nabo_config.h"

namespace Nabo{

extern "C" void setDt(double dt);
extern "C" void run(inputStruct &in,outputStruct &out);

}//namespace
