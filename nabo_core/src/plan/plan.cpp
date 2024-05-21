/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#include "plan.h"

namespace Plan
{

	void basePlanClass::setDt(double dtt)
	{
		dt = dtt;
		rbt.setDt(dt);
		logCnt = int(0.01 / dt);	// 在这里调整log的记录频率 0.01s一帧log
		consoleCnt = int(0.05 / dt); // 在这里调整控制台输出频率 0.05s 打印一次
	}
	void basePlanClass::baseInit()
	{
		tim = 0;
		quitFlag = 0;
		For(NMot)
		{
			wFil[i].init(dt, 500, 1, 0);
			tOutFil[i].init(dt, 200, 1, 0);
		}
		rbt.imu.init(in.rpy);
		rbt.setSwing(0, 0);
		rbt.setSwing(1, 0);
	}

} // namespace
