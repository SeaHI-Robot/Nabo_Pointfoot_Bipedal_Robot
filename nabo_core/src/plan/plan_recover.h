/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#pragma once
#include "plan.h"
#include <array>

namespace Plan
{

	class rcPlanClass final : public basePlanClass  // 继承basePlanClass类
	{
	public:
		rcPlanClass();
		bool run(const Nabo::inputStruct &inRef, Nabo::outputStruct &outRef) override final;
		void log(bool &inLogFlag) override final;

	private:
		void init();
		void plan();
		void dwdate(Nabo::outputStruct &outRef);
		array<double, NMot> j0, j1, j2;
		double errSum[NMot];
		double tReach;
		double kp, ki, kd;
	};
} // namespace
