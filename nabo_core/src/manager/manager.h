/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
步态规划全隔离封装，提供外部接口
*/

#pragma once
#include "nabo_config.h"

namespace Mng
{
	class manageClass
	{
	public:
		static manageClass &instance();
		void setDt(double dt);
		void run(Nabo::inputStruct &in, Nabo::outputStruct &out);

	private:
		manageClass();
		manageClass(const manageClass &) = delete;
		manageClass &operator=(const manageClass &) = delete;
		class impClass;
		impClass &imp;
	};
} // namespace
