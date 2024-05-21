/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
cbic即wbic，这里按照了nabo作者的叫法，没改名
*/

#pragma once
#include "eigen.h"
#include"ini.h"

namespace Blc
{
	class cbicClass
	{
	public:
		static cbicClass &instance();
		void init(double dt);
		void setZero();
		void setCiParam(double mu);						   // 摩擦系数mu，旋转摩擦系数muz，脚面半长lx，脚面半宽ly
		void setFr(const vec3d (&mf)[2]);													   // F系，出力方向
		void run(vec3d &aAcc, vec3d &acc);													   // F系，根据相位分配力
		void run(vec3d &aAcc, vec3d &acc, const double (&stPgs)[2], bool isLock);			   // F系，根据相位分配力-支撑腿平滑
		void run(vec3d &aAcc, vec3d &acc, const double (&stPgs)[2], const double (&swPgs)[2]); // F系，根据相位分配力-支撑腿、空中腿分别平滑
		vec3d mf[2];  // 参考力																		   // F系，出力方向

	private:
		cbicClass();
		cbicClass(const cbicClass &) = delete;
		cbicClass &operator=(const cbicClass &) = delete;

		double maxFz = 200;
		class impClass;
		impClass &imp;
	};
} // namespace