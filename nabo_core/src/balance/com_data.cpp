/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
com_data对应wbic中对“加速度”的部分
*/

#include "com_data.h"
namespace WBC
{
	comDataStruct::comDataStruct()
	{
		kpA = vec3d(40, 40, 40);
		kiA = vec3d(0.02, 0.02, 0.02);
		kdA = vec3d(10, 10, 10);
		kpP = vec3d(30, 30, 40);
		kiP = vec3d(0.02, 0.02, 0.02);
		kdP = vec3d(10, 10, 10);
	
		// kpA = vec3d(40, 40, 40);
		// kiA = vec3d(0.02, 0.02, 0.02);
		// kdA = vec3d(10, 10, 10);
		// kpP = vec3d(20, 20, 40);
		// kiP = vec3d(0.005, 0.005, 0.005);
		// kdP = vec3d(1, 1, 2);	
		setZero();
	}


	void comDataStruct::setZero()
	{
		aAcc.setZero();
		acc.setZero();
		errASum.setZero();
		errPSum.setZero();
		for (int i = 0; i != 6; ++i)
		{
			fil[i].reset(30, 0);
		}
	}
	
	
	void comDataStruct::pid(const vec3d &errA, const vec3d &errP, const vec3d &errW, const vec3d &errV)
	{
		errASum += errA;
		errPSum += errP;
		// 这里为什么要乘一个系数？？ 可能是控制sum，让他不要过大吧，同时更像制造了一个discount，越近的error discount factor越大
		errASum *= 0.999; 
		errPSum *= 0.998;

		aAcc = kpA.cwiseProduct(errA) + kiA.cwiseProduct(errASum) + kdA.cwiseProduct(errW);
		
		acc = kpP.cwiseProduct(errP) + kiP.cwiseProduct(errPSum) + kdP.cwiseProduct(errV);
		// acc[2] = 0;
	}
	
	
	void comDataStruct::filt()
	{
		aAcc[0] = fil[0].filt(aAcc[0]);
		aAcc[1] = fil[1].filt(aAcc[1]);
		aAcc[2] = fil[2].filt(aAcc[2]);
		acc[0] = fil[3].filt(acc[0]);
		acc[1] = fil[4].filt(acc[1]);
		acc[2] = fil[5].filt(acc[2]);
	}
}