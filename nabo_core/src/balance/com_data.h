#pragma once
#include "eigen.h"
namespace WBC
{
	struct comDataStruct
	{
		comDataStruct();
		vec3d acc, aAcc;
		void setZero();
		void pid(const vec3d &errA, const vec3d &errP, const vec3d &errW, const vec3d &errV);
		void filt();

	private:
		vec3d kpP, kiP, kdP, kpA, kiA, kdA;
		vec3d errPSum, errASum;
		Alg::filterTwoClass fil[6];
		// Ei::filterOneClass filter[2];
		// vec3d accLast,aAccLast;
		// void smoothing(){
		// 	Ei::cmd2out1step(acc,accLast,0.03);
		// 	Ei::cmd2out1step(aAcc,aAccLast,0.06);
		// 	acc=accLast;
		// 	aAcc=aAccLast;
		// }
	};
} // namespace