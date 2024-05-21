/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#pragma once
//涉及数组大小，必须编译前给定
static const int NLegDof       = 3;
static const int NMot          = NLegDof * 2;
static const int MemMot        = NMot * 8;
static const int NBaseDof      = 6;
static const int NGenDof       = NBaseDof + NMot;
//其他常量
static const double HipY       = 0.1;
static const double HipZ       = -0.1;
static const double LenThigh   = 0.2;
static const double LenShank   = 0.2;
static const double BodyH      = 0.45;
static const double MaxMotToq  = 50;


namespace Nabo{

	struct inputStruct{
		int cnt;
		double cmdVx, cmdVy, cmdWz;
		double supP[3], supV[3];
		double rpy[3], gyr[3], acc[3];
		double j[NMot], w[NMot], t[NMot];
		bool LogFlag = 0;
	};

	struct outputStruct{
		double j[NMot], w[NMot], t[NMot];
		
};

}//namespace
