/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#pragma once
#include "plan.h"
#include "cpg.h"
#include "estimator.h"
#include "curve.h"
#include "com_data.h"
#include "mpc.h"
#include "cbic.h"

namespace Plan
{

	class walkPlanClass final : public basePlanClass
	{
	public:
		walkPlanClass();
		bool run(const Nabo::inputStruct &inRef, Nabo::outputStruct &outRef) override final;
		void log(bool &inLogFlag) override final;

	private:
		Cpg::cpgClass &cpg = Cpg::cpgClass::instance();
		Est::estClass &est = Est::estClass::instance();
		Blc::mpcClass &mpc = Blc::mpcClass::instance();
		Blc::cbicClass &cbic = Blc::cbicClass::instance();
		Crv::swTrj3dClass swTrj3d[2];
		WBC::comDataStruct com;
		Blc::stateStruct stt;
		bool cbicFlag, cmdForceFlag;  // Flag标志,表示是否启用 整体惯量补偿 和是否使用控制命令
		double cpCoeffX, cpCoeffY, offsetX, offsetPitchP, offsetPitchD;  
		double swHeight;  // 摆动腿的高度

		mat3d cmdTipR[2];
		vec3d cmdTipP[2], stdTipP[2];
		vec3d cmdTipV3d[2];
		vec3d cmdTipF3d[2];


		struct  // 期望平滑轨迹结构体
		{
			vec3d rpy, p, v2S;			// p：W系，v：S系（坡面系，遥控指令）
			mat3d RxyS, RxyA2S, Rz, RS; // 地形、主动（地形之外机器人自身）、yaw、地形+yaw
			double wz;					// F与W系等价
			double deltaVx, deltaVy, deltaWz;
			void init()
			{
				p.setZero();
				v2S.setZero();
				rpy.setZero();
				RxyS.setIdentity();
				RxyA2S.setIdentity();
				Rz.setIdentity();
				wz = 0;
				deltaVx = 0;
				deltaVy = 0;
				deltaWz = 0;
			}
		} des; // desire：F系下

		struct  // 目标数据结构体
		{
			mat3d Rxy, R; // 地形+主动（B to W系）
			vec3d p, w, v, v2F;
			double zOff;
			void init()
			{
				Rxy.setIdentity();
				R.setIdentity();
				p.setZero();
				w.setZero();
				v.setZero();
				v2F.setZero();
				zOff = 0;
			}
		} tgt; // target：世界系下

		struct  // 限幅数据的结构体 
		{
			double vx = 1.0, vy = 0.2, wz = 0.5;
			double deltaVx, deltaVy, deltaWz;
			void init(double dt)
			{
				// 这里数值设置小一些，让它更缓慢的增长
				deltaVx = 0.05 * dt;
				deltaVy = 0.05 * dt;
				deltaWz = 0.05 * dt;
			}
		} lmt; // limit：限幅

		void init();
		void update();
		void plan();
		void balance();
		void dwdate(Nabo::outputStruct &outRef);
	};

} // namespace
