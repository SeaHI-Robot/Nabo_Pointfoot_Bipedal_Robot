/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
plan_recover中的内容是让机器人静态时候去用pid follow各个joint的期望位置，
以让机器人达到预定的一组关节configuration

在000.ini中打开latewalk参数可以在仿真前一秒内由plan_recover接管
*/

#include "iopack.h"
#include "plan_recover.h"

namespace Plan
{

	rcPlanClass::rcPlanClass()
	{
		logFlag = ini["rcLogFlag"];
		consoleFlag = ini["rcConsoleFlag"];
		kp = ini["kpRc"];
		ki = ini["kiRc"];
		kd = ini["kdRc"];
	}
	
	bool rcPlanClass::run(const Nabo::inputStruct &inRef, Nabo::outputStruct &outRef)
	{
		in = inRef;
		if (in.cnt == 0)
		{
			baseInit();
			init();
		}
		plan();  // 在下面定义了；执行recovery的算法
		dwdate(outRef);  // 在下面定义了；给输入的OutRef（out的引用）套一层PID
		return quitFlag;
	}
	
	
	void rcPlanClass::init()
	{
		memcpy(j0.data(), in.j, MemMot);  // j0是输入进来的关节角
		j1 = { 0, -0.5, 1,   0 , -0.5, 1 };  // 目标关节角
		memset(errSum, 0, MemMot);
		tReach = 1;  // Reach到特定关节角的阶段的“时间”
		
		cout << "\nplan_recover initialized...\n";
	}
	

	void rcPlanClass::plan()
	{
		tim = in.cnt * dt;
		
		if (tim < tReach)
		{
			double pgs = tim / tReach;  // Progress in sitting phase
			double s = 0.5 - cos(pgs * Pi) * 0.5;  // 三角函数位置规划
			double sd = Pi * 0.5 * sin(pgs * Pi) / tReach;  //  三角函数速度规划
			For(NMot)
			{
				out.j[i] = j0[i] + s * (j1[i] - j0[i]);
				out.w[i] = sd * (j1[i] - j0[i]);
			}
		}
		else
		{
			quitFlag = 1;
		}
	}

	void rcPlanClass::dwdate(Nabo::outputStruct &outRef)
	{
		For(NMot)
		{
			double err{out.j[i] - in.j[i]};
			errSum[i] += err;
			errSum[i] *= 0.99;
			out.t[i] = kp * err + ki * errSum[i] + kd * (out.w[i] - in.w[i]);
			out.t[i] = tOutFil[i].filt(out.t[i]);
			Alg::clip(out.t[i], MaxMotToq);
		}

		outRef = out;
	}


	void rcPlanClass::log(bool &inLogFlag)  
	{
		if (logFlag && inLogFlag && in.cnt % logCnt == 0)
		{
			fout << "plan_recover\t" << tim << "\t";
			fout << "tgtJ\t";
			For(6) { fout << out.j[i] << "\t"; }
			fout << "actJ\t";
			For(6) { fout << in.j[i] << "\t"; }

			fout << "tgtW\t";
			For(6) { fout << out.w[i] << "\t"; }
			fout << "actW\t";
			For(6) { fout << in.w[i] << "\t"; }

			fout << "tgtT\t";
			For(6) { fout << out.t[i] << "\t"; }
			fout << "actT\t";
			For(6) { fout << in.t[i] << "\t"; }
			fout << endl;
		}

		if (consoleFlag && in.cnt % consoleCnt == 0)
		{
			cout << "plan_recover: tim=" << tim << endl;
		}
	}
} // namespace
