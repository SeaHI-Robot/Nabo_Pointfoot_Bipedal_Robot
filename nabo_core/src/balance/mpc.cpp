/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
站立腿单刚体mpc的主要代码
*/

#include "mpc.h"
#include "robot.h"
#include "cpg.h"
#include "eiquadprog/eiquadprog-fast.hpp"
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include "iopack.h"

// #define DefMpcTiming
// #define DefMpcTest


#ifdef DefMpcTest
#pragma message("mpc.cpp: test模式")
#endif

using namespace std;
namespace Blc
{
	const int xDim = 12, uDim = 6;
	const int Horizon = 10;
	const int Horizon12 = Horizon * 12;
    const int Horizon6 = Horizon * 6;
	const double maxFz = 200;
	// 在这里,impClass就是mpcClass的"implementation",
	// 即MPC算法实现的具体封装和隐藏在impClass内部。通过public接口与imp交互,从而调用和使用MPC算法,
	// 而不暴露实现细节。
	// 这种设计模式被称为桥接模式(Bridge Pattern)或者Pimpl惯用法(Pointer to implementation idiom),
	// 可以增加代码的可维护性和可扩展性。
	class mpcClass::impClass
	{
	public:
		impClass();
		void setCiParam(double mu);
		void loop();
		void runOnce();
		void test();
		Rbt::rbtClass &rbt = Rbt::rbtClass::instance();
		Cpg::cpgClass &cpg = Cpg::cpgClass::instance();
		double pcT, stT, g = 10;
		double mu;
		int stStage[Horizon * 2];
		double q[12], r[6];
		double cpX = ini["cpCoeffX"];
		double cpY = ini["cpCoeffY"];
		double offsetX = ini["offsetX"];
		double offsetPitchP = ini["offsetPitchP"];
		double offsetPitchD = ini["offsetPitchD"];

		stateStruct stt, stt0;
		double mass = 10; // 初始化质量 随便给的值
		mat3d Inertia0 = mat3d::Identity(), InertiaInv = mat3d::Identity();


		matXd(12, 6) B;
		vecXd(12) g0;  // MPC里的重力项
		vecXd(Horizon12) AG, xd;
		vecXd(Horizon6) h;
		vecXd(Horizon12) h_tmp;		  		 // 计算h的时候的暂存向量
		matXd(Horizon12, Horizon6) BB; 		// AA已简化到A*x0-tgt+BB*g
		matXd(Horizon6, Horizon6) H;
		matXd(Horizon12, Horizon6) H_tmp;	// 计算h的时候的暂存向量
		Eigen::VectorXd u;
		vecXd(6) mf;  // MPC的优化出来的作用力，原来NABO的MPC包含力矩（moment），为了少改动（减少出Bug的可能性）故没有做改动
		vecXd(12) x1;  // 

		matXd(0, Horizon12) CE;  // 等式约束矩阵
		matXd(0, 1) ce;  // 等式约束矩阵
		matXd(-1, -1) CI;  // 在后面专门对维度赋值 // 不等式约束矩阵
		vecXd(-1) ci;  // 不等式约束矩阵
		eiquadprog::solvers::EiquadprogFast qp;  // qp求解器

		bool isStarted{0}, isStop{1};
		bool isWait{0}, needSleep{1};
		mutex mut;
		condition_variable cv;
	};

	mpcClass::impClass::impClass()
	{
		isWait = 1;
		qp.reset(Horizon6, 0, Horizon * 10);
		u.resize(Horizon6);
		// CI.setZero(Horizon * 22, Horizon12);
		CI.setZero(Horizon * 10, Horizon12);
		// ci.setZero(Horizon * 22);
		ci.setZero(Horizon * 10);
		setCiParam(1);

		mass = rbt.getMass();  // MPC这里的mass其实是读取的ini中的质量
		Inertia0 = rbt.getInertia();  // MPC这里的inertia是在robot.cpp中给的定死的，直接用了机身的inertia
		InertiaInv = Inertia0.inverse();

		B.setZero();
		BB.setZero();
		g0.setZero();
		stt.setZero();
		stt0.setZero();

		x1.setZero();  // 计算0状态（当前状态）后第一个状态，也就是即将采用的输入后对应的状态
		mf.setZero();
		mf[2] = mass * 5;
		mf[5] = mass * 5; // 5=g/2

		ini.getArray("mpcQ", q);
		ini.getArray("mpcR", r);
	};
	
	void mpcClass::impClass::setCiParam(double mu)
	{
		Alg::clip(mu, 0, 10);
		
		this->mu = mu;
		
		// //== CI*x+ci>0，参考cbic.cpp拓，【但mpc的f方向是需求方向】
		For(Horizon)
		{
			int i6 = i * 6;
			int i10 = i * 10;
			// 右腿
			CI.block<2, 2>(i10, i6).setIdentity();
			CI.block<2, 1>(i10, i6 + 2) << mu, mu;
			CI.block<2, 2>(i10 + 2, i6).setIdentity();
			CI.block<2, 2>(i10 + 2, i6) *= -1;
			CI.block<3, 1>(i10 + 2, i6 + 2) << mu, mu, -1;
			// 左腿
			CI.block<2, 2>(i10 + 5, i6 + 3).setIdentity();
			CI.block<2, 1>(i10 + 5, i6 + 5) << mu, mu;
			CI.block<2, 2>(i10 + 7, i6 + 3).setIdentity();
			CI.block<2, 2>(i10 + 7, i6 + 3) *= -1;
			CI.block<3, 1>(i10 + 7, i6 + 5) << mu, mu, -1;
		}
		// fprintEi(CI);
	}

	void mpcClass::impClass::loop()
	{
		auto refClock = chrono::high_resolution_clock::now();
		int period = 10000; // 微秒
		auto tag0 = refClock, tag1 = refClock;
		while (isStarted)
		{
			while (needSleep)
			{
				this_thread::sleep_for(chrono::microseconds(period)); // 微秒
				refClock = chrono::high_resolution_clock::now();
			}
			refClock += chrono::microseconds(period); // 微秒

#ifdef DefMpcTiming
			tag0 = chrono::high_resolution_clock::now();
#endif

			unique_lock<mutex> lock(mut);
			isWait = 1;
			while (isWait)
			{
				cv.wait(lock);
			}
			stt = stt0;
			lock.unlock();

#ifdef DefMpcTiming
			tag1 = chrono::high_resolution_clock::now();
#endif

			runOnce();
//==平均耗时计算（20个点）==
#ifdef DefMpcTiming
			static double rcd0[20]{}, average0 = 0, rcd1[20]{}, average1 = 0;
			static int idx = 0;
			auto dddt0 = chrono::duration_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now() - tag0);
			auto dddt1 = chrono::duration_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now() - tag1);
			rcd0[idx] = dddt0.count() / 1e6;
			rcd1[idx] = dddt1.count() / 1e6;
			average0 += rcd0[idx] / 20;
			average1 += rcd1[idx] / 20;
			idx++;
			idx %= 20;
			average0 -= rcd0[idx] / 20;
			average1 -= rcd1[idx] / 20;
			if (idx == 0)
			{
				cout << "mpc循环平均耗时(ms)=" << average0 << "，计算平均耗时(ms)=" << average1 << endl;
			}
#endif
			//=================
			// this_thread::sleep_until(refClock);
		}
		isStop = 1;
	}

	void mpcClass::impClass::runOnce()
	{
		pcT = cpg.getPeriod() / Horizon;
		stT = cpg.getStT(0);
		For(Horizon)
		{
			double t = pcT * i;
			int i2{i * 2};
			cpg.isFutureSt(t, stStage + i2);
			
			if (stStage[i2])
			{
				ci[i * 10 + 2] = maxFz;
			}
			else
			{
				ci[i * 10 + 2] = 0;
			}
			if (stStage[i2 + 1])
			{
				ci[i * 10 + 5] = maxFz;
			}
			else
			{
				ci[i * 10 + 5] = 0;
			}
		}
		g0[5] = -10 * pcT * pcT * 0.5;
		g0[11] = -10 * pcT;

		AG.head<12>() << stt.aDlt + stt.w * pcT,    stt.pDlt + stt.v * pcT,    stt.w,    stt.v;
		AG.head<12>() += g0;
		xd.head<12>() << stt.w * pcT,    stt.v * pcT,    stt.w,    stt.v;
		for (int i = 1; i < Horizon; i++)
		{
			// A*G_i = A*A*G_(i-1) + g
			int i12{i * 12};
			AG.segment<12>(i12) = AG.segment<12>(i12 - 12);
			AG.segment<6>(i12) += AG.segment<6>(i12 + 6) * pcT;
			AG.segment<12>(i12) += g0;
			xd.segment<12>(i12) = xd.segment<12>(i12 - 12);
			xd.segment<6>(i12) += xd.segment<6>(i12 + 6) * pcT;
		}

		//========================
		vec3d tip[2]{stt.leg0, stt.leg1};  // tip储存了两个腿的脚尖到Base的三维坐标
		// B角速度项
		B.block<3, 3>(6, 0) = InertiaInv * Ei::skew(tip[0]) * pcT;
		B.block<3, 3>(6, 3) = InertiaInv * Ei::skew(tip[1]) * pcT;
		// B角度项
		B.topRows<3>() = B.middleRows<3>(6) * pcT * 0.5;
		// B速度项
		double tmp = pcT / mass;
		B(9, 0) = tmp;
		B(10, 1) = tmp;
		B(11, 2) = tmp;
		B(9, 3) = tmp;
		B(10, 4) = tmp;
		B(11, 5) = tmp;
				// B位置项
		tmp = 0.5 * pcT * tmp;
		B(3, 0) = tmp;
		B(4, 1) = tmp;
		B(5, 2) = tmp;
		B(3, 3) = tmp;
		B(4, 4) = tmp;
		B(5, 5) = tmp;
		
		BB.topLeftCorner<12, 6>() = B;
		for (int i = 1; i < Horizon; i++)
		{
			int i12{i * 12};
			int i6{i * 6};
			mat2d Rz2d = Ei::rot2R2d(stt.wz * pcT * i);
			vec2d dXY = Rz2d * vec2d(stt.vx, stt.vy) * pcT;
			for (int l = 0; l < 2; l++)
			{
				if (stStage[i * 2 + l] == 2 && stStage[i * 2 + l - 2] == 0)
				{ // 由sw首次落地预测落脚点
				    
					vec2d tmp( stt.vx * stT *1.05 + cpX * (stt.v[0] - stt.vx) + offsetX + offsetPitchP*stt.aDlt[1] + offsetPitchD*stt.w[1], stt.vy * stT *1.05 + 0.1 * (2 * l - 1) + cpY * (stt.v[1] - stt.vy) );
					// vec2d tmp( stt.vx * stT  + cpX * (stt.v[0] - stt.vx) , stt.vy * stT  + 0.1 * (2 * l - 1) + cpY * (stt.v[1] - stt.vy) );
					// vec2d tmp( stt.vx * stT / 2  , stt.vy * stT / 2 + 0.1 * (2 * l - 1)  );
					tmp = Rz2d * tmp;
					tip[l][0] = tmp[0]; // 运动学预测是在当前F系，但动力学发生在未来的F系，故无需添加body位移
					tip[l][1] = tmp[1];
				}
				else
				{
					tip[l][0] -= dXY[0];
					tip[l][1] -= dXY[1];
				}
				mat3d tmp = InertiaInv * Ei::skew(tip[l]) * pcT;
				B.block<3, 3>(6, l * 3) = tmp;
				B.block<3, 3>(0, l * 3) = tmp * pcT * 0.5;
			}
			BB.block<12, 6>(i12, i6) = B;
			for (int j = 0; j < i; j++)
			{
				BB.block<6, 6>(i12 + 6, j * 6) = BB.block<6, 6>(i12 - 6, j * 6);
				BB.block<6, 6>(i12, j * 6) = BB.block<6, 6>(i12 - 12, j * 6) + BB.block<6, 6>(i12 + 6, j * 6) * pcT;
			}
		}

		h_tmp = AG - xd;  // h_tmp ~ 12
		For(Horizon12)   // 这里的维度操作有问题
		{
			double qi = q[i % 12];
			H_tmp.row(i) = BB.row(i) * qi; // 实际的Q是ini中q的平方
			h_tmp[i] *= qi * qi;
		}
		H = H_tmp.transpose() * H_tmp;
		h = BB.transpose() * h_tmp;
		For(Horizon6)
		{
			H(i, i) += r[i % 6]; // R权重
		}
		For(Horizon)
		{
			int i2{i * 2}, i6{i * 6};
			if (stStage[i2] & stStage[i2 + 1])
			{
				h[i6 + 2] -= r[2] * mass * 5; // 5=g/2
				h[i6 + 5] -= r[5] * mass * 5; // 5=g/2
			}
			else
			{
				if (stStage[i2])
				{
					h[i6 + 2] -= r[2] * mass * 10;
				}
				else if (stStage[i2 + 1])
				{
					h[i6 + 5] -= r[5] * mass * 10;
				}
			}
		}
		qp.solve_quadprog(H, h, CE, ce, CI, ci, u);
		memcpy(mf.data(), u.data(), 48);
		x1 = AG.head<12>() + BB.topLeftCorner<12, 6>() * mf;  // 计算0状态（当前状态）后第一个状态，也就是即将采用的输入后对应的状态
	}
	
	//============================================================================================
	mpcClass &mpcClass::instance()
	{
		static mpcClass singleton;
		return singleton;
	}

	mpcClass::mpcClass() : imp(*new impClass()) {}
	
	bool mpcClass::start()
	{
		if (!imp.isStarted & imp.isStop)
		{
			imp.isStarted = 1;
			thread thd(&mpcClass::impClass::loop, &(this->imp)); // 因为loop不是静态，写成这样就可以了
			thd.detach();
			imp.isStop = 0;
			return 1;
		}
		return 0;
	}

	bool mpcClass::stop()
	{
		imp.isStarted = 0;
		return 1;
	}

	bool mpcClass::wake()
	{
		imp.needSleep = 0;
		if (imp.isStarted)
		{
			return 1;
		}
		return 0;
	}

	bool mpcClass::sleep()
	{
		imp.needSleep = 1;
		if (imp.isStarted)
		{
			return 1;
		}
		return 0;
	}
	
	void mpcClass::setCiParam(double mu)
	{
		imp.setCiParam(mu);
	}

	void mpcClass::setState(stateStruct &stt)
	{
		lock_guard<mutex> lock(imp.mut);
		if (imp.isWait)
		{
			imp.stt0 = stt;
			imp.isWait = 0;
			imp.cv.notify_one();
		}
#ifdef DefMpcTest
		imp.stt = stt;
#endif
	}

	void mpcClass::getResult(vec3d &p, mat3d &R, vec3d &w, vec3d &v, vec3d &mf0, vec3d &mf1)
	{
		R = Ei::rpy2R(imp.x1.segment<3>(0));
		p = imp.x1.segment<3>(3);
		w = imp.x1.segment<3>(6);
		v = imp.x1.segment<3>(9);
		mf0 = -imp.mf.head<3>();
		mf1 = -imp.mf.tail<3>();
	}

	void mpcClass::getMF(vec3d &mf0, vec3d &mf1)
	{
		mf0 = -imp.mf.head<3>();
		mf1 = -imp.mf.tail<3>();
	}

	//==打印测试=============================
#ifndef DefMpcTest
	void mpcClass::test() {}
	void mpcClass::testRun() {}
#else
	void mpcClass::testRun() { imp.runOnce(); }
	void mpcClass::test() { imp.test(); }
	void mpcClass::impClass::test()
	{
		runOnce();
		cout << "mpc: test 将写入 zzz.txt\n";
		fout << "=======* mpc test *=======\nstage:\n";
		For(Horizon)
		{
			fout << stStage[i * 2] << ", ";
		}
		fout << endl;
		For(Horizon)
		{
			fout << stStage[i * 2 + 1] << ", ";
		}
		// fout<<"\nci:\n";
		// For(Horizon){
		// 	fprintEi(ci.segment<22>(i*22));
		// }
		fout << "\nu:\n";
		For(Horizon)
		{
			fprintEi(u.segment<12>(i * 12));
		}
		fout << "\nAG:\n";
		For(Horizon)
		{
			fprintEi(AG.segment<12>(i * 12));
		}
		vecXd(Horizon12) x = BB * u;
		fout << "\nBB*u:\n";
		For(Horizon)
		{
			fprintEi(x.segment<12>(i * 12));
		}
		x += AG;
		fout << "\nx:(A,p,w,v)\n";
		For(Horizon)
		{
			fprintEi(x.segment<12>(i * 12));
		}
		fout << "\ncost=";
		auto cost1 = u.transpose() * H * u; // +u.transpose()*h;
		auto cost2 = u.transpose() * h;
		fout << cost1 << ", " << cost2 << "\n合力: ";
		vec3d xyz(0, 0, 0);
		For(2)
		{
			xyz += u.segment<3>(i * 6 + 3);
		}
		xyz -= mass * vec3d(0, 0, 10);
		fprintEi(xyz);
		fprintEi(xyz / mass);
	}
#endif // DefMpcTest
} // namespace


/*============ nabo点足 MPC CI矩阵 =============== 
   1    0  -mu    0    0    0   
   0    1  -mu    0    0    0  
  -1    0  -mu    0    0    0   
   0   -1  -mu    0    0    0   
   0    0   -1    0    0    0    
   0    0    0    1    0  -mu   
   0    0    0    0    1  -mu    
   0    0    0   -1    0  -mu    
   0    0    0    0   -1  -mu   
   0    0    0    0    0   -1   
========================================*/