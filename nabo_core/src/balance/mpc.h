/*=========== ***doc description @ yyp*** ===========
线性mpc，F系
	Blc::mpcClass &mpc=Blc::mpcClass::instance();
	Blc::stateStruct stt;
=====================================================*/
#pragma once
#include "eigen.h"
#include "nabo_config.h"

namespace Blc
{
	struct stateStruct
	{					  // 全部为F系
		vec3d aDlt, pDlt; // A、p的相对差（act-tgt）
		vec3d w, v;
		vec3d leg0, leg1;
		double vx, vy, wz; // 命令值
		void setZero()
		{
			aDlt.setZero();
			pDlt.setZero();
			w.setZero();
			v.setZero();
			vx = 0;
			vy = 0;
			wz = 0;
			leg0 << 0, -HipY, -BodyH;
			leg1 << 0, HipY, -BodyH;
		}
	};
	//=========================
	class mpcClass
	{
	public:
		static mpcClass &instance();
		~mpcClass() { stop(); };
		bool start();
		bool stop();
		bool wake();
		bool sleep();
		void setCiParam(double mu); // 摩擦系数mu，旋转摩擦系数muz，脚面半长lx，脚面半宽ly
		void setState(stateStruct &stt);
		void getResult(vec3d &p, mat3d &R, vec3d &w, vec3d &v, vec3d &mf0, vec3d &mf1);
		void getMF(vec3d &mf0, vec3d &mf1); // F系，出力方向
		void test();
		void testRun();

	private:
		mpcClass();
		mpcClass(const mpcClass &) = delete;
		mpcClass &operator=(const mpcClass &) = delete;

		class impClass;
		impClass &imp;
	};
} // namespace