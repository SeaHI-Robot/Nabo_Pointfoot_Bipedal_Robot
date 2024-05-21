#include "robot.h"
#include <rbdl/rbdl.h>
#include "iopack.h"

namespace Rbt
{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	class rbtClass::impClass
	{
	public:
		Model model;
		unsigned int feetId[2];
		rbtClass &omp;
		double dt = 0.001; // 初始值会被rbt.setDt覆盖
		vec3d hip[2]{{0, -HipY, HipZ}, {0, HipY, HipZ}}; // HipY=0.1 HipZ=-0.1 在nabo_config中定义
		double maxL = LenThigh + LenShank - 0.001; // 最大腿长 为什么要减0.001？

		vecXd(-1) q, qd, qdLast, qdd, t; // 角度，加速度，上一次的角速度，角加速度，力矩
		vec3d ankleP[2];  // ankel即球形足
		mat3d ankleR[2];  // ankel即球形足
		vec3d ankleV3d[2];
		vec3d ankleF3d[2];
		matXd(-1, -1) Jcb3d;
		matXd(3, NLegDof) JcbAnkle3d[2];  // 3行，对应末端的三个位置xyz， NLegDof(3)列对应一条推三个关节

		InverseKinematicsConstraintSet ikSet;
		vec3d tgtAnkV3d[2];
		vec3d tgtAnkF3d[2];
		vecXd(-1) tgtQ, tgtQd, tgtQdLast, tgtQdd, tgtT; // 目标的 角度，加速度，上一次的角速度，角加速度，力矩

		vecXd(3) legKp, legKd;
		vecXd(NMot) jntKp, jntKd; // NMot=6, 在nabo_config中定义
		vecXd(NGenDof) dnmKp, dnmKd; // // NGenDof=12, 在nabo_config中定义


		impClass(rbtClass *omp);

		// 更新机器人的状态(关节位置、速度、力矩)
		void update(const double (&jj)[NMot], const double (&ww)[NMot], const double (&tt)[NMot]);
		
		// 执行逆运动学和动力学计算,更新目标状态
		void dwdate();
	};


	rbtClass::impClass::impClass(rbtClass *omp) : omp(*omp)
	{
		model.gravity = Vector3d(0, 0, -10.0);

		Body torso(10, Vector3d(0, 0, 0), Vector3d(0.08333333, 0.02966667, 0.09633333)), // 惯量由mujoco直接给出
		// For nabo_pointfoot normal_weight version:
			// hipRol(0.5, Vector3d(0, 0, 0), Vector3d(0.0001, 0.000067, 0.000067)),
			// thigh(1, Vector3d(0, 0, -LenThigh / 2), Vector3d(0.0024336, 0.0024336, 0.0001936)),
			// shank(1, Vector3d(0, 0, -LenShank / 2), Vector3d(0.00170036, 0.00170036, 0.00010929)),
		
		// For nabo_pointfoot light_weight version:
			hipRol(0.35, Vector3d(-0.03, 0, 0), Vector3d( 7.00000000e-05, 4.66666667e-05, 4.66666667e-05)),
			thigh(0.1, Vector3d(0, 0, -LenThigh / 2), Vector3d(2.4336e-04, 2.4336e-04, 1.9360e-05)),
			shank(0.1, Vector3d(0, 0, -LenShank / 2), Vector3d(1.70035714e-04, 1.70035714e-04, 1.09285714e-05)),
			
			ankle(0.1, Vector3d(0, 0, 0), Vector3d(1e-05, 5e-06, 1e-05));  // 圆柱脚
			// ankle(0.1, Vector3d(0, 0, 0), Vector3d(4.e-06, 4.e-06, 4.e-06));  // 球形脚，穿模严重
			
		Joint jx(JointTypeRevolute, Vector3d(1, 0, 0)),
			jy(JointTypeRevolute, Vector3d(0, 1, 0)),
			jfixed(JointTypeFixed),
			// SpatialVector定义顺序为【rx,ry,rz,x,y,z】，希望的顺序为【x,y,z,rz,ry,rx】
			floatBase(SpatialVector(0, 0, 0, 1, 0, 0),
					  SpatialVector(0, 0, 0, 0, 1, 0),
					  SpatialVector(0, 0, 0, 0, 0, 1),
					  SpatialVector(0, 0, 1, 0, 0, 0),
					  SpatialVector(0, 1, 0, 0, 0, 0),
					  SpatialVector(1, 0, 0, 0, 0, 0));

		unsigned int torsoId = model.AddBody(0, Xtrans(Vector3d(0, 0, 0)), floatBase, torso);

		
		// 右腿
		model.AddBody(torsoId, Xtrans(Vector3d(0, -HipY, HipZ)), jx, hipRol);
		model.AppendBody(Xtrans(Vector3d(0, 0, 0)), jy, thigh);
		model.AppendBody(Xtrans(Vector3d(0, 0, -LenThigh)), jy, shank);
		feetId[0] = model.AppendBody(Xtrans(Vector3d(0, 0, -LenShank)), jfixed, ankle);
		
		// 左腿
		model.AddBody(torsoId, Xtrans(Vector3d(0, HipY, HipZ)), jx, hipRol);
		model.AppendBody(Xtrans(Vector3d(0, 0, 0)), jy, thigh);
		model.AppendBody(Xtrans(Vector3d(0, 0, -LenThigh)), jy, shank);
		feetId[1] = model.AppendBody(Xtrans(Vector3d(0, 0, -LenShank)), jfixed, ankle);
		
        // 逆解，右、左脚的点足的位、姿，另外身体6维冗余，在B系中应固定身体位姿为0
		ikSet.AddFullConstraint(feetId[0], {0, 0, 0}, {0, -HipY, -maxL}, mat3d::Identity());
		ikSet.AddFullConstraint(feetId[1], {0, 0, 0}, {0, HipY, -maxL}, mat3d::Identity());
		ikSet.AddFullConstraint(torsoId, {0, 0, 0}, {0, 0, 0}, mat3d::Identity());

		q.setZero(NGenDof);
		qd.setZero(NGenDof);
		qdLast.setZero(NGenDof);
		qdd.setZero(NGenDof);
		t.setZero(NGenDof);
		tgtQ.setZero(NGenDof);
		tgtQd.setZero(NGenDof);
		tgtQdLast.setZero(NGenDof);
		tgtQdd.setZero(NGenDof);
		tgtT.setZero(NGenDof);
		Jcb3d.setZero(3, NGenDof);

		double kpP = ini["kpP"];
		double kdP = ini["kdP"];
		double kpJ = ini["kpJ"];
		double kdJ = ini["kdJ"];

		legKp <<  kpP, kpP, kpP;
		legKd <<  kdP, kdP, kdP;
		
		jntKp.setOnes();
		jntKd.setOnes();
		jntKp *= kpJ;
		jntKd *= kdJ;
		dnmKp.setZero();
		dnmKd.setZero();
		dnmKp.segment<NMot>(NBaseDof).setOnes();
		dnmKd.segment<NMot>(NBaseDof).setOnes();
		dnmKp *= 20;
		dnmKd *= 2;
	}


	void rbtClass::impClass::update(const double (&jj)[NMot], const double (&ww)[NMot], const double (&tt)[NMot])
	{
		// 1. 更新关节位置、速度
		memcpy(q.data() + NBaseDof, jj, MemMot);
		memcpy(qd.data() + NBaseDof, ww, MemMot);
		qdd = (qd - qdLast) / dt;
		memcpy(qdLast.data() + NBaseDof, ww, MemMot);
		memcpy(t.data() + NBaseDof, tt, MemMot);

		// 2. 使用RBDL计算正运动学
		UpdateKinematics(model, q, qd, qdd);

		// 3. 计算脚踝位置、姿态、速度等 
		vec3d body_point_position{0, 0, 0}; 
		ankleP[0] = CalcBodyToBaseCoordinates(model, q, feetId[0], body_point_position, 0);
		ankleP[1] = CalcBodyToBaseCoordinates(model, q, feetId[1], body_point_position, 0);

		ankleV3d[0] = CalcPointVelocity(model, q, qd, feetId[0], body_point_position, 0);
		ankleV3d[1] = CalcPointVelocity(model, q, qd, feetId[1], body_point_position, 0);

		// 4. 计算雅可比矩阵
		CalcPointJacobian(model, q, feetId[0], body_point_position, Jcb3d, 0);
		JcbAnkle3d[0] = Jcb3d.block(0, NBaseDof, 3, NLegDof);
		CalcPointJacobian(model, q, feetId[1], body_point_position, Jcb3d, 0);
		JcbAnkle3d[1] = Jcb3d.block(0, NBaseDof + NLegDof, 3, NLegDof);
	

		// InverseDynamics(model,q,qd,qdd,t);
		ankleF3d[0] = JcbAnkle3d[0].transpose().colPivHouseholderQr().solve(t.segment<NLegDof>(NBaseDof));
		ankleF3d[1] = JcbAnkle3d[1].transpose().colPivHouseholderQr().solve(t.segment<NLegDof>(NBaseDof + NLegDof));
	}


	void rbtClass::impClass::dwdate()
	{
		auto &tgtP = ikSet.target_positions;
		double tgtL = (tgtP[0] - hip[0]).norm();
		
		if (tgtL > maxL)
		{
			tgtP[0] = hip[0] + maxL / tgtL * (tgtP[0] - hip[0]);
		}
		tgtL = (tgtP[1] - hip[1]).norm();
		
		if (tgtL > maxL)
		{
			tgtP[1] = hip[1] + maxL / tgtL * (tgtP[1] - hip[1]);
		}

		InverseKinematics(model, q, ikSet, tgtQ);

		tgtQd.segment<NLegDof>(NBaseDof) = JcbAnkle3d[0].householderQr().solve(tgtAnkV3d[0]);
		tgtQd.segment<NLegDof>(NBaseDof + NLegDof) = JcbAnkle3d[1].householderQr().solve(tgtAnkV3d[1]);
		
		tgtQdd = (tgtQd - tgtQdLast) / dt * 0.8;								 // 前馈加速度
		tgtQdd += dnmKp.cwiseProduct(tgtQ - q) + dnmKd.cwiseProduct(tgtQd - qd); // 反馈闭环加速度

		For(NGenDof) { Alg::clip(tgtQdd[i], 200); }
		InverseDynamics(model, q, tgtQd, tgtQdd, tgtT);
		tgtQdLast = tgtQd;


		aAxis errA;
		vec3d err3d;

		err3d << tgtP[0] - ankleP[0];
		tgtAnkF3d[0] += legKp.cwiseProduct(err3d) + legKd.cwiseProduct(tgtAnkV3d[0] - ankleV3d[0]);

		err3d << tgtP[1] - ankleP[1];
		tgtAnkF3d[1] += legKp.cwiseProduct(err3d) + legKd.cwiseProduct(tgtAnkV3d[1] - ankleV3d[1]);

		tgtT.segment<NLegDof>(NBaseDof) += JcbAnkle3d[0].transpose() * tgtAnkF3d[0];
		tgtT.segment<NLegDof>(NBaseDof + NLegDof) += JcbAnkle3d[1].transpose() * tgtAnkF3d[1];
		// tgtT.setZero();

		memcpy(omp.tgtT, tgtT.data() + NBaseDof, MemMot);
		tgtT.segment<NMot>(NBaseDof) += jntKp.cwiseProduct(tgtQ.segment<NMot>(NBaseDof) - q.segment<NMot>(NBaseDof)) + jntKd.cwiseProduct(tgtQd.segment<NMot>(NBaseDof) - qd.segment<NMot>(NBaseDof));		

	}


	//=========================================
	rbtClass &rbtClass::instance()
	{
		static rbtClass singtn;
		return singtn;
	}


	rbtClass::rbtClass() : imp(*new impClass(this))
	{
		m = ini["mass"];
		inertia << 0.08333, 0, 0,    
		           0, 0.02967, 0,    
				   0, 0, 0.09633;
		// inertia.setIdentity();
	}


	void rbtClass::update(const double (&j)[NMot], const double (&w)[NMot], const double (&t)[NMot])
	{
		imp.update(j, w, t);
	}


	const vec3d &rbtClass::getAnkleP2B(int legId) { return imp.ankleP[legId]; }
	const vec3d &rbtClass::getAnkleV3d2B(int legId) { return imp.ankleV3d[legId]; }
	const vec3d &rbtClass::getAnkleF3d2B(int legId) { return imp.ankleF3d[legId]; } // F系下足尖受力，反leg.f


	void rbtClass::dwdate(double (&tOut)[NMot])
	{
		imp.dwdate();
		memcpy(tOut, imp.tgtT.data() + NBaseDof, MemMot);
	}


	void rbtClass::dwdate(double (&jOut)[NMot], double (&wOut)[NMot], double (&tOut)[NMot])
	{
		imp.dwdate();
		memcpy(jOut, imp.tgtQ.data() + NBaseDof, MemMot);
		memcpy(wOut, imp.tgtQd.data() + NBaseDof, MemMot);
		memcpy(tOut, imp.tgtT.data() + NBaseDof, MemMot);
	}


	void rbtClass::setDt(double dt)
	{
		imp.dt = dt;
	}
	// void rbtClass::setTip2B(vec6d tpP[2]){

	//}
	// void rbtClass::setTip2B(vec6d tpP[2],vec6d tpV[2]){

	//}
	void rbtClass::setTip2B( const vec3d (&tpP)[2], const vec3d (&tpV3d)[2], const vec3d (&tpF3d)[2])
	{
		imp.ikSet.target_positions[0] = tpP[0];				   // 右踝
		imp.ikSet.target_positions[1] = tpP[1];				   // 左踝
		imp.tgtAnkV3d[0] = tpV3d[0];
		imp.tgtAnkV3d[1] = tpV3d[1];
		imp.tgtAnkF3d[0] = tpF3d[0];
		imp.tgtAnkF3d[1] = tpF3d[1];
	}
} // namespace