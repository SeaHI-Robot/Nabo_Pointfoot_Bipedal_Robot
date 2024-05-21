#include "plan_walk.h"

namespace Plan
{

	walkPlanClass::walkPlanClass()
	{
		logFlag = ini["wkLogFlag"];
		consoleFlag = ini["wkConsoleFlag"];
		cpCoeffX = ini["cpCoeffX"];
		cpCoeffY = ini["cpCoeffY"];
		offsetX = ini["offsetX"];
		offsetPitchP = ini["offsetPitchP"];
		offsetPitchD = ini["offsetPitchD"];
		cbicFlag = ini["cbicFlag"];
		cmdForceFlag = ini["cmdForceFlag"];
		swHeight = ini["swHeight"];
	}

	bool walkPlanClass::run(const Nabo::inputStruct &inRef, Nabo::outputStruct &outRef)
	{
		in = inRef;
		if (in.cnt == 0)
		{
			baseInit(); // 基类初始化
			update();	// 确实多一次 // 更新一次(确实多一次)
			init();		// 初始化
		}

		update();		// 更新状态
		plan();			// 规划足端轨迹
		balance();		// 计算平衡的足端力
		dwdate(outRef); // 更新输出

		return quitFlag; // 返回是否退出的标志
	}

	void walkPlanClass::init()
	{
		cmdTipP[0] = rbt.getAnkleP2B(0);	 // 获取初始左脚端位置
		cmdTipP[1] = rbt.getAnkleP2B(1);	 // 获取初始右脚端位置
		cmdTipV3d[0] = rbt.getAnkleV3d2B(0); // 获取初始左脚端速度
		cmdTipV3d[1] = rbt.getAnkleV3d2B(1); // 获取初始右脚端速度
		cmdTipF3d[0] << 0, 0, -56.5;			 // 初始化左脚端力, 机器人总质量为11.3
		cmdTipF3d[1] << 0, 0, -56.5;			 // 初始化右脚端力, 机器人总质量为11.3
		stdTipP[0] << 0, -HipY, -BodyH;		 // 初始化左侧标准脚端位置
		stdTipP[1] << 0, HipY, -BodyH;		 // 初始化右侧标准脚端位置
		swTrj3d[0].setP0(cmdTipP[0]);		 // 设置左侧起始姿态和位置
		swTrj3d[0].setP1(cmdTipP[0]);		 // 设置左侧目标姿态和位置
		swTrj3d[0].update(0);				 // 更新左侧轨迹
		swTrj3d[1].setP0(cmdTipP[1]);		 // 设置右侧起始姿态和位置
		swTrj3d[1].setP1(cmdTipP[1]);		 // 设置右侧目标姿态和位置
		swTrj3d[1].update(0);				 // 更新右侧轨迹

		des.init();	  // 初始化期望平滑轨迹结构体
		tgt.init();	  // 初始化目标数据结构体
		lmt.init(dt); // 初始化限幅数据结构体

		double duty[2]{0.5, 0.5};				   // 摆动占空比
		double offset[2]{0., 0.5};				   // 相位差
		cpg.init(dt, ini["period"], duty, offset); // 初始化CPG

		
		if (ini["cpg"]) // 如果配置文件中启用CPG
		{
			cpg.unlock(); // 解锁CPG
			cout << "\ncpg started...\n";
		}

		est.init(dt);		// 初始化状态估计器
		cbic.init(dt);		// 初始化整体惯量补偿
		cbic.setCiParam(1); // 设置整体惯量补偿参数
		com.setZero();		// 清零质心数据
		mpc.setCiParam(1);	// 设置模型预测控制参数
		mpc.start();		// 启动模型预测控制
		mpc.wake();			// 唤醒模型预测控制
		cout << "\nplan_walk initialized...\n\n";

	}


	void walkPlanClass::update()
	{
		tim = in.cnt * dt; // 按照cnt计算当前时间

		rbt.imu.update(in.rpy, in.rpy[2], in.gyr, in.acc); // 更新IMU数据
		rbt.update(in.j, in.w, in.t);					   // 更新机器人状态

		cpg.update(); // 更新CPG

		est.p << in.supP[0], in.supP[1], in.supP[2] - BodyH; // 更新位置估计（mujoco里直接拿数据），原点在机身的floatbase处
		est.v << in.supV[0], in.supV[1], in.supV[2];		 // 更新速度估计
		est.v2F = rbt.imu.Rz.transpose() * est.v;			 // 计算F系速度

		Alg::clip(in.cmdVx, lmt.vx);  // 限幅x方向速度命令
		Alg::clip(in.cmdVy, lmt.vy);  // 限幅y方向速度命令
		Alg::clip(in.cmdWz, lmt.wz);  // 限幅角速度命令

		des.deltaVx = (in.cmdVx - des.v2S[0]) * dt; // 计算x方向速度变化量
		des.deltaVy = (in.cmdVy - des.v2S[1]) * dt; // 计算y方向速度变化量
		des.deltaWz = (in.cmdWz - des.wz) * dt;		// 计算角速度变化量

		Alg::clip(des.deltaVx, lmt.deltaVx); // 限幅x方向速度变化量
		Alg::clip(des.deltaVy, lmt.deltaVy); // 限幅y方向速度变化量
		Alg::clip(des.deltaWz, lmt.deltaWz); // 限幅角速度变化量

		Alg::cmd2out1step(in.cmdVx, des.v2S[0], des.deltaVx); // 平滑x方向速度指令
		Alg::cmd2out1step(in.cmdVy, des.v2S[1], des.deltaVy); // 平滑y方向速度指令
		Alg::cmd2out1step(in.cmdWz, des.wz, des.deltaWz);	  // 平滑角速度指令

		// des我的理解是过度结构体变量，我觉得原作者这里写的有点冗余，暂时我没有去修改
		des.RxyS = aAxisY(est.slope[1]) * aAxisX(est.slope[0]); // 计算地形旋转矩阵(基于估计的斜坡) 平地slope都是0 因此des.RxyS是单位矩阵
		des.RxyA2S = aAxisY(des.rpy[1]) * aAxisX(des.rpy[0]);	// 计算主动(非地形)旋转矩阵 因为期望机身torso的roll和pitch都是0 因此des.RxyA2S是单位矩阵
		des.rpy[2] += des.wz * dt;								// 更新期望yaw角
		des.Rz = aAxisZ(des.rpy[2]);							// 计算期望yaw旋转矩阵
		des.RS = des.Rz * des.RxyS;								// 计算期望旋转矩阵(地形+yaw)
		des.p += des.RS * des.v2S * dt;							// 更新期望位置(积分速度)

		// tgt描述的应该是在"cmd系”下, 即原点根据cmdVx, cmdVy, cmdWz改变的系
		tgt.Rxy = des.RxyA2S * des.RxyS; // 计算目标旋转矩阵(地形+主动)  // 主动(pit、rol) + 地形  // 这一项在平地上是单位矩阵
		tgt.R = des.Rz * tgt.Rxy;		 // 计算目标旋转矩阵(地形+主动+yaw)  // 在平地上 tgt.R = des.Rz
		tgt.v2F = des.RxyS * des.v2S;	 // 计算F系下的目标速度   // 在平地上 des.RxyS是单位矩阵， tgt.v2F = des.v2S
		tgt.v = des.RS * des.v2S;		 // 计算目标系速度   
		tgt.w << 0, 0, des.wz;			 // 设置目标角速度
		tgt.p = des.p;					 // 设置目标位置
	}


	void walkPlanClass::plan()
	{
		auto &imu = rbt.imu;		// 获取IMU引用
		double stT = cpg.getStT(0); // 获取第一条腿的支撑时间, 一般情况下应该站立时间等于摆动时间
		For(2)
		{
			double swPgs = cpg.getSwPgs(i); // 获取当前摆动周期进度
			
			if (swPgs > 0)					
			{   // ===== 如果当前在摆动阶段 =====
				if (!rbt.isSwing(i))	   // 如果当前未设置为摆动状态
				{						   // 每迈步初始
					swTrj3d[i].setP0now(); // 设置当前为起始姿态和位置
					swTrj3d[i].setH(swHeight); // 设置最大高度
					rbt.setSwing(i, 1);	   // 设置为摆动状态
				}

				mat3d Rdz(aAxisZ(tgt.w[2] * (1.05 * stT + cpg.getSwTicRm(i)))); // 计算yaw旋转矩阵(用于补偿摆动过程中的yaw变化) // 这里的heuristic是raibert heuristic的二倍更为合适
				// mat3d Rdz(aAxisZ(tgt.w[2] * (cpg.getSwTicRm(i)))); // 计算yaw旋转矩阵(用于补偿摆动过程中的yaw变化)
				// mat3d R1(Rdz * des.RS);										   // 计算目标旋转矩阵(地形+yaw+摆动yaw补偿)

				// vec3d pBody(pRate * est.p + (1 - pRate) * tgt.p); // 计算参考位置(估计位置和目标位置的加权平均)
				vec3d pBody(est.p);	 // 计算参考位置(估计位置和目标位置的加权平均)
				pBody[2] = tgt.p[2]; // 使用目标高度
				// vec3d vBody(vRate * est.v + (1 - vRate) * tgt.v);  // 计算参考速度(估计速度和目标速度的加权平均)
				vec3d vBody(est.v);																					 // 计算参考速度(估计速度和目标速度的加权平均)
				vBody[2] = tgt.v[2];																				 // 使用目标垂直速度
				vec3d hip(0, stdTipP[i][1], 0), leg(0, 0, stdTipP[i][2] - tgt.zOff);	// 相对于身体的臀 相对于臀的腿 的位置

				vec3d cp_term(cpCoeffX * (est.v[0] - tgt.v[0]), cpCoeffY * (est.v[1] - tgt.v[1]), 0);
				vec3d offset(offsetX + offsetPitchP*stt.aDlt[1] + offsetPitchD*stt.w[1], 0, 0);
											 
				vec3d p1 = pBody + Rdz * (vBody * (cpg.getSwTicRm(i) + 1.05 * stT) + cp_term + des.Rz * (tgt.Rxy * hip + leg) + offset); // 计算目标脚端位置 // 平地上tgt.Rxy是单位矩阵 // 原作者写的Rdz的一项可能可以理解为对旋转的heuristic项 // 这里的heuristic是raibert heuristic的二倍更为合适 
				// vec3d p1 = pBody + Rdz * (vBody * (cpg.getSwTicRm(i) + 0.5 * stT) + cp_term + des.Rz * (tgt.Rxy * hip + leg) + offset); // 计算目标脚端位置 // 平地上tgt.Rxy是单位矩阵 // 原作者写的Rdz的一项可能可以理解为对旋转的heuristic项 // 这里的heuristic是raibert heuristic的二倍更为合适 
		
				
				swTrj3d[i].setP1(p1, swPgs); // 设置目标姿态和位置
				swTrj3d[i].update(swPgs);	 // 更新轨迹

				swTrj3d[i].getV(cmdTipV3d[i]);
				cmdTipV3d[i] /= cpg.getSwT(i); // 将速度转换为单位时间内的增量
			}
			else  // ===== 如果当前不在摆动阶段 =====
			{
				if (rbt.isSwing(i)) // 如果当前设置为摆动状态
				{
					rbt.setSwing(i, 0); // 重置为非摆动状态
				}
				cmdTipV3d[i].setZero(); // 清零脚端速度, cmdRTipV3d在世界系下，理应和地面相对静止；
			}

			// cmdTipP， cmdTipV3d, cmdTipF3d都在世界系下
			swTrj3d[i].getP(cmdTipP[i]); // 获取脚端位置
			
			cmdTipP[i] = tgt.R.transpose() * (cmdTipP[i] - tgt.p);	// 将位置转换到目标坐标系
			
			cmdTipV3d[i] = tgt.R.transpose() * (cmdTipV3d[i] - tgt.v - tgt.w.cross(tgt.R * rbt.getAnkleP2B(i)));  // 将线速度转换到目标坐标系
		}
	}


	void walkPlanClass::balance()
	{
		auto &imu = rbt.imu;				  // 取IMU引用
		mat3d RzT = imu.Rz.transpose();		  // 计算世界到base旋转矩阵的转置
		aAxis Tmp(tgt.R * imu.R.transpose()); // W系 // 计算目标和实际姿态差异

		vec3d errA(Tmp.angle() * Tmp.axis()); // 计算姿态误差
		errA = RzT * errA;					  // 转到F系

		vec3d errP{ RzT * (tgt.p - est.p) }; // 计算位置误差

		vec3d errW(-imu.w);	   // 计算角速度误差
		// vec3d errW(-in.gyr[0], -in.gyr[1], -in.gyr[2]);	   // 计算角速度误差
		errW[2] += tgt.w[2];   // 添加期望yaw角速度
		errW = imu.Rxy * errW; // 将角速度误差转换到斜坡

		vec3d errV(tgt.v2F - est.v2F); // 计算速度误差(F坐标系)

		com.pid(errA, errP, errW, errV); // 通过PID根据 rpy，位置, rpy角速度， 速度的误差 计算期望加速度
		// com.acc[0] -= 4;  // 调整x方向期望加速度 减4是个什么操作
		com.filt(); // 滤波加速度

		stt.aDlt = -errA;		 // 设置姿态误差
		stt.pDlt = -errP;		 // 设置位置误差
		stt.w = imu.Rxy * imu.w; // 设置角速度(斜坡坐标系)
		// stt.w <<  in.gyr[0] , in.gyr[1] , in.gyr[2]; // 设置角速度(斜坡坐标系)
		stt.v = est.v2F;		 // 设置速度(斜坡坐标系)
		stt.wz = tgt.w[2];		 // 设置期望yaw角速度
		stt.vx = tgt.v2F[0];	 // 设置x方向期望速度
		stt.vy = tgt.v2F[1];	 // 设置y方向期望速度
		stt.leg0 = cmdTipP[0];	 // 设置左脚端位置
		stt.leg1 = cmdTipP[1];	 // 设置右脚端位置

		mpc.setState(stt); // 设置模型预测控制状态

		mpc.getMF(cmdTipF3d[0], cmdTipF3d[1]); // 计算脚端控制力

		if (cbicFlag) // 如果启用整体惯量补偿
		{
			cbic.setFr(cmdTipF3d);		 // 设置脚端参考力
			cbic.run(com.aAcc, com.acc); // 计算整体惯量补偿
			// vec3d Aacc;
			// vec3d acc;
			// Aacc << in.Aacc[0], in.Aacc[1], in.Aacc[2];
			// acc  << in.acc[0], in.acc[1], in.acc[2];
			// cbic.run(Aacc, acc); // 计算整体惯量补偿
			cmdTipF3d[0] = cbic.mf[0];	 // 获得WBC处理过后的左脚端控制力
			cmdTipF3d[1] = cbic.mf[1];	 // 获得WBC处理过后的右脚端控制力
		}

		mat3d RxyT = tgt.Rxy.transpose();	// 计算斜坡到世界坐标系旋转矩阵的转置
		cmdTipF3d[0] = RxyT * cmdTipF3d[0]; // 将左脚端控制力转换到世界坐标系
		cmdTipF3d[1] = RxyT * cmdTipF3d[1]; // 将右脚端控制力转换到世界坐标系
	}

	void walkPlanClass::dwdate(Nabo::outputStruct &outRef)
	{
		if (!cmdForceFlag) // 如果不使用优化的末端力
		{
			cmdTipF3d[0].setZero(); // 清零左脚端控制力
			cmdTipF3d[1].setZero(); // 清零右脚端控制力
		}

		rbt.setTip2B(cmdTipP, cmdTipV3d, cmdTipF3d); // 设置脚端期望姿态、位置、速度和力
		rbt.dwdate(out.j, out.w, out.t);			 // 获得输出
		For(NMot)
		{
			out.t[i] = tOutFil[i].filt(out.t[i]); // 滤波关节力矩
			Alg::clip(out.t[i], MaxMotToq);		  // 限幅关节力矩
		}
		outRef = out;
	}

	void walkPlanClass::log(bool &inLogFlag)
	{
		if (logFlag && inLogFlag && in.cnt % logCnt == 0)
		{
			vec3d actTipP[2]{rbt.getAnkleP2B(0), rbt.getAnkleP2B(1)};

			fout << "wk_cnt\t" << tim << "\t";

			fout << "errA\t" << stt.aDlt[0] << "\t" << stt.aDlt[1] << "\t" << stt.aDlt[2] << "\t";
			
			fout << "errP\t" << stt.pDlt[0] << "\t" << stt.pDlt[1] << "\t" << stt.pDlt[2] << "\t";

			fout << "w\t" << stt.w[0] << "\t" << stt.w[1] << "\t" << stt.w[2] << "\t";

			fout << "v\t" << stt.v[0] << "\t" << stt.v[1] << "\t" << stt.v[2] << "\t";

			fout << "cmd\t" << in.cmdVx << "\t" << in.cmdVy << "\t" << in.cmdWz << "\t";



			fout << "tgtQ\t";
			For6 { fout << out.j[i] << "\t"; }
			fout << "actQ\t";
			For6 { fout << in.j[i] << "\t"; }

			// fout << "cmdTipP\t";
			// For3 { fout << cmdTipP[0][i] << "\t"; }
			// For3 { fout << cmdTipP[1][i] << "\t"; }
			// fout << "actTipP\t";
			// For3 { fout << actTipP[0][i] << "\t"; }
			// For3 { fout << actTipP[1][i] << "\t"; }

			// fout<<"cmdTipV\t";
			// For3{fout<<cmdTipV3d[0][i]<<"\t";}
			// For3{fout<<cmdTipV3d[1][i]<<"\t";}

			// fout << "cmdTipF\t";
			// For3 { fout << cmdTipF3d[0][i] << "\t"; }
			// For3 { fout << cmdTipF3d[1][i] << "\t"; }
			
			// fout << "actTipF3d\t";
			// For3 { fout << rbt.getAnkleF3d2B(0)[i] << "\t"; }
			// For3 { fout << rbt.getAnkleF3d2B(1)[i] << "\t"; }

			// fout << "swPgs\t" << cpg.getSwPgs(0) << "\t" << cpg.getSwPgs(1) << "\t";

			// auto &imu = rbt.imu;
			// fout << "rpy\t";
			// For3 { fout << imu.rpy[i] << "\t"; }
			// fout << imu.yawAct << "\t";
			// fout << "w\t";
			// For3 { fout << imu.w[i] << "\t"; }
			// fout << "est\t";
			// For3 { fout << est.p[i] << "\t"; }
			// For3 { fout << est.v2F[i] << "\t"; }

			// fout<<"mpc-stt\t";
			// For(3){fout<<stt.aDlt[i]<<"\t";}
			// For(3){fout<<stt.pDlt[i]<<"\t";}
			// For(3){fout<<stt.w[i]<<"\t";}
			// For(3){fout<<stt.v[i]<<"\t";}

			// fout << "cbcAcc\t";
			// For3 { fout << com.aAcc[i] << "\t"; }
			// For3 { fout << com.acc[i] << "\t"; }

			// fout<<"cmdJ\t";
			// For(6){fout<<out.j[i]<<"\t";}
			// fout<<"actJ\t";
			// For(6){fout<<in.j[i]<<"\t";}
			// fout<<"cmdW\t";
			// For(6){fout<<out.w[i]<<"\t";}
			// fout<<"actW\t";
			// For(6){fout<<in.w[i]<<"\t";}
			// fout << "cmdT\t";
			// For(6) { fout << out.t[i] << "\t"; }
			// fout<<"tmpT\t";
			// For(6){fout<<rbt.tgtT[i]<<"\t";}
			// fout<<"actT\t";
			// For(6){fout<<in.t[i]<<"\t";}
			
			fout << endl;
		}

		if (consoleFlag && in.cnt % consoleCnt == 0) // 控制台输出，输出频率在plan.cpp中调整
		{
			cout << "------\ncnt=" << in.cnt << ", time=" << tim << endl;
		
		// 打印用户指令 
			cout << "cmd: vx=" << in.cmdVx << ", vy=" << in.cmdVy << ", wz=" << in.cmdWz << endl;
		
		// 打印状态量
			cout << "errA: " << stt.aDlt[0] << ", " << stt.aDlt[1] << ", " << stt.aDlt[2] << endl;
			cout << "errP: " << stt.pDlt[0] << ", " << stt.pDlt[1] << ", " << stt.pDlt[2] << endl;
			cout << "  w : " << stt.w[0] << ", " << stt.w[1] << ", " << stt.w[2] << endl;
			cout << "  v : " << stt.v[0] << ", " << stt.v[1] << ", " << stt.v[2] << endl;


		// 打印floatbase的位置和速度
			// cout << "in.supP, in.supV: " << in.supP[0] << ", " << in.supP[1] << ", " << in.supP[2] << ";   " << in.supV[0] << ", " << in.supV[1] << ", " << in.supV[2] << endl;

		// 打印pitch角度
		    // cout << "pitch: " << in.gyr[1] << endl;
		
		// 打印floatbase的rpy角速度和加速度
			// cout << "in.gyr, in.acc: "; 
			// For3 { cout << in.gyr[i] << ", "; }
			// cout << " ";
			// For3 { cout << in.acc[i] << ", "; }
			// cout << endl;
		
		// 打印pid计算过的角加速度和线加速度
		    // cout << "com.aAcc, com.acc: ";
			// For3 { cout << com.aAcc[i] << ", "; }
			// cout << " ";
			// For3 { cout << com.acc[i] << ", "; }
			// cout << endl;

		// 打印 Balence Controller 优化的需要参与平衡的力
			// cout << "cmdTipF:  ";
			// For3 { cout << cmdTipF3d[0][i] << ", "; }
			// cout << "   ";
			// For3 { cout << cmdTipF3d[1][i] << ", "; }	
			// cout << endl;

		// 打印实际经过正向刚体动力学解算得到的末端力
			// cout << "actF3d:  ";
			// For3 { cout << rbt.getAnkleF3d2B(0)[i] << ", "; }
			// cout << "   ";
			// For3 { cout << rbt.getAnkleF3d2B(1)[i] << ", "; }	
			// cout << endl;

		// tgt.p
			// cout << "tgt.p:  ";
			// For3 { cout << tgt.p[i] << ", ";}; 
			// cout << endl;

			// cout << "out.t: ";
			// For(6) { cout << out.t[i] << ", "; }
			// cout << endl;

		
		}
	}

} // namespace
