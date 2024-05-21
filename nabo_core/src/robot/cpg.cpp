/*=========== ***doc description @ yyp*** ===========

采用时钟计时生成步态周期

// ******************** CPG过程中的变量 ******************** //
pgs: 表示 CPG 输出的相位进度,范围为 [0, 1)。它用于跟踪一个完整的步态周期的进度。
swPgs[Num]: 一个数组,表示每条腿在摆动阶段的相位进度,范围为 [0, 1)。
swPgs2[Num]: 一个数组,在摆动阶段与 swPgs 相同,但在支撑阶段对应另一条腿的站立swPgs,可以用来计算摆动过程的剩余时间。
stPgs[Num]: 一个数组,表示每条腿在支撑阶段的相位进度,范围为 [0, 1)。
swTic[Num]: 一个数组,表示每条腿在当前摆动阶段已经经过的时间。
swTic2[Num]: 一个数组,在摆动阶段与 swTic 相同,但在支撑阶段对应另一条腿的站立stTic,可以用来计算摆动过程的剩余时间。
stTic[Num]: 一个数组,表示每条腿在当前支撑阶段已经经过的时间。
isSw[Num]: 一个布尔数组,表示每条腿当前是否处于摆动阶段。
isSwAny: 一个布尔变量,表示是否至少有一条腿处于摆动阶段。
isFirstStep[Num]: 一个布尔数组,表示腿是否处于第一步。
locking: 一个布尔变量,表示 CPG 是否处于锁定状态。
shouldStart: 一个布尔变量,表示在解锁后是否应该立即启动 CPG。
stT[Num]: 这是一个数组,表示每条腿在支撑(stance)阶段的持续时间。它的值由步态周期 period 和占空比 duty 决定。
swT[Num]: 这是一个数组,表示每条腿在摆动(swing)阶段的持续时间。它的值由步态周期 period 和支撑时间 stT 决定。
// ******************** CPG过程中的变量 ******************** //

=====================================================*/


#include <cstring>  // 包含用于memset函数的头文件
#include "cpg.h"  // 包含cpg.h头文件
#include "algorithms.h"  // 包含算法工具函数头文件

#define For2 for (int i(0); i != 2; ++i)  // 定义一个循环2次的宏

namespace Cpg
{
	cpgClass &cpgClass::instance(){  // 单例模式,获取cpgClass的唯一实例
		static cpgClass singtn;
		return singtn;
	};
	
	cpgClass::cpgClass(){}  // cpgClass的默认构造函数
	

	// 初始化CPG,设置步态周期、占空比和相位偏移等参数
	void cpgClass::init(const double dt, const double period, const double (&duty)[Num], const double (&offset)[Num])
	{
		this->dt = dt;  // 时间步长
		this->period = period;  // 设置步态周期
		cmd.period = period; 
		// cmd.offset是一个长度为2的双精度数组,表示两条腿相位偏移的命令值。
		// cmd.duty是一个长度为2的双精度数组,表示中心模式发生器(CPG)的占空比(duty cycle)命令值。
		// cmd.period是一个标量(单个值),表示CPG输出的周期时长的命令值,单位是秒。
		memcpy(cmd.offset, offset, 8 * Num);
		memcpy(cmd.duty, duty, 8 * Num);
		For2
		{
			Alg::clip(cmd.duty[i], 0.00001, 0.99999);  // 将占空比值限制在合理范围内
			Alg::clip(cmd.offset[i], 0, 1);  // 将相位偏移值限制在0到1之间
			stT[i] = period * cmd.duty[i];   // stT: stance time,表示支撑(stance)阶段的持续时间。
			swT[i] = period - stT[i]; 		 // swT: swing time,表示摆动(swing)阶段的持续时间。
		}
		memcpy(this->offset, cmd.offset, 8 * Num);
		memcpy(this->duty, cmd.duty, 8 * Num);
		setZero();
	}
	
	void cpgClass::setZero()
	{
		locking = 1;  // 初始化为锁定状态；锁定状态(locking)是一种特殊的状态,用于控制CPG的启动和停止
		shouldStart = 0;  // 不立即启动  
		pgs = pgsEarly;  // 设置为提前相位， 在unlock后等一段时间再动，负数
		memset(isFirstStep, 1, 2);  // 将isFirstStep数组的前2个元素设置为1,表示第一步
		memset(swTic, 0, 8 * Num);  // 将swTic数组的所有元素设置为0
		memset(swPgs, 0, 8 * Num);  // 将swPgs数组的所有元素设置为0
		memset(isSw, 0, 2);  // 将isSw数组的前2个元素设置为0,表示初始为支撑状态
		For2{
			stPgs[i] = 1;  // 将stPgs设置为1,表示支撑阶段刚结束
			stTic[i] = stT[i];  // 将stTic设置为支撑阶段的总时长
		}
	}

	// 设置CPG的参数,实现平滑改变
	void cpgClass::setParam(const double period, const double (&duty)[Num], const double (&offset)[Num])
	{
		cmd.period = period;
		memcpy(cmd.offset, offset, 8 * Num);  // 复制相位偏移命令值
		memcpy(cmd.duty, duty, 8 * Num);  // 复制占空比命令值
		Alg::clip(cmd.period, 0.1, 10);  // 保证数据大小安全
		For2{
			Alg::clip(cmd.offset[i], 0, 1);  // 保证数据大小安全
			Alg::clip(cmd.duty[i], 0.00001, 0.99999);  // 保证数据大小安全
		}
	}

	// 判断未来时间fT时是否两条腿都处于摆动阶段
	bool cpgClass::isAllSw(double fT)
	{
		bool re = !isStop();  // 如果不是停止状态,才有可能全为摆动
		double fp(fT / period);   //等价于fp = fT / period； 计算未来时间对应的相位进度
		For2
		{
			double tmp(pgs + fp - offset[i] + 1);  //+1省去判断<0; tmp可理解为当前进度
			tmp -= int(tmp);  // 取tmp的小数部分
			re &= (tmp < (1 - duty[i]));  // 如果tmp小于摆动阶段的时间比例，则为摆动状态 （&=是什么鬼）
			re &= !(isFirstStep[i] & (pgs + fp < offset[i]));  // 
		}
		return re;
	}

	// 判断未来时间fT时每条腿是否处于支撑状态,并将结果存储在isSt数组中
	void cpgClass::isFutureSt(double fT, bool *isSt)
	{
		double fp(fT / period);  // 等价于fp = fT / period
		For2
		{
			isSt[i] = locking & (!isSw[i]);  // 如果锁定且当前不是摆动状态,则为支撑状态
			double tmp(pgs + fp - offset[i] + 1); // +1省去判断<0; tmp可理解为当前进度
			tmp -= int(tmp);  // 取tmp的小数部分
			isSt[i] |= !(tmp < 1 - duty[i]);  // 如果tmp不小于摆动阶段的持续时间比例,则为支撑状态  （|=是什么鬼）
			isSt[i] |= isFirstStep[i] & (pgs + fp < offset[i]);  // 如果是第一步且未到相位偏移,则为支撑状态
			// cout<<"cpg:"<<fT<<", "<<tmp<<", "<<isSt[i]<<endl;
		}
	}
	
	// 判断未来时间fT时每条腿的状态,并将结果存储在stage数组中
    // 0表示摆动阶段,1表示第一次支撑阶段,2表示连续多个支撑阶段
	void cpgClass::isFutureSt(double fT, int *stage)
	{
		
		double fp(fT / period);  // 计算未来时间对应的相位进度
		For2
		{
			double tmp(pgs - offset[i] + 2);  //计算相对相位, 多+2保证为正
			tmp += -int(tmp) + fp;  // 调整tmp范围到[0,2)
			if (isSw[i])  // 如果当前是摆动状态
			{
				if (tmp > 1 - duty[i] && tmp < 1)  // 如果相位在[1-duty,1)范围内
				{
					stage[i] = 2;  // 表示连续多个支撑阶段
				}
				else
				{
					stage[i] = 0;  // 否则为摆动阶段
				}
			}
			else if (locking)  // 如果当前锁定
			{
				stage[i] = 1;  // 则为第一次支撑阶段
			}
			else  // 如果当前不锁定且不是摆动状态
			{
				if (tmp < 1)   // 如果相位小于1
				{ // 好处是不用考虑第一步
					stage[i] = 1;  // 支撑阶段
				}
				else if (tmp < 2 - duty[i]) // 如果相位在[1, 2-duty[i]]之间
				{
					stage[i] = 0;  // 摆动状态
				}
				else
				{
					stage[i] = 2;  // 连续多个支撑阶段
				}
			}
		}
	}
	
	// 计算从未来时间fT开始,下一次状态变换的时间
	double cpgClass::getNextEvent(double fT)
	{
		double fp(fT / period), cmp = 1;  // 初始化相位进度fp和比较值cmp
		For2
		{
			double tmp(offset[i] - pgs - fp + 100); //+100保证为正; 计算相对相位,+100确保tmp>0
			tmp -= int(tmp);  // 取tmp的小数部分
			if (tmp < cmp)
			{
				cmp = tmp;
			}
			tmp += 100 - duty[i]; //+100保证为正
			tmp -= int(tmp);
			if (tmp < cmp)
			{
				cmp = tmp;
			}
		}
		return cmp * period;
	}
	
	// 更新CPG的状态
	void cpgClass::update()
	{   
		isSwAny = 0;  // 初始化isSwAny为0,表示没有腿处于摆动状态
		For2
		{
			isSwAny |= isSw[i];   // 如果有腿处于摆动状态,设置isSwAny为true
		}
		if (!isSwAny && shouldStart)  // 如果没有腿处于摆动状态且需要启动
		{
			shouldStart = 0;
			locking = 0;
		}

		//==步态参数线性变化==
		Alg::cmd2out1step(cmd.period, period, 0.0015);
		For2
		{
			Alg::cmd2out1step(cmd.offset[i], offset[i], 0.0016);
			Alg::cmd2out1step(cmd.duty[i], duty[i], 0.0008);
			stT[i] = period * duty[i];
			swT[i] = period - stT[i];
		}
		//=======
		
		pgs += dt / period;  // 更新相位进度
		pgs -= int(pgs);  // 确保相位进度在[0,1)范围内
		For2
		{
			if (isSw[i])           // 如果当前是摆动状态
			{                      
				if (calSw(i))      // 如果结束摆动
				{				   
					calSt(i);      // 计算支撑状态
				}
			}
			else if (locking)   	// 如果当前锁定
			{
				stTic[i] = stT[i];  // 将支撑时间增量设置为支撑时长
				stPgs[i] = 1;		 // 将支撑阶段进度设置为1
				swPgs2[i] = 0;		// 清空摆动阶段进度2
				swTic2[i] = 0;		// 清空摆动时间增量2
				if (!isSwAny)		// 如果没有腿处于摆动状态,设置相位进度为提前值
				{
					pgs = pgsEarly;  // 在unlock后等一段时间再动，负数
				}					
				isFirstStep[i] = 1; // 在isAllSw(fT)内判断第一步 // 标记为第一步
			}
			else					// 如果当前是支撑状态
			{ 
				if (pgs < 0)
				{ // 在unlock后等一段时间再动
					continue;
				}
				if (calSt(i))  // 如果支撑结束
				{
					calSw(i);  // 计算摆动状态
				}
			}
		}
	}
	
	// 计算第i条腿的摆动状态
	bool cpgClass::calSw(int i)
	{
		
		isFirstStep[i] = 0;
		swPgs[i] = pgs - offset[i];
		if (swPgs[i] < 0)
		{
			swPgs[i] += 1;
		}
		swPgs[i] /= 1 - duty[i];
		swTic[i] = swPgs[i] * swT[i];

		swTic2[i] = swTic[i]; // 在sw时二者相等，避免同时清零，需放在sw>=1之前
		swPgs2[i] = swPgs[i];
		if (swPgs[i] > 1 || duty[i] > 0.99)
		{
			swPgs[i] = 0;
			swTic[i] = 0;
			isSw[i] = 0;
			return 1;
		}
		return 0;
	}
	
	// 计算第i条腿的支撑状态
	bool cpgClass::calSt(int i)
	{
		
		stPgs[i] = pgs - offset[i] + duty[i] - 1;  // 计算支撑阶段进度
		if (stPgs[i] < 0)
		{
			stPgs[i] += 1;
		}
		// 让第一步不飞
		if (stPgs[i] < 0)
		{
			stPgs[i] = 0;
		}
		stPgs[i] /= duty[i];
		stTic[i] = stPgs[i] * stT[i];
		swPgs2[i] = pgs - offset[i];
		if (isFirstStep[i])
		{
			swPgs2[i] = 0;
		}
		if (swPgs2[i] < 0)
		{
			swPgs2[i] += 1;
		}

		swPgs2[i] /= 1 - duty[i];
		swTic2[i] = swPgs2[i] * swT[i];  // 在st时swTic2继续增长，在st>=1需要清零

		if (stPgs[i] >= 1)
		{
			stPgs[i] = 0;
			stTic[i] = 0;
			isSw[i] = 1;
			return 1;
		}
		return 0;
	}

	void cpgClass::unlock()
	{
		if (locking)
		{
			if (isSwAny)
			{
				shouldStart = 1;
			}
			else
			{
				locking = 0;
			}
		}
	};
} // namespace

//==cpg测试===========
// Cpg::cpgClass& cpg=Cpg::cpgClass::instance();
// double period=1,dt=0.01;
// double duty[Num]={0.6,0.6};
// double offset[Num]={0, 0.5};
// cpg.init(dt,period,duty,offset);

// double pgs,swPgs[Num],stPgs[Num],swPgs2[Num],swTic[Num],stTic[Num],swTic2[Num],swTicRm[Num],stTicRm[Num];
// double t=0;
// cpg.unlock();

// // int st[Num];
// // For(10){
// // 	cpg.isFutureSt(0.1*i,st);
// // 	cout<<st[0]<<","<<st[1]<<","<<st[2]<<","<<st[3]<<","<<st[4]<<","<<st[5]<<endl;
// // }

// for(t=0;t<3;t+=dt){
// 	// if(t>0.5){cpg.unlock();}
// 	cpg.update();
// 	pgs=cpg.getPgs();
// 	for(int i=0;i<Num;i++){
// 		swPgs[i]=cpg.getSwPgs(i);
// 		stPgs[i]=cpg.getStPgs(i);
// 		swPgs2[i]=cpg.getSwPgs2(i);
// 		swTic[i]=cpg.getSwTic(i);
// 		stTic[i]=cpg.getStTic(i);
// 		swTic2[i]=cpg.getSwTic2(i);
// 		swTicRm[i]=cpg.getSwTicRm(i);
// 		stTicRm[i]=cpg.getStTicRm(i);
// 	}
// 	fout<<t<<","<<pgs<<",swPgs,";
// 	for(int i=0;i<Num;i++){
// 		fout<<swPgs[i]<<",";
// 	}
// 	fout<<"stPgs,";
// 	for(int i=0;i<Num;i++){
// 		fout<<stPgs[i]<<",";
// 	}
// 	fout<<"swPgs2,";
// 	for(int i=0;i<Num;i++){
// 		fout<<swPgs2[i]<<",";
// 	}
// 	fout<<"swTic,";
// 	for(int i=0;i<Num;i++){
// 		fout<<swTic[i]<<",";
// 	}
// 	fout<<"stTic,";
// 	for(int i=0;i<Num;i++){
// 		fout<<stTic[i]<<",";
// 	}
// 	fout<<"swTic2,";
// 	for(int i=0;i<Num;i++){
// 		fout<<swTic2[i]<<",";
// 	}
// 	fout<<"swTicRm,";
// 	for(int i=0;i<Num;i++){
// 		fout<<swTicRm[i]<<",";
// 	}
// 	fout<<"stTicRm,";
// 	for(int i=0;i<Num;i++){
// 		fout<<stTicRm[i]<<",";
// 	}
// 	fout<<endl;
// }
