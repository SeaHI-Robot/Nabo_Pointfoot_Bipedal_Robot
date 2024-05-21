/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#pragma once
#include <cmath>
using namespace std;


// 定义常用数学常数
static const double Pi = 3.14159265359;
static const double Pi2 = 6.28318530718;
static const double P2i = 1.57079632679;


// 定义一些循环宏,用于简化循环代码
#define For3 for (int i(0); i != 3; ++i)
#define For4 for (int i(0); i != 4; ++i)
#define For6 for (int i(0); i != 6; ++i)
#define For(x) for (int i(0); i != x; ++i)

namespace Alg
{
	//==符号函数======================
	inline int sign(const int &a) { return a < 0 ? -1 : 1; }  // 返回整数 a 的符号 (-1 或 1)
	inline int sign(const float &a) { return a < 0 ? -1 : 1; }  // 返回浮点数 a 的符号 (-1 或 1)
	inline int sign(const double &a) { return a < 0 ? -1 : 1; }  // 返回双精度浮点数 a 的符号 (-1 或 1)
	
	//==双曲正割，钟形=================
	inline double sech(float x) { return 2 / (exp(x) + exp(-x)); }  // 计算双曲正割函数的浮点数值
	inline double sech(double x) { return 2 / (exp(x) + exp(-x)); }  // 计算双曲正割函数的双精度浮点数值
	
	//==高斯函数，钟形，高阶,更收缩=======
	inline double gause(float x) { return exp(-x * x); }  // 计算高斯函数的浮点数值
	inline double gause(double x) { return exp(-x * x); }  // 计算高斯函数的双精度浮点数值
	
	//==上下限区间，触发过边界return 1====================
	template <typename T1, typename T2, typename T3>
	bool clip(T1 &x, const T2 &a, const T3 &b);  // 将 x 限制在 [a, b] 区间内,超出返回 1
	template <typename T1, typename T2>
	bool clip(T1 &x, T2 lim);  // 将 x 限制在 [-lim, lim] 区间内,超出返回 1
	
	//==递增函数=============当cmd=tgt时return 1
	inline bool cmd2out1step(double cmd, double &tgt, double step)  
	{  // 将 tgt 以步长 step 递增或递减到 cmd
		if (step < 0)
		{
			step = -step;
		}
		if (tgt - cmd <= -step)
		{
			tgt += step;
			return 0;
		}
		else if (tgt - cmd >= step)
		{
			tgt -= step;
			return 0;
		}
		else
		{
			tgt = cmd;
			return 1;
		}
	}
	//==阀值=============
	inline void thresh(double &x, double threshold)
	{  // 对 x 进行阈值处理
		if (threshold < 0)
		{
			threshold = -threshold;
		}
		if (x > threshold)
		{
			x -= threshold;
		}
		else if (x < -threshold)
		{
			x += threshold;
		}
		else
		{
			x = 0;
		}
	}
	inline double threshed(double x, double threshold)
	{  // 返回 x 的阈值处理结果
		if (threshold < 0)
		{
			threshold = -threshold;
		}
		if (x > threshold)
		{
			x -= threshold;
		}
		else if (x < -threshold)
		{
			x += threshold;
		}
		else
		{
			x = 0;
		}
		return x;
	}

	//==一阶滤波 class============
	class filterOneClass
	{
	public:
		filterOneClass();
		filterOneClass(double dt, double cutF, double ki = 1, double y0 = 0);
		double init(double dt, double cutF, double ki = 1, double y0 = 0);
		double reset(double cutF, double ki = 1, double y0 = 0);
		double setBase(double y0 = 0);
		void setCutF(double cutF, double ki = 1);
		const double &filt(const double &x);

	private:
		double dt;
		double y, errSum;
		double k, ks;
	};

	//==二阶低通滤波 class============
	class filterTwoClass
	{
	public:
		filterTwoClass();  // 构造函数
		filterTwoClass(double dt, double cutF, double ki = 1, double y0 = 0);  // 构造函数，初始化参数
		double init(double dt, double cutF, double ki = 1, double y0 = 0);  // 初始化参数
		double reset(double cutF, double ki = 1, double y0 = 0);  // 重置参数
		double setBase(double y0 = 0);   // 设置初始值
		void setCutF(double cutF, double ki = 1);   // 设置截止频率
		const double &filt(const double &x);  // 二阶低通滤波
		double getV();  // 获取滤波器输出

	private:
		double dt;   // 采样时间
		double y, yd, ydd;  // 滤波器状态变量
		double errSum, ks;  // 滤波器系数
		double w, w1414, ww; // w=2*Pi*截止频率  滤波器系数(与截止频率有关)
	};
} // namespace