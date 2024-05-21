#include "estimator.h"
#include "robot.h"

namespace Est
{
	class estClass::impClass
	{
	public:
		impClass(estClass *omp);
		Rbt::rbtClass &rbt = Rbt::rbtClass::instance();
		estClass &omp;
		double dt;

		const vec3d g = vec3d(0, 0, 10.0);
		// const vec3d g = vec3d(0, 0, 9.8);
		mat3d I3 = mat3d::Identity();

		void run();
	};
	estClass::impClass::impClass(estClass *omp) : omp(*omp)
	{
	}
	void estClass::impClass::run()
	{
		auto &imu = rbt.imu;
	}
	//=========================================
	estClass &estClass::instance()
	{
		static estClass singtn;
		return singtn;
	}
	estClass::estClass() : imp(*new impClass(this)) {}

	void estClass::init(double dt)
	{
		imp.dt = dt;
		setZero();
	}
	void estClass::setZero()
	{
		p.setZero();
		v2F.setZero();
		slope[0] = 0;
		slope[1] = 0;
	}
	void estClass::run()
	{
		imp.run();
	}
} // namespace
