/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#include "manager.h"
#include "plan.h"
#include "plan_recover.h"
#include "plan_walk.h"

namespace Mng
{

	class manageClass::impClass{
		public:
			int rcCnt;
			Plan::rcPlanClass rc;
			Plan::walkPlanClass wk;
	};

	
	//=====================================================
	manageClass::manageClass() : imp(*new impClass()) {}
	
	
	manageClass &manageClass::instance()
	{
		static manageClass singtn;
		return singtn;
	}

	void manageClass::setDt(double dt)
	{
		imp.rcCnt = int(1 / dt); // “1” 表示在plan_recover控制中待1s
		imp.rc.setDt(dt);
		imp.wk.setDt(dt);
		cout << "\nnabo_manager: dt=" << dt << endl;
	}

	void manageClass::run(Nabo::inputStruct &in, Nabo::outputStruct &out)
	{
		if (!ini["lateWalk"])
		{
			imp.wk.run(in, out);
			imp.wk.log(in.LogFlag); // 涉及io，实机log应在实时线程之外，此处暂且不考虑
		}
		else
		{
			if (in.cnt < imp.rcCnt)
			{
				imp.rc.run(in, out);
				imp.rc.log(in.LogFlag); // 涉及io，实机log应在实时线程之外，此处暂且不考虑	
			}
			else
			{
				in.cnt -= imp.rcCnt;
				imp.wk.run(in, out);
				imp.wk.log(in.LogFlag); // 涉及io，实机log应在实时线程之外，此处暂且不考虑
			}
		}
	}

} // namespace
