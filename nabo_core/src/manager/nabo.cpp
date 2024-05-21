/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#include "manager.h"
#include "nabo.h"

namespace Nabo
{
	
	void setDt(double dt)
	{
		Mng::manageClass::instance().setDt(dt);
	}
	
	void run(inputStruct &in, outputStruct &out)
	{
		Mng::manageClass::instance().run(in, out);
	}

} // namespace
