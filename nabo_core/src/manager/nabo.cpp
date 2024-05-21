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
