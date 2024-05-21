/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include "main.h"
#include "robot.h"
#include "cpg.h"
#include "mpc.h"
#include "cbic.h"
#include "manager.h"

double dt = 0.002;

void setup()
{
	Rbt::rbtClass &rbt = Rbt::rbtClass::instance();
	Cpg::cpgClass &cpg = Cpg::cpgClass::instance();
	Blc::mpcClass &mpc = Blc::mpcClass::instance();
	Blc::cbicClass &cbic = Blc::cbicClass::instance();
	Blc::stateStruct stt;
	double duty[2]{0.5, 0.5};
	double offset[2]{0, 0.5};
	cpg.init(dt, 0.2, duty, offset);
	double jj[NMot]{0, -0.53, 1, 0, -0.45, 0.9};
	double ww[NMot]{};
	double tt[NMot]{};
	rbt.update(jj, ww, tt);
	// rbt.update(jj, ww, tt);
	if (ini["cpg"])
	{
		cpg.unlock();
	}
	cpg.update();
	cbic.init(dt);


	stt.setZero();
	stt.pDlt << 0, -0.2, -0.05;
	stt.v << 0, -0.8, -0.2;
	stt.aDlt << 0.36, 0, 0;
	ini.getArray("acc", stt.pDlt.data(), 3);
	ini.getArray("aAcc", stt.aDlt.data(), 3);
	printEi(stt.aDlt);
	mpc.setState(stt);
}


void tictoc()
{
	Rbt::rbtClass &rbt = Rbt::rbtClass::instance();
	Blc::mpcClass &mpc = Blc::mpcClass::instance();
	Cpg::cpgClass &cpg = Cpg::cpgClass::instance();
	vec3d mf[2];
	mpc.test();
	mpc.getMF(mf[0], mf[1]);
	printEi(mf[0]);
	printEi(mf[1]);

	// vec3d mf[2];
	// For(200){
	// 	cpg.update();
	// 	mpc.testRun();
	// 	mpc.getMF(mf[0],mf[1]);
	// 	fprint(mf[0].transpose(), mf[1].transpose());
	// }

	// Blc::cbicClass&cbic=Blc::cbicClass::instance();
	// vec3d f[2];
	// f[0].setZero();f[1].setZero();
	// f[0][2]=-80;f[1][2]=-80;
	// cbic.setFr(f);
	// vec3d acc,aAcc;
	// ini.getArray("acc",acc.data(),3);
	// ini.getArray("aAcc",aAcc.data(),3);
	// rbt.setSwing(1,1);
	// cbic.run(acc,aAcc);
	// printEi(cbic.mf[0]);
	// printEi(cbic.mf[1]);
}
