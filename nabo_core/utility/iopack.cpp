/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
测试环境
集成更易用的print系列
*/

#include<sstream>
#include<iomanip>
#include"iopack.h"

void eiPrint(ostream &out,const Eigen::MatrixXd &m,bool enter){
	auto r=m.rows(),c=m.cols();
	//找最大宽度
	size_t width{0};
	For(r){
		for(int j=0;j!=c;++j){
			stringstream ss;
			if(abs(m(i,j))>1e-4){
				ss<<m(i,j);
				if(ss.str().length()>width){
					width=ss.str().length();
				}
			}
		}
	}
	//列向量行打印
	if(c==1){
		For(r){
			if(abs(m(i,0))<1e-4){out.width(width);out<<0;}
			else{out.width(width);out<<m(i,0);}
			if( i < r-1 ){ out<<" "; }
		}
		out << ";\n";
	}else{
		For(r){
			for(int j=0;j!=c;++j){
				if(abs(m(i,j))<1e-4){out.width(width);out<<0;}
				else{out.width(width);out<<m(i,j);}
				if(j<c-1){out<<" ";}
			}
			out << ";\n";
		}
	}
	if(enter){ out<<endl; }
}

void printEi(const Eigen::MatrixXd &m,bool enter){
	eiPrint(cout, m, enter);
}
void fprintEi(const Eigen::MatrixXd &m,bool enter){
	eiPrint(fout, m, enter);
}