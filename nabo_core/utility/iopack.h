/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

/*
测试环境
fout定义，集成更易用的print系列
*/
#pragma once
#include<iostream>
#include"eigen.h"
#include"ini.h"


// ===== eigen 打印 =====
extern void printEi(const Eigen::MatrixXd &m,bool enter=0);
// ===== eigen log =====
extern void fprintEi(const Eigen::MatrixXd &m,bool enter=0);
// ===== 变参数打印 =====
template<typename T>
void print(const T& a){
	cout<<a<<endl;
};
// void print(const double& a){
// 	cout<<a<<endl;
// };
template<typename T1,typename... T2>
void print(const T1& a,const T2&...b){
	cout<<a<<"\t";
	print(b...);
};
// template<typename... T2>
// void print(const double& a,const T2&...b){
// 	if(abs(a)<1e-4){cout<<"0\t";}
// 	else{cout<<a<<"\t";}
// 	print(b...);
// };
//--数组
template<typename T,int n>
void print(const T(&a)[n]){
	for(int i=0;i<n-1;i++){
		if(abs(a[i])>1e-4){cout<<a[i]<<"\t";}
		else{cout<<"0\t";}
	}
	if(abs(a[n-1])>1e-4){cout<<a[n-1]<<endl;}
	else{cout<<"0\n";}
}
// ===== 变参数log =====
template<typename T>
void fprint(const T& a){
	fout<<a<<"\t;"<<endl;
}
template<typename T1,typename... T2>
void fprint(const T1& a,const T2&...b){
	fout<<a<<"\t";
	fprint(b...);
}
// template<typename... T2>
// void fprint(const double& a,const T2&...b){
// 	if(abs(a)<1e-4){fout<<"0\t";}
// 	else{fout<<a<<"\t";}
// 	fprint(b...);
// }
