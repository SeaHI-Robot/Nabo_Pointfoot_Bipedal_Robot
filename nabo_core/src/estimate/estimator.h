/*=========== ***doc description @ yyp*** ===========
#include"estimator.h"
	Est::estClass &est=Est::estClass::instance();
=====================================================*/
#pragma once
#include "eigen.h"

namespace Est
{
	class estClass
	{
	public:
		static estClass &instance();
		void init(double dt);
		void setZero();
		void run();
		vec3d p, v, v2F; // p：xy+身高z
		double slope[2]; // F系rol、pit
	private:
		estClass();
		estClass(const estClass &) = delete;
		estClass &operator=(const estClass &) = delete;

		class impClass;
		impClass &imp;
	};
} // namespace
