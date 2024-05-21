/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#include <chrono>
#include <thread>
#include "iopack.h"
#pragma once

void setup();
void tictoc();

int main(int argc, char *argv[])
{
	cout << endl;
	setup();
	auto clockTag = chrono::high_resolution_clock::now();
	tictoc();
	auto clockPeriod = chrono::duration_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now() - clockTag);
	double clockPeriodMs = clockPeriod.count() / 1000000.0;
	cout << "\n------\ntime cost: " << clockPeriodMs << " ms" << endl;
	fout.close();
	return 0;
}
