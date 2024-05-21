/*
Nabo Pointfoot, built on top of [nabo](https://github.com/tryingfly/nabo)
Under MIT License.

<https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot>
*/

#include <sstream>
#include <string>
#include <chrono>
#include <thread>
#include "mujoco/mujoco.h"
#include "glfw3.h"
#include "nabo.h"
#include <iostream>
#define For(x) for (int i = 0; i < x; ++i)
using namespace std;


// #define TimeCostTest

#ifdef TimeCostTest
chrono::duration<double> total_duration(0);
int cnt = 0;
#pragma message("main.cpp: nabo核心库计算时间测试开启")
#endif


int sleepMs = 20;
double dt;
Nabo::inputStruct in;
Nabo::outputStruct out;
bool floatBaseFlag = 0;
bool slowRenderFlag = 1;
bool startLogFlag = 0;

stringstream msg;

inline void clip(double &x, double lim)
{
	if (lim < 0)
	{
		lim = -lim;
	}
	if (x < -lim)
	{
		x = -lim;
	}
	else if (x > lim)
	{
		x = lim;
	}
}

inline void initRobotPose(mjModel &m, mjData &d)
{
	if (m.nq > NMot) // j1 = {0, 0, -0.5, 1, -0.5, 0, 0, 0, -0.5, 1, -0.5, 0};  //
	{
		floatBaseFlag = 1;

		d.qpos[0 + 7] = 0;
		d.qpos[1 + 7] = -0.5;
		d.qpos[2 + 7] = 1.;

		d.qpos[3 + 7] = 0;
		d.qpos[4 + 7] = -0.5;

		d.qpos[5 + 7] = 1.;
	}
	else
	{
		floatBaseFlag = 0;
		d.qpos[0] = 0;
		// d.qpos[1] = -0.5;
		d.qpos[1] = -0.9;
		d.qpos[2] = 1.2;

		d.qpos[3] = 0;
		d.qpos[4] = -0.9;
		d.qpos[5] = 1.2;
	}
}

// =============  主控制函数  =============
void runCtrl(mjData &d)
{

	in.cnt = (int)(d.time / dt);

	if (floatBaseFlag) // 是floatBase
	{
		in.LogFlag = startLogFlag;
		mjtNum *q = d.qpos + 3;
		in.rpy[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
		in.rpy[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
		in.rpy[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
		for (int i = 0; i < 3; i++) // 取前三个维度
		{
			in.gyr[i] = d.qvel[i + 3];
			in.acc[i] = d.qacc[i];
			in.supP[i] = d.qpos[i];
			in.supV[i] = d.qvel[i];
			// in.Aacc[i] = d.qacc[i + 3];
		}
		// in.supP[2] -= 0.005; // 考虑到穿模，这里给了一点offset
		For(NMot)
		{
			in.j[i] = d.qpos[i + 7]; // 当存在浮基，qpos前7个数为【浮动基3d位置+四元数】
			in.w[i] = d.qvel[i + 6]; // 当存在浮基，qvel前6个数为【浮动基3d速度+角速度】
			in.t[i] = d.actuator_force[i];
		}
	}
	else // 不是floatBase
	{
		// in.supP[2] = 0.45 - (1e-4);  // 为何 - 1e-4
		in.supP[2] = 0.45;
		For(NMot)
		{
			in.j[i] = d.qpos[i];
			in.w[i] = d.qvel[i];
			in.t[i] = d.actuator_force[i];
		}
	}


#ifdef TimeCostTest
	auto tik = std::chrono::steady_clock::now();
#endif

	// 调用Nabo函数run获得输出
	Nabo::run(in, out);

#ifdef TimeCostTest
	auto tok = std::chrono::steady_clock::now();
	chrono::duration<double> duration = tok - tik;
	total_duration += duration;
#endif


	For(NMot)
	{
		d.ctrl[i] = out.t[i];
	}
}

// =================================================
mjModel *m;
mjData *d;
mjvCamera cam;	// abstract camera
mjvOption opt;	// visualization options
mjvScene scn;	// abstract scene
mjrContext con; // custom GPU context
GLFWwindow *window;

// ========== mouse interaction ==========
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
double pause = false;

// =============  keyboard callback  =============
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
	// backspace: reset simulation
	if (act == GLFW_PRESS)
	{
		switch (key)
		{
		case GLFW_KEY_BACKSPACE:
			mj_resetData(m, d);
			initRobotPose(*m, *d);
			mj_forward(m, d);
			cam.lookat[0] = -0.0913163;
			cam.lookat[1] = -0.259744;
			cam.lookat[2] = 0.355032;
			cam.azimuth = -92.6;
			cam.elevation = -3.15;
			cam.distance = 2.4;
			in.cmdVx = 0;
			in.cmdVy = 0;
			in.cmdWz = 0;
			break;
		case GLFW_KEY_X:
			in.cmdVx = 0;
			in.cmdVy = 0;
			in.cmdWz = 0;
			break;
		case GLFW_KEY_W:
			// in.cmdVx += 0.1;
			in.cmdVx -= 0.05;
			break;
		case GLFW_KEY_I:
			// in.cmdVx += 0.1;
			in.cmdVx -= 0.01;
			break;
		case GLFW_KEY_S:
			// in.cmdVx -= 0.1;
			in.cmdVx += 0.05;
			break;
		case GLFW_KEY_A:
			in.cmdVy += 0.05;
			break;
		case GLFW_KEY_D:
			in.cmdVy -= 0.05;
			break;
		case GLFW_KEY_J:
			in.cmdWz += 0.05;
			break;
		case GLFW_KEY_L:
			in.cmdWz -= 0.05;
			break;
		case GLFW_KEY_F:
			opt.flags[mjVIS_CONTACTFORCE] = !opt.flags[mjVIS_CONTACTFORCE];
			break;
		case GLFW_KEY_SPACE:
			pause = !pause;
		case GLFW_KEY_Z:
			startLogFlag = !startLogFlag;
		}
		if (abs(in.cmdVx) < 1e-4)
		{
			in.cmdVx = 0;
		}
		if (abs(in.cmdVy) < 1e-4)
		{
			in.cmdVy = 0;
		}
		if (abs(in.cmdWz) < 1e-4)
		{
			in.cmdWz = 0;
		}
		clip(in.cmdVx, 1.0);
		// clip(in.cmdVx, 0.5);
		clip(in.cmdVy, 0.2);
		clip(in.cmdWz, 0.5);
		msg.clear();
		msg.str("");
		// msg << "cmd: vx=" << in.cmdVx << ", vy=" << in.cmdVy << ", wz=" << in.cmdWz <<"; LogFlag=" << startLogFlag;
		msg << "cmd: vx=" << in.cmdVx << ", vy=" << in.cmdVy << ", wz=" << in.cmdWz;
	}
}

// =============  mouse button callback  =============
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
	// update button state
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}

// =============  mouse move callback  =================
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
	if (!button_left && !button_middle && !button_right)
	{
		return;
	}
	// compute mouse displacement, save
	double dx = xpos - lastx, dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;
	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	// get shift key state
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
	// determine action based on mouse button
	mjtMouse action;
	if (button_right)
	{
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	}
	else if (button_left)
	{
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	}
	else
	{
		action = mjMOUSE_ZOOM;
	}
	mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// =============  scroll callback  =============
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
	// emulate vertical mouse motion=5% of window height
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

int main(int argc, char *argv[])
{
	char errorMsg[64]{"fail to load xml!\n"};
	// m = mj_loadXML("../model/nabo_pointfoot_spherefoot.xml", 0, errorMsg, 64);
	m = mj_loadXML("../model/nabo_pointfoot.xml", 0, errorMsg, 64);
	d = mj_makeData(m);

	if (!glfwInit())
	{
		mju_error("Could not initialize GLFW");
	}

	// create window, make OpenGL context current, request v-sync
	window = glfwCreateWindow(1200, 800, "nabo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);

	cam.lookat[0] = -0.0913163;
	cam.lookat[1] = -0.259744;
	cam.lookat[2] = 0.355032;
	cam.azimuth = -92.6;
	cam.elevation = -3.15;
	cam.distance = 2.4;

	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_100);

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	dt = m->opt.timestep;
	Nabo::setDt(dt);

	initRobotPose(*m, *d);

	msg << "keyboard: {w s a d j l} for movments, {x} ro step in place, {space} to pause";

	// ==========  仿真主循环  ==========
	while (!glfwWindowShouldClose(window))
	{
		mjtNum simstart = d->time;
		// 小循环内mj_step按硬件能力进行场景计算，循环外按30帧更新显示
		while (!pause && d->time - simstart < 1.0 / 30)
		{
			runCtrl(*d);

#ifdef TimeCostTest
			cnt += 1;
#endif
			mj_step(m, d);
			cam.lookat[0] = d->subtree_com[0];
			cam.lookat[1] = d->subtree_com[1];
			cam.lookat[2] = d->subtree_com[2];

			// cam.distance = 2.4;
		}
		// 延时为了人眼看清楚。另：不同旋转视角渲染速度也不同
		if (slowRenderFlag)
		{
			this_thread::sleep_for(chrono::milliseconds(sleepMs));
		}
		// if(d->time>0.99){exit(0);}
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		mjrRect rect{0, 0, 200, 50};
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, msg.str().data(), 0, &con);
		glfwSwapBuffers(window); // swap OpenGL buffers(blocking call due to v-sync)
		glfwPollEvents();		 // process pending GUI events, call GLFW callbacks
	}

	cout << "\n\nazimuth: " << cam.azimuth << "  elevation: " << cam.elevation << "  distance: " << cam.distance << "\nlookat: " << cam.lookat[0] << " " << cam.lookat[1] << " " << cam.lookat[2] << endl; // 退出的时候打印相机的视角

	mjv_freeScene(&scn);
	mjr_freeContext(&con);
	mj_deleteData(d);
	mj_deleteModel(m);
	// terminate GLFW(crashes with Linux NVidia drivers)
	// #if defined(__APPLE__) || defined(_WIN32)
	// glfwTerminate();
	// #endif

#ifdef TimeCostTest
	double average_duration = total_duration.count() / cnt;
	cout << "\n\nAverage nabo_core computation duration: " << average_duration << " seconds" << std::endl;
#endif

	return 0;
}
