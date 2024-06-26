<br>

<p align='center'><img src="./assets.README/nabopointfoot.png" alt="nabo_pointfoot"  width="550"/>
</a>
</p>

<p align="center">
"<b>Na</b>ughty <b>bo</b>y <b>Pointfoot</b>" 🤖
</p>

<p align="center">
    <a href="https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot" target="__blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot?style=social"></a>
</p>

<p align="center">
  <a href="https://www.bilibili.com/video/BV1Cx4y1q76N/?spm_id_from=333.999.0.0&vd_source=489a733550a7c846fcce2e3eb3a683cc">Bilibili</a> | <a href="https://github.com/tryingfly/nabo">Nabo</a> | <a href="https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot/blob/nabo_pointfoot_opensourced/README_zh.md">简体中文</a> 
</p>
<br>


https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot/assets/84261527/11ce7f6e-0806-42d1-891f-866498119e43


<br>

## Introduction

- Nabo Pointfoot is a **pointfoot bipedal robot simulation framework** in [MuJoCo](https://github.com/google-deepmind/mujoco), constructed fully with C++. 

- Nabo Pointfoot is built on top of [**nabo**](https://github.com/tryingfly/nabo),  which is a bipedal robot framework for a 12DOF robot model.

- **MPC+WBIC** balence controller. The controller code is isolated from simulation, which is elegant.

- Nabo Pointfoot walks with the height of 0.45m, maximux walking speed **0.95m/s**.

<br>

## Features

- 💻Pure C++
- ⚡️Computing Efficient, formulating QP with [eiquadprog](https://github.com/stack-of-tasks/eiquadprog)
- 🤹Interactive [MuJoCo](https://github.com/google-deepmind/mujoco) GUI
- 📝Rich code comments but most in Chinese, I apologize for those who don't understand Chinese


<p align='center'><img src="./assets.README/model_comparison.png" alt="nabo_pointfoot"  width="500"/>
</a>
</p>

<p align="center">
Size Comparison with <b>LIMX P1</b> & <b>Unitree H1</b> 🤖
</p>

<br>

## Usage
 
1. This project was developed on Ubuntu20.04, g++/gcc version `9.4.0`. 
2. `$ git clone https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot.git`, then cd to this repo's directory.
3. `$ ./make_and_run` to compile both the controller and the simulation, then start simulation with the compiled controller loaded. 
4. When both codes in `nabo_core/` and `nabo_mujoco/` are compiled，simply do `$ ./run_sim.sh` to only start the simulation without any compilation.
5.  Edit `nabo_mujoco/000.ini` to adjust the parameters for tuning. `nabo_mujoco/zzz.txt` stores the log file during simulation when the log flag is turned on. 
6. Pitch is currently poorly controlled at close to zero speed and will stabilize with a bias. Stable yaw angle control cannot be achieved for the time being.

> In the MuJoCo GUI:
>
>  Press:
>  - w/s : change command speed in x
>  - a/d : change command speed in y
>  - j/l : change command angular speed in z
>  - x: clear all speed command
>  - backspace: reset
>  - space: pause
>  - f: toggle visualization of contact force
>  - z: toggle log flag

<br>

## Discussion

If U have any problem, please start an **Issue**, or join QQ group 609601974. 

<br>

## Acknowledgement

1. [nabo](https://github.com/tryingfly/nabo),
2. [MIT mini-cheetah](https://arxiv.org/abs/1909.06586),
3.  [MuJoCo](https://mujoco.org/) Simulator, 

4. and SUSTech🌈.

<p align='center'><img src="./assets.README/SUSTech-en.png" alt="SUSTech"  width="350"/>
</a>
</p>
