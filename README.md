<br>

<p align='center'><img src="./assets.README/nabopointfoot.png" alt="nabo_pointfoot"  width="550"/>
</a>
</p>

<p align="center">
"<b>Na</b>ughty <b>bo</b>y Pointfoot" 🤖
</p>

<p align="center">
    <a href="https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot" target="__blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/slidevjs/slidev?style=social"></a>
</p>

<p align="center">
  <a href="https://twitter.com/antfu7/status/1389604687502995457">Video</a> | <a href="https://github.com/tryingfly/nabo">Nabo</a> | <a href="https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot/blob/nabo_pointfoot_opensourced/README_zh.md">简体中文</a> 
</p>

---

<br>

## Introduction

- **Nabo Pointfoot** is a pointfoot bipedal robot simulation framework in MuJoCo, constructed fully with C++. 

- **Nabo Pointfoot** is built on top of [**nabo**](https://github.com/tryingfly/nabo),  which is a bipedal robot framework for a 12DOF robot model.

- **MPC+WBIC** balence controller. The controller code is isolated from simulation, which is elegant.
- **Nabo Pointfoot** walks with the height of 0.45m, maximux walking speed 0.95m/s.

<br>

## Features

- 💻Pure C++
- ⚡️Computing Efficient, formulating QP with [eiquadprog](https://github.com/stack-of-tasks/eiquadprog)
- 🤹Interactive [MuJoCo](https://github.com/google-deepmind/mujoco) GUI



<p align='center'><img src="./assets.README/model_comparison.png" alt="nabo_pointfoot"  width="500"/>
</a>
</p>

<p align="center">
Size Comparison with <b>LIMX P1</b> & <b>Unitree H1</b> 🤖
</p>

<br>

## Usage

1. The codes are developed on Ubuntu20.04, do not run this project on Windows.
2. `$ git clone https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot.git`, then cd to this repo's directory.
3. `$ ./make_and_run` to make both codes of controller and simulation, then start simulation. 
4. When both codes in `nabo_core/` and `nabo_mujoco/` are maked，simply `$ ./run_sim.sh` to only start simulation.
5.  Edit `nabo_mujoco/000.ini` to adjust the parameters. And`nabo_mujoco/zzz.txt` stores the log file during simulation if needed. 

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

