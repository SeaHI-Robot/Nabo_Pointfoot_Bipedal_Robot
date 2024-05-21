<br>

<p align='center'><img src="./assets.README/nabopointfoot.png" alt="nabo_pointfoot"  width="550"/>
</a>
</p>

<p align="center">
"<b>Na</b>ughty <b>bo</b>y Pointfoot" 🤖
</p>
<p align="center">
    <a href="https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot" target="__blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot?style=social"></a>
</p>
<p align="center">
  <a href="http://www.bilibili.com/">视频</a> | <a href="https://github.com/tryingfly/nabo">Nabo</a> | <a href="https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot/blob/nabo_pointfoot_opensourced/README.md">English</a> 
</p>



<br>

<br>

## 简介

- **Nabo Pointfoot** 是一个点式双足的仿真框架，使用MuJoCo仿真引擎，代码全部由C++构建 

- **Nabo Pointfoot** 参考 [**nabo**]项目(https://github.com/tryingfly/nabo),  一个 12自由度的双足机器人的控制框架

- **MPC+WBIC** 组成了平衡控制器，控制器代码和仿真代码隔离，易于维护
- **Nabo Pointfoot** 行走身高0.45米, 最大行走速度0.95m/s.

<br>

## 特点

- 💻纯C++构建
- ⚡️计算效率高, MPC和WBIC的QP使用 [eiquadprog](https://github.com/stack-of-tasks/eiquadprog)求解
- 🤹交互式 [MuJoCo](https://github.com/google-deepmind/mujoco) UI界面
- 📝相比原[nabo](https://github.com/tryingfly/nabo)项目注释丰富


<p align='center'><img src="./assets.README/model_comparison.png" alt="nabo_pointfoot"  width="500"/>
</a>
</p>
<p align="center">
和 <b>LIMX P1</b> & <b>Unitree H1</b> 尺寸对比 🤖
</p>


<br>

## 使用说明

1. 本项目在Ubuntu20.04上开发完成, 不要在Windows上跑.
2. `$ git clone https://github.com/SeaHI-Robot/Nabo_Pointfoot_Bipedal_Robot.git`, 之后cd到该仓库的路径下.
3. `$ ./make_and_run` 来编译控制器和仿真的代码工程, 并打开方针UI界面. 
4. 完成编译 `nabo_core/` 和 `nabo_mujoco/` 目录下的仿真工程后，运行 `$ ./run_sim.sh` 可以只打开仿真界面.
5.  编辑 `nabo_mujoco/000.ini` 中的内容调整参数.。`nabo_mujoco/zzz.txt` 负责储存仿真过程中的log文件，当启动log选项时。

<br>

## 讨论

有任何问题请提 **Issue**, 或者加 QQ 群 609601974，该群为nabo原项目作者创建. 

<br>

## 致谢

1. [nabo](https://github.com/tryingfly/nabo),
2. [MIT mini-cheetah](https://arxiv.org/abs/1909.06586),
3. [MuJoCo](https://mujoco.org/) Simulator,

4. and SUSTech/南方科技大学🌈


<p align='center'><img src="./assets.README/SUSTech-zh.png" alt="SUSTech"  width="350"/>
</a>
</p>
