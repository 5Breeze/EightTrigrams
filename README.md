# EightTrigrams (八卦电子罗盘)
<img width="591" height="412" alt="image" src="https://github.com/user-attachments/assets/d9a9b193-0e1b-4cb8-a546-86faaf8e68b8" />

## Project Overview (项目概述)
The EightTrigrams project (repository: https://github.com/5Breeze/EightTrigrams) is an innovative hardware initiative that integrates traditional Zhouyi Bagua (Eight Trigrams) culture with modern electronic technology, drawing inspiration from the "gossip-electronic-compass" project on the open-source hardware platform oshwhub. It aims to combine the orientation detection capability of electronic compasses with the Bagua directional system, digitizing and hardware-izing traditional directional culture while preserving its core essence, and making the identification and display of Bagua directions more intuitive and accurate through electronic technology.
八卦电子罗盘（EightTrigrams）项目（仓库地址：https://github.com/5Breeze/EightTrigrams）是一款融合传统周易八卦文化与现代电子技术的创新型硬件项目，参考开源硬件平台oshwhub上的“gossip-electronic-compass”项目理念开发。该项目旨在将电子罗盘的方位检测能力与八卦方位体系结合，在保留传统文化内核的前提下，实现传统方位文化的数字化、硬件化呈现，通过电子技术让八卦方位的识别、展示更直观、精准。

## Core Functions (核心功能)
1. **Precise Orientation Detection (方位精准检测)**: Real-time acquisition of azimuth angles based on electronic compass (magnetometer) modules, accurately identifying the current physical orientation and mapping it to the directional system corresponding to the Zhouyi Eight Trigrams (Qian, Kun, Zhen, Xun, Kan, Li, Gen, Dui).
基于电子罗盘（磁力计）模块实现方位角的实时采集，精准识别当前物理方位，并映射到周易八卦对应的方位体系（乾、坤、震、巽、坎、离、艮、兑）。
2. **Bagua Direction Mapping (八卦方位映射)**: Built-in logic for the correspondence between Eight Trigrams and directions (e.g., Kan for North, Li for South, Zhen for East, Dui for West), converting angle data collected by the electronic compass into corresponding Bagua symbols and directional information.
内置八卦与方位的对应逻辑（如坎为北、离为南、震为东、兑为西等），将电子罗盘采集的角度数据转化为对应的八卦符号与方位信息。
3. **Intuitive Information Display (直观信息展示)**: Real-time display of Bagua symbols, azimuth angles, and hexagram interpretations corresponding to the current orientation through hardware peripherals (such as OLED screens, LED light groups, etc.), supporting visual interaction.
通过硬件外设（如OLED屏、LED灯组等）实时显示当前方位对应的八卦符号、方位角、卦象释义等信息，支持可视化交互。
4. **Scalable Adaptation (可扩展适配)**: The project's code and hardware circuits are open-source designs, supporting functional expansion according to needs (e.g., voice output of hexagram cultural interpretations, mobile phone Bluetooth data synchronization, adaptation of different-sized display screens, etc.).
项目代码与硬件电路均为开源设计，支持根据需求扩展功能（如卦象文化解读语音输出、手机蓝牙同步数据、不同尺寸显示屏适配等）。

## Technical Architecture (技术架构)
### Hardware Layer (硬件层面)
- **Main Control Unit (主控单元)**: Low-cost, easy-to-use microcontrollers (such as STM32, Arduino, ESP32, etc.) are used as the core, responsible for data collection, logical operation, and peripheral control;
采用低成本、易上手的微控制器（如STM32、Arduino、ESP32等）作为核心，负责数据采集、逻辑运算与外设控制；
- **Sensor Module (传感器模块)**: Integrates magnetometers (such as HMC5883L, QMC5883L) for orientation detection, with an optional accelerometer for tilt compensation to improve orientation detection accuracy;
集成磁力计（如HMC5883L、QMC5883L）实现方位检测，可选配加速度计进行倾角补偿，提升方位检测精度；
- **Display Module (显示模块)**: Supports OLED screens, nixie tubes, or dot matrix screens for displaying Bagua symbols and directional data;
支持OLED屏、数码管或点阵屏，用于展示八卦符号、方位数据；
- **Power Supply Module (供电模块)**: Compatible with USB power supply and lithium battery power supply to meet portable usage needs.
兼容USB供电、锂电池供电，满足便携使用需求。

### Software Layer (软件层面)
- **Low-Level Drivers (底层驱动)**: Encapsulates sensor driver programs to achieve stable collection and calibration of azimuth angles (supporting magnetic field calibration to offset environmental magnetic interference);
封装传感器驱动程序，实现方位角的稳定采集与校准（支持磁场校准，抵消环境磁干扰）；
- **Core Algorithms (核心算法)**: Implements the mapping logic from angle data to Bagua directions, including the definition of the coordinate system for Bagua directions and the division of angle intervals;
实现角度数据到八卦方位的映射逻辑，包含八卦方位的坐标体系定义、角度区间划分；
- **Interaction Logic (交互逻辑)**: Provides interactive methods such as key operation and automatic refresh for user-friendly operation.
提供按键操作、自动刷新等交互方式，保障操作便捷性。
