%% 第二章配套源代码 / Chapter 2 companion code
% 中文图书：《非结构化场景自动驾驶轨迹规划技术》
% English translation of the book title:
%   Trajectory Planning Techniques for Autonomous Driving in Unstructured Environments
% 本书为中文图书。上述英文书名仅用于说明中文书名，并非英文版图书。
% This is a Chinese-language book; the English title above is a translation.
%
% 本代码配套第二章《面向狭窄泊车位场景的自主泊车轨迹规划基本方法》。
% 请结合本书第二章阅读车辆模型、三角面积避障约束、直接法离散与初始解构造。
% Read Chapter 2 for the mathematical model and numerical solution method.
%
% 使用本代码开展研究时，请在论文、技术报告等成果中引用以下 KBS 2015 论文：
% If you use this code in your research, please cite:
%   Bai Li and Zhijiang Shao,
%   "A unified motion planning method for parking an autonomous vehicle
%    in the presence of irregularly placed obstacles,"
%   Knowledge-Based Systems, vol. 86, pp. 11-20, 2015.
%   DOI: https://doi.org/10.1016/j.knosys.2015.04.016
%
% 运行环境：Windows 64 位，MATLAB R2021b 或更新版本，Navigation Toolbox。
% 选择 case_id = 1、2、3、4 之一，然后直接运行本文件；默认第 4 例。
% 统一使用 200 个配置点，其他车辆参数对应第二章表 2-1。
% MATLAB 调用 AMPL/IPOPT 的 .run 命令文件，通过本地 TXT 交换结果，无须 API。
% 运行后显示两张静态图：
%   1. 最优车身足迹、HA* 初始轨迹与最优轨迹（先画足迹，再画轨迹）。
%   2. 最优状态量与控制量的时间历程。
% result 保存数值结果；AmplResults 保存 TXT 和求解日志。不生成视频或 GIF。
% 受保护的初始化函数以 .p 文件提供，运行时不需要同名 .m 文件。
% 文件说明、总体架构、算例说明与 BibTeX 引用见 README.md。
% Repository: https://github.com/libai1943/KBS2015

case_id = 4;                       % 1, 2, 3, or 4
result = RunParkingDemo(case_id);  % Two figures; numerical results in result.
