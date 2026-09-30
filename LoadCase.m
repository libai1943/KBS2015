function LoadCase()
% 固定装载 KBS 四个算例（Case 1~4）的起点、终端车位与障碍物
% Four scenarios from Li and Shao, Knowledge-Based Systems 86 (2015), 11-20.
% Target poses below are the fixed-pose choices in this code release.
global params_

cid = params_.user.case_id;  % 1,2,3,4
params_.obstacle = struct();
params_.task     = struct();

switch cid
    %% ------------------------- Case 1 -------------------------
    case 1
        % 起点（两点边值条件中给出）
        params_.task.x0     = -10.0;
        params_.task.y0     =  -3.0;
        params_.task.theta0 = pi;

        % 用于本演示的固定终端位姿
        params_.task.xf = 1.3; params_.task.yf = 0; params_.task.thetaf = pi;

        % 障碍物（P1, P2）
        obstacles = cell(2,1);
        % P1
        obstacles{1}.x = [-3.390, -3.577, -8.100, -7.913, -3.390];
        obstacles{1}.y = [ 0.601, -1.094, -0.594,  1.100,  0.601];
        % P2
        obstacles{2}.x = [ 7.156,  7.576,  2.844,  2.424,  7.156];
        obstacles{2}.y = [ 1.193, -0.646, -1.727,  0.111,  1.193];

        params_.obstacle.num_obs = numel(obstacles);
        params_.obstacle.obs     = obstacles;

        %% ------------------------- Case 2 -------------------------
    case 2
        params_.task.x0     = -10.0;
        params_.task.y0     =   3.4; % Original 3.0 overlaps P1 with the Table 2-1 vehicle.
        params_.task.theta0 =   0.0;

        % Original xf=1.8 overlaps P2 with the Table 2-1 vehicle.
        params_.task.xf = 1.5; params_.task.yf = 0; params_.task.thetaf = pi;

        obstacles = cell(2,1);
        % P1
        obstacles{1}.x = [-5.998, -7.431, -9.894, -8.461, -5.998];
        obstacles{1}.y = [-1.462, -2.385,  1.441,  2.364, -1.462];
        % P2
        obstacles{2}.x = [ 7.492,  7.348,  2.508,  2.652,  7.492];
        obstacles{2}.y = [ 1.034, -0.846, -0.474,  1.406,  1.034];

        params_.obstacle.num_obs = numel(obstacles);
        params_.obstacle.obs     = obstacles;

        %% ------------------------- Case 3 -------------------------
    case 3
        params_.task.x0     = -10.0;
        params_.task.y0     =   6.0;
        params_.task.theta0 =   0.0;

        params_.task.xf = 0; params_.task.yf = 2; params_.task.thetaf = 1.45*pi;

        obstacles = cell(2,1);
        % P1
        obstacles{1}.x = [-1.944, -3.650, -3.656, -1.951, -1.944];
        obstacles{1}.y = [-2.353, -2.355,  2.195,  2.197, -2.353];
        % P2
        obstacles{2}.x = [ 2.175,  0.375,  1.825,  3.625,  2.175];
        obstacles{2}.y = [-2.585, -2.021,  2.611,  2.047, -2.585];

        params_.obstacle.num_obs = numel(obstacles);
        params_.obstacle.obs     = obstacles;

        %% ------------------------- Case 4 -------------------------
    case 4
        % 起点为给定姿态（来自表2）
        params_.task.x0     =  8.0;
        params_.task.y0     =  6.0;
        params_.task.theta0 = -0.2 + pi;

        params_.task.xf = 1.5; params_.task.yf = 0.2; params_.task.thetaf = pi;

        obstacles = cell(4,1);
        % P1
        obstacles{1}.x = [-4.750, -4.702, -9.250, -9.298, -4.750];
        obstacles{1}.y = [ 0.917, -0.787, -0.917,  0.787,  0.917];
        % P2
        obstacles{2}.x = [ 6.193,  7.443,  3.807,  2.557,  6.193];
        obstacles{2}.y = [ 2.425,  1.012, -2.204, -0.791,  2.425];
        % P3
        obstacles{3}.x = [-0.845,  0.620,  2.945,  1.480, -0.845];
        obstacles{3}.y = [ 7.420,  8.291,  4.380,  3.509,  7.420];
        % P4
        obstacles{4}.x = [ 1.471,  1.377, -3.471, -3.377,  1.471];
        obstacles{4}.y = [-1.679, -3.563, -3.321, -1.437, -1.679];

        params_.obstacle.num_obs = numel(obstacles);
        params_.obstacle.obs     = obstacles;

    otherwise
        error('Unknown case_id = %s. 可选：1, 2, 3, 4。', mat2str(cid));
end

% 起始和终止速度均为零。
params_.task.v0 = 0; params_.task.vf = 0;

end
