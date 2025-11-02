% SPINN3D_PARAMS_BLOCK  —— 统一参数块（含 SPINN 能量门控）

% === 机器人参数 ===
params = struct();
params.L           = [0.24 0.214 0.324];
params.gravity     = [0 0 -9.81];
params.base_mass   = 6.0;
params.base_radius = 0.15;
params.base_height = 0.04;

% === PID 增益 ===
Kp = [120 100 80 60];
Ki = [1 1 1 1];
Kd = [6 5 4 3];
pid_opts = struct('useGravityComp',true, 'uMax',[120 90 70 50], 'antiWindup',true, 'Imax',2);

% === 功率/扭矩上限 ===
caps = struct();
caps.Pmax   = 50;                     % 注意：这里是总功率上限；按需调整
caps.Prated = [100 100 100 60];
caps.tauMax = [120  90  70  50];
caps.useTotalPowerCap = true;
caps.useAxisPowerCap  = true;

% === SPINN 能量门控（训练监督与推理同参） ===
gate = struct('enable', true, 'Pmin', 20, 'kE', 5.0);   % 近目标自动收权

% === 仿真参数 ===
sim = struct();
sim.goal_radius = 0.07;          % [m] 末端空间到达判定半径（默认 3cm）
sim.stop_on_first_hit = true;    % 首次进入目标区即停止仿真/绘图
sim.dt      = 0.002;
sim.t_final = 20;
sim.q0      = deg2rad([0 0 0 0]);
sim.dq0     = [0 0 0 0];
sim.q_ref   = deg2rad([30, 20, 35, 60]);
sim.dq_ref  = [0 0 0 0];
sim.damping = [0.06 0.05 0.05 0.04];
sim.jointLimit = struct('deadband_deg',0.5,'freezeInward',true);
sim.integrator = 'semi';
sim.caps = caps;

% === 模型文件 ===
MODEL_MAT = 'spinn3d_v2_net_fast.mat'
