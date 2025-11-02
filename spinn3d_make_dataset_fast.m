% SPINN3D_MAKE_DATASET_FAST —— 生成快达监督数据集
clear; clc;
if ~(exist('params','var')&&exist('sim','var')&&exist('caps','var')&&exist('Kp','var')&&exist('Kd','var')&&exist('Ki','var')&&exist('gate','var'))
    run('spinn3d_params_block.m');   % 复用统一参数、caps/gate/sim配置
end
[robot, kin] = spinn3d_robot(params);
pid = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);   % 方向基线（与 demo 相同）  :contentReference[oaicite:6]{index=6}

EPISODES   = 60;            % 多少条轨迹
T_FINAL    = 6.0;           % 每条仿真秒数
sim0       = sim; sim0.t_final = T_FINAL; sim0.stop_on_first_hit = false; sim0.caps = caps;

X = [];   % features  [*,42]
T = [];   % targets   [*, (n+1+n)] = [alpha*, g*, w]
n = numel(sim.q0);

for e = 1:EPISODES
    % 随机目标/初值
    sim0.q0    = sim.q0 + deg2rad( (randn(1,n).* [20 15 15 15]) );
    sim0.dq0   = zeros(1,n);
    sim0.q_ref = sim.q_ref + deg2rad( (randn(1,n).* [5 5 5 5]) );

    % 仅用 PID 方向（和 demo 一致）驱动，让系统产生有效状态分布
    ctl = @(t,q,dq,qr,dqr,robot) pid.step(t,q,dq,qr,dqr,robot);
    log = spinn3d_simulate(robot, ctl, sim0);   % 功率在积分前，caps 顺序和你线上口径一致  :contentReference[oaicite:7]{index=7}

    for k = 1:numel(log.t)
        qk  = log.q(k,:); dqk = log.dq(k,:);
        qr  = sim0.q_ref; dqr = sim0.dq_ref;

        % 特征（42 维）
        fk = spinn3d_features_v2(log.t(k), qk, dqk, qr, dqr, robot, kin);

        % 快达标签 + 进度权重
        [aStar, gStar, dbg] = spinn3d_oracle_fast(qk, dqk, qr, dqr, robot, kin, caps, gate, pid);
        wk = max(0, -log.tau_raw(k,:).*dqk);     % 用 tau_ref(=tau_raw) 与 dq 计算推进权重（一致口径）

        if any(isnan([fk aStar gStar wk]))
            continue;
        end
        X(end+1,:) = fk;                         %#ok<AGROW>
        T(end+1,:) = [aStar, gStar, wk];        %#ok<AGROW>
    end
    fprintf('Episode %d/%d -> samples: %d\n', e, EPISODES, size(X,1));
end

DATA_FILE = 'spinn3d_fast_dataset.mat';
save(DATA_FILE, 'X', 'T', 'params', 'caps', 'gate', 'sim');
fprintf('Saved dataset: %s  (X: %dx%d, T: %dx%d)\n', DATA_FILE, size(X,1), size(X,2), size(T,1), size(T,2));
