% SPINN3D_DEMO_NN_V2_PBUD —— NN_v2 推理（无动画）+ 在“Total Power”图中叠加 P_bud(t)
% 说明：和 spinn3d_demo_nn_v2 完全一致，只多了一步——计算并叠加门控预算曲线 P_bud(t)
% 依赖：spinn3d_params_block.m, spinn3d_robot.m, spinn3d_controller_pid.m,
%       spinn3d_controller_nn_v2.m, spinn3d_features_v2.m, spinn3d_simulate.m,
%       spinn3d_static_figure.m, spinn3d_power_check.m, spinn3d_energy_gate.m

clear; clc;
% 统一参数块
if ~(exist('params','var')&&exist('sim','var')&&exist('caps','var')&&exist('Kp','var')&&exist('Kd','var')&&exist('Ki','var'))
    run('spinn3d_params_block.m');
end
if ~(exist('MODEL_MAT','var')&&exist(MODEL_MAT,'file')==2)
    error('未找到 NN 模型文件：%s（请先运行 spinn3d_run_v2 训练）', MODEL_MAT);
end
if ~exist('gate','var'), gate = struct('enable', true, 'Pmin', 20, 'kE', 5.0); end

% 构建对象
model = load(MODEL_MAT);
[robot, kin] = spinn3d_robot(params);
pid  = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);
nnctl= spinn3d_controller_nn_v2(model, @spinn3d_features_v2, caps, gate);

% 仿真
controllerFcn = @(t,q,dq,qr,dqr,robot) nnctl.step(t,q,dq,qr,dqr,robot,kin,pid);
log = spinn3d_simulate(robot, controllerFcn, sim);

% 体检 + 出图（保持与你现有风格一致，不显示 P_raw）
spinn3d_power_check(log);
opts_fig = struct('q_goal', sim.q_ref, 'goal_radius',0.03, 'Pmax', caps.Pmax, ...
                  'trim_t0',0.02,'robust_pct',0.995,'show_raw', false);
spinn3d_static_figure(kin, log, opts_fig);

% === 计算并叠加 P_bud(t) ===
if exist('spinn3d_energy_gate','file')==2 && isstruct(gate) && isfield(gate,'enable') && gate.enable
    Nt = numel(log.t);
    Pbud = zeros(Nt,1);
    for k = 1:Nt
        [Pbud(k), ~] = spinn3d_energy_gate(robot, log.q(k,:), log.dq(k,:), sim.q_ref, caps, gate);
    end
    % 找到“Total Power”子图（标题含 Total Power）
    fig = gcf;
    ax_all = findall(fig,'Type','axes');
    axP = [];
    for a = transpose(ax_all(:))
        ttl = ""; yl = "";
        if ~isempty(get(a,'Title')),  ttl = get(get(a,'Title'),'String'); end
        if ~isempty(get(a,'YLabel')), yl  = get(get(a,'YLabel'),'String'); end
        if (ischar(ttl) && contains(ttl,'Total Power')) || (isstring(ttl) && contains(ttl,"Total Power"))
            axP = a; break;
        end
        if isempty(axP) && ((ischar(yl) && contains(yl,'Power1111')) || (isstring(yl) && contains(yl,"Power")))
            % 兜底：有些版本没有标题，就用 Y 轴名称含 Power 的那个
            axP = a;
        end
    end
    if ~isempty(axP)
        hold(axP,'on');
        plot(axP, log.t, Pbud, ':', 'LineWidth', 1.5, 'DisplayName', 'P_{bud}(t)');
        legend(axP,'show','Location','best');
        hold(axP,'off');
    else
        warning('未能定位到“Total Power”子图，跳过 P_{bud}(t) 叠加。');
    end

    % 控制台打印利用率
    U_tot  = mean(log.P)/caps.Pmax;
    U_gate = mean( log.P ./ max(Pbud,1e-9) );
    U95    = prctile( log.P ./ max(Pbud,1e-9), 95 );
    fprintf('Util_total = %.1f%% of Pmax,  Util_gate = %.1f%% of Pbud (P/Pbud @95%% = %.1f%%)\n', ...
            100*U_tot, 100*U_gate, 100*U95);
else
    fprintf('门控禁用或缺少 spinn3d_energy_gate，未叠加 P_{bud}(t)。\n');
end
