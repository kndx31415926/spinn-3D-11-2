function [alpha_star, g_star, dbg] = spinn3d_oracle_fast(q, dq, q_ref, dq_ref, robot, kin, caps, gate, pid)
% 生成“首达更快”风格的监督标签 (alpha*, g*).
% 约束与在线一致：逐轴/总功率上限，功率在积分前按 dq(t) 口径计算。
% 输出：
%   alpha_star : 1×n, 单纯形
%   g_star     : 标量∈[0,1]
%   dbg        : 调试信息（可选）

    q  = q(:)'; dq = dq(:)'; q_ref = q_ref(:)'; dq_ref = dq_ref(:)'; 
    n  = numel(q); epsw = 1e-6;

    % 1) 方向扭矩（与在线一致；默认 PID，可改为任务空间 PD）
    [tau_ref, ~] = pid.step(0, q, dq, q_ref, dq_ref, robot);  % 方向基线（保持和 demo 一致）
    tau_ref = tau_ref(:)';

    % 2) 能量门控上界 g_H（缺省=1）
    gH = 1.0;
    try
        [~, gHtmp, ~] = spinn3d_energy_gate(q, dq, q_ref, dq_ref, robot, gate); %#ok<NASGU>
        if ~isempty(gHtmp), gH = min(max(gHtmp,0),1); end
    catch
        gH = 1.0;
    end
    g_star = gH;   % 外域可再加底座油门：g_star = max(g_floor, gH)（此处先保持与线上一致）

    % 3) 推进贡献权重（谁更快降低 ΔH / 误差就优先）
    w = max(0, -tau_ref .* dq);        % |τ_ref * dq| 的推进部分（≤0 的置 0）
    if sum(w) <= 1e-9, w = abs(tau_ref .* dq); end
    if sum(w) <= 1e-12, w = ones(1,n); end
    alpha = w / sum(w);                % 初始份额（单纯形）

    % 4) 将总预算 Pbud 分到各轴，并受逐轴额定/扭矩上限裁剪
    Pbud   = g_star * caps.Pmax;       % 总功率预算
    Prated = expand_(caps.Prated, n);
    tauMax = expand_(caps.tauMax, n);

    Paxis  = alpha * Pbud;
    Paxis  = min(Paxis, Prated);                         % 逐轴额定功率
    Pcap   = min(Paxis, tauMax .* abs(dq + epsw));       % 逐轴功率极限（受|dq|影响）
    Pref   = abs(tau_ref .* dq);                         % 参考方向的“需求功率”
    deficit= max(0, Pref - Pcap);                        % 该轴需求超过可供 → 需要“腾挪”
    freed  = sum(deficit);

    if freed > 0
        mask = (Pref <= Pcap);                           % 仍有余量的轴
        if any(mask)
            w2 = w .* mask;  s2 = sum(w2);  if s2<=1e-12, w2 = mask/sum(mask); else, w2 = w2/s2; end
            addP = freed * w2;                           % 释放预算按推进权重分给未饱和轴
            Paxis = Paxis + addP;
            % 再次裁剪到额定功率上限
            Paxis = min(Paxis, Prated);
        end
    end

    alpha_star = Paxis / max(Pbud, 1e-12);
    s = sum(alpha_star); if s<=0, alpha_star = ones(1,n)/n; else, alpha_star = alpha_star/s; end

    if nargout > 2
        dbg = struct('w',w,'Pbud',Pbud,'Paxis',Paxis,'Pcap',Pcap,'Pref',Pref,'deficit',deficit);
    end
end

function y = expand_(x, n)
    if numel(x)==1, y = repmat(x,1,n); else, y = x(:)'; end
end
