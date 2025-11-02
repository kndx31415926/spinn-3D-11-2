%% spinn3d_run_v2.m
% 一键：生成数据(带进度) -> 每10条轨迹保存一次 -> 累计到主表 -> (可选)训练
% 依赖：spinn3d_make_dataset_v2.m, spinn3d_train_v2.m, spinn3d_params_block.m

%% ===== 可配参数（不传参，直接改这里） =====
N_TRAJ_ADD        = 2000;        % 本次要新增的轨迹数
CHUNK_TRAJ        = 10;          % 每多少条轨迹保存一次（固定为10）
DO_TRAIN_AFTER    = true;        % 追加完成后是否立刻训练：true/false
DT_GEN            = 0.02;        % 生成用 dt
T_FINAL_GEN       = 6.0;         % 每条轨迹时长（秒）

%% ===== 路径与输出文件 =====
MASTER_DIR         = 'data';
CHK_DIR            = fullfile(MASTER_DIR,'chk');    % 分片checkpoint目录
MASTER_TBL_VAR     = 'SPINN3D_DATASET';             % 运行空间里的变量名（table）
MASTER_TBL_FILE    = fullfile(MASTER_DIR,'spinn3d_dataset_table_master.mat');
MASTER_DATASET_MAT = fullfile(MASTER_DIR,'spinn3d_v2_dataset_master.mat'); % 聚合 X/Y 给训练用
MODEL_MAT          = fullfile(MASTER_DIR,'spinn3d_v2_net.mat');            % 训练输出模型

ensure_dir(MASTER_DIR);
ensure_dir(CHK_DIR);

%% ===== 前置检查 =====
assert(exist('spinn3d_make_dataset_v2','file')==2, '缺少 spinn3d_make_dataset_v2.m');
assert(exist('spinn3d_train_v2','file')==2,       '缺少 spinn3d_train_v2.m');
assert(exist('spinn3d_params_block.m','file')==2, '缺少 spinn3d_params_block.m');

%% ===== 公共参数（由 params_block 提供） =====
run spinn3d_params_block   % 提供 params, caps, Kp, Ki, Kd, pid_opts, sim, gate 等

% 生成器基础设置（采样范围单位：度）
simgen_base = struct('dt', DT_GEN, 't_final', T_FINAL_GEN, ...
                     'Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...      % ★ 透传 Ki
                     'pid_opts', pid_opts, ...              % ★ 透传 pid 选项（anti-windup等）
                     'damping', sim.damping, ...
                     'q0_range',   [-30 30; -20 20; -20 20; -20 20], ...
                     'qref_range', [ 15 45;  10 30; -40 -10;  10 40]);

%% ===== 载入历史主表与聚合 X/Y =====
[T_master, Xagg, Yagg] = load_master(MASTER_TBL_FILE, MASTER_TBL_VAR, MASTER_DATASET_MAT);
N0_hist = height(T_master);

%% ===== 进度头信息 =====
fprintf('== [1/3] 生成数据: 计划 N_TRAJ=%d, dt=%.3f, T=%.1f, 每 %d 条保存一次 ==\n', ...
        N_TRAJ_ADD, DT_GEN, T_FINAL_GEN, CHUNK_TRAJ);

t0 = tic; added_rows_total = 0;
num_chunks = ceil(N_TRAJ_ADD / CHUNK_TRAJ);

for c = 1:num_chunks
    curN = min(CHUNK_TRAJ, N_TRAJ_ADD - (c-1)*CHUNK_TRAJ);
    if curN <= 0, break; end

    % === 生成一个分片 ===
    simgen = simgen_base; simgen.nTraj = curN;
    ds = spinn3d_make_dataset_v2(params, caps, simgen, gate);  % ds.X (M×D), ds.Y (M×K)

    % === 分片落盘（checkpoint） ===
    tag = datestr(now,'yyyymmdd-HHMMSS');
    shard_file = fullfile(CHK_DIR, sprintf('spinn3d_v2_dataset-chunk%03d-%s.mat', c, tag));
    save(shard_file, '-struct', 'ds', '-v7.3');

    % === 追加到主表 & 聚合 X/Y ===
    [T_master, Xagg, Yagg, M_new] = append_to_master(T_master, Xagg, Yagg, ds);
    added_rows_total = added_rows_total + M_new;

    % === 保存主表/聚合（每片一次） ===
    SPINN3D_DATASET = T_master; %#ok<NASGU>
    save(MASTER_TBL_FILE, 'SPINN3D_DATASET', '-v7.3');

    % 训练用聚合数据文件里保存变量名必须是 X/Y
    X = Xagg; Y = Yagg; %#ok<NASGU>
    save(MASTER_DATASET_MAT, 'X','Y','caps','params','gate','-v7.3');

    % === 打印进度 ===
    elapsed = toc(t0);
    eta = elapsed / c * (num_chunks - c);
    fprintf('[%d/%d] +%d 轨迹 -> 样本 +%d 行, 累计 %d 行 | 用时 %.1fs | 预计剩余 %.1fs\n', ...
            c, num_chunks, curN, M_new, height(T_master), elapsed, eta);
end

%% ===== 汇总信息 =====
fprintf('== [2/3] 追加完成：历史 %d 行 + 新增 %d 行 -> 现有 %d 行 ==\n', ...
        N0_hist, added_rows_total, height(T_master));
fprintf('已保存表到：%s\n', MASTER_TBL_FILE);
fprintf('已保存聚合数据到：%s（X: %d×%d, Y: %d×%d）\n', ...
        MASTER_DATASET_MAT, size(Xagg,1), size(Xagg,2), size(Yagg,1), size(Yagg,2));

%% ===== (可选) 训练 =====
if DO_TRAIN_AFTER
    fprintf('== [3/3] 训练模型 ==\n');
    spinn3d_train_v2(MASTER_DATASET_MAT, MODEL_MAT);
    fprintf('模型已保存：%s\n', MODEL_MAT);
else
    fprintf('== [3/3] 跳过训练（DO_TRAIN_AFTER=%s） ==\n', string(DO_TRAIN_AFTER));
end

fprintf('完成。\n');

%% ====== 本地函数 ======
function ensure_dir(d)
    if exist(d,'dir')~=7, mkdir(d); end
end

function [T, Xagg, Yagg] = load_master(tbl_file, tbl_var, agg_file)
    T = table(); Xagg = []; Yagg = [];
    % 运行空间优先
    if evalin('base', sprintf('exist(''%s'',''var'')==1', tbl_var))
        T = evalin('base', tbl_var);
    elseif exist(tbl_file,'file')
        S = load(tbl_file);
        if isfield(S, tbl_var), T = S.(tbl_var); end
        if isempty(T) && isfield(S, 'SPINN3D_DATASET')  % 兼容旧字段名
            T = S.SPINN3D_DATASET;
        end
    end
    if exist(agg_file,'file')
        S2 = load(agg_file);
        if isfield(S2,'X'), Xagg = S2.X; end
        if isfield(S2,'Y'), Yagg = S2.Y; end
    end
end

function [T, Xagg, Yagg, M_new] = append_to_master(T, Xagg, Yagg, ds)
    [M_new, D] = size(ds.X);  K = size(ds.Y,2);
    % 组装成 table（带列名）
    xnames = strcat("x", string(1:D));
    if K==5, ynames = ["alpha1","alpha2","alpha3","alpha4","gate"];
    else,    ynames = cellstr("y"+string(1:K)); end
    Tnew = array2table([ds.X ds.Y], 'VariableNames', [cellstr(xnames) cellstr(ynames)]);

    if isempty(T)
        T = Tnew;
    else
        assert(width(T)==width(Tnew), '表结构不一致（历史=%d, 新=%d）。', width(T), width(Tnew));
        T = [T; Tnew]; %#ok<AGROW>
    end

    if isempty(Xagg), Xagg = ds.X; else, Xagg = [Xagg; ds.X]; end %#ok<AGROW>
    if isempty(Yagg), Yagg = ds.Y; else, Yagg = [Yagg; ds.Y]; end %#ok<AGROW>

    % 同步到运行空间
    assignin('base', 'SPINN3D_DATASET', T);
end
