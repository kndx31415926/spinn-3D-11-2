function ds = spinn3d_make_dataset_v2(params, caps, simgen, gate)
% Generate supervised dataset for NN_v2 with energy-aware total power gate.
% Teacher controller: PID (from simgen.Kp/Ki/Kd and simgen.pid_opts), gravity compensation enabled.
% gate: struct('enable',true,'Pmin',20,'kE',5.0). If missing/disabled -> falls back to fixed Pmax.

if nargin < 1 || isempty(params), params = struct(); end
if nargin < 2 || isempty(caps),   caps   = struct('Pmax',200,'Prated',[100 100 100 60],'tauMax',[120 90 70 50]); end
if nargin < 3, simgen = struct(); end
if nargin < 4, gate = struct('enable',true,'Pmin',20,'kE',5.0); end

% ===== defaults =====
if ~isfield(simgen,'nTraj'),     simgen.nTraj   = 100;  end
if ~isfield(simgen,'t_final'),   simgen.t_final = 6.0;  end
if ~isfield(simgen,'dt'),        simgen.dt      = 0.01; end
if ~isfield(simgen,'q0_range'),  simgen.q0_range   = [-30 30; -20 20; -20 20; -20 20]; end % [deg]
if ~isfield(simgen,'qref_range'),simgen.qref_range = [ 15 45;  10 30; -40 -10;  10 40]; end % [deg]
if ~isfield(simgen,'Kp'),        simgen.Kp = [120 100 80 60]; end
if ~isfield(simgen,'Kd'),        simgen.Kd = [6 5 4 3];        end
if ~isfield(simgen,'Ki'),        simgen.Ki = zeros(size(simgen.Kp)); end   % default: PD if not provided
if ~isfield(simgen,'damping'),   simgen.damping = [0.06 0.05 0.05 0.04]; end
if ~isfield(simgen,'pid_opts'),  simgen.pid_opts = struct('useGravityComp',true,'uMax',[]); end

% ===== robot & teacher PID =====
[robot, kin] = spinn3d_robot(params);
Ki_use = simgen.Ki;
opts   = simgen.pid_opts;
pid    = spinn3d_controller_pid(simgen.Kp, Ki_use, simgen.Kd, opts);  % ★ teacher=PID (uses Ki)

% ===== probe feature dim =====
q0p = zeros(1,4); dq0p = zeros(1,4); qrefp = zeros(1,4); dqrefp = zeros(1,4);
tau_probe = pid.step(0, q0p, dq0p, qrefp, dqrefp, robot);
x_probe   = spinn3d_features_v2(robot, kin, q0p, dq0p, qrefp, dqrefp, tau_probe);
D = numel(x_probe);

% ===== prealloc =====
M_est = max(1, round(simgen.nTraj * (simgen.t_final/simgen.dt + 1)));
X = zeros(M_est, D, 'like', x_probe);
Y = zeros(M_est, 5);
m = 0;

% ===== generation =====
for tr = 1:simgen.nTraj
    q0   = deg2rad(simgen.q0_range(:,1)'  + (simgen.q0_range(:,2)  - simgen.q0_range(:,1))'.*rand(1,4));
    dq0  = zeros(1,4);
    qref = deg2rad(simgen.qref_range(:,1)' + (simgen.qref_range(:,2) - simgen.qref_range(:,1))'.*rand(1,4));
    dqref= [0 0 0 0];

    q = q0; dq = dq0;
    for t = 0:simgen.dt:simgen.t_final
        % teacher torque (PID + gravity)
        tau_raw = pid.step(t, q, dq, qref, dqref, robot);

        % features
        x = spinn3d_features_v2(robot, kin, q, dq, qref, dqref, tau_raw);

        % === total power budget (energy-aware) ===
        [Pbud, gH] = spinn3d_energy_gate(robot, q, dq, qref, caps, gate);
        capH = caps; capH.Pmax = Pbud;

        % Oracle allocation under Pbud -> labels (alpha*, g*)
        [alpha, ~] = spinn3d_oracle_alloc_v2(tau_raw, dq, capH);
        g = gH;

        % store
        m = m + 1;
        if m > size(X,1)
            X = [X; zeros(M_est, D, 'like', x_probe)]; %#ok<AGROW>
            Y = [Y; zeros(M_est, 5)];                 %#ok<AGROW>
        end
        X(m,:) = x;
        Y(m,:) = [alpha, g];

        % physics evolution (uncapped) for data diversity
        [ddq, ~, ~, ~] = spinn3d_fd(robot, q, dq, tau_raw, simgen.damping);
        dq = dq + ddq*simgen.dt;
        q  = q  + dq *simgen.dt;
        [q, dq] = spinn3d_joint_limit(robot, q, dq, struct('deadband_deg',0.5,'freezeInward',true));
    end
end

% ===== trim & pack =====
X = X(1:m,:); Y = Y(1:m,:);
meta = struct('nTraj',simgen.nTraj,'dt',simgen.dt,'t_final',simgen.t_final, ...
              'pid', struct('Kp',simgen.Kp,'Ki',Ki_use,'Kd',simgen.Kd,'opts',opts));
ds = struct('X',X,'Y',Y,'caps',caps,'params',params,'gate',gate,'meta',meta);

save('spinn3d_v2_dataset.mat','-struct','ds');
fprintf('Dataset saved: spinn3d_v2_dataset.mat  (M=%d, D=%d)\n', size(X,1), size(X,2));
end
