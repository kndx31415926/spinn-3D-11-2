function [alpha, g, Paxis, tau_star, info] = spinn3d_oracle_alloc_v2(tau_raw, dq, caps)
% Power allocation oracle for v2:
% Maximize alignment with desired torque tau_raw under per-axis & total power caps.
% Continuous knapsack: allocate power to axes with higher |tau_raw|/|w| first.
%
% Inputs:
%   tau_raw : 1xN desired torque (PID + gravity), before caps
%   dq      : 1xN joint velocities
%   caps    : struct with fields Pmax, Prated(1xN), tauMax(1xN), epsw(optional)
%
% Outputs:
%   alpha   : 1xN normalized axis fractions P_i / Pmax (0..1); zeros if Pmax<=0
%   g       : scalar gate in [0,1], equals sum(P_i)/Pmax (how much of Pmax is used)
%   Paxis   : 1xN axis powers actually used (W)
%   tau_star: 1xN torque after optimal allocation but NOT exceeding |tau_raw| nor tauMax
%   info    : struct with fields 'score','order'

if ~isfield(caps,'epsw'), caps.epsw = 1e-3; end
N = numel(tau_raw);
w = abs(dq(:).') + caps.epsw;

% per-axis available power caps (rated + torque limit)
Pcap = inf(1,N);
if isfield(caps,'Prated') && ~isempty(caps.Prated), Pcap = min(Pcap, caps.Prated(:).'); end
if isfield(caps,'tauMax') && ~isempty(caps.tauMax), Pcap = min(Pcap, abs(caps.tauMax(:).').*w); end
Pcap(~isfinite(Pcap)) = 0;

% score for allocation (knapsack weight)
score = abs(tau_raw(:).') ./ w;   % bigger -> allocate first
[~, order] = sort(score, 'descend');

Prem = max(0, caps.Pmax);
Paxis = zeros(1,N);
for idx = 1:N
    j = order(idx);
    if Prem <= 0, break; end
    give = min(Pcap(j), Prem);
    if ~isfinite(give) || give<=0, continue; end
    Paxis(j) = give;
    Prem = Prem - give;
end

Pused = sum(Paxis);
if caps.Pmax > 0
    alpha = Paxis / caps.Pmax;
    g = Pused / caps.Pmax;
else
    alpha = zeros(1,N); g = 0;
end

% torque from allocated power: do not exceed |tau_raw| nor tauMax
tau_lim = inf(1,N);
if isfield(caps,'tauMax') && ~isempty(caps.tauMax)
    tau_lim = min(tau_lim, caps.tauMax(:).');
end
tau_star = sign(tau_raw(:).') .* min( [abs(tau_raw(:).'), Paxis./w, tau_lim], [], 2 ).';

info = struct('score',score,'order',order,'Pcap',Pcap,'Pused',Pused);
end
