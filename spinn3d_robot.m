function [robot, kin] = spinn3d_robot(params)
% 4-DoF (yaw + 3×pitch) 刚体树，构造后做惯量体检/修复
if nargin < 1 || ~isstruct(params), params = struct(); end
if ~isfield(params,'L'),         params.L = [0.24 0.214 0.324]; end
if ~isfield(params,'mass'),      params.mass = [1.6 1.2 0.8];   end
if ~isfield(params,'com_ratio'), params.com_ratio = [0.5 0.5 0.5]; end
if ~isfield(params,'gravity'),   params.gravity = [0 0 -9.81];  end
if ~isfield(params,'base_mass'),   params.base_mass   = 5.0;  end
if ~isfield(params,'base_radius'), params.base_radius = 0.12; end
if ~isfield(params,'base_height'), params.base_height = 0.03; end
if ~isfield(params,'base_com'),    params.base_com    = [0 0 0]; end
if ~isfield(params,'base_z') || isempty(params.base_z), params.base_z = params.base_height; end

deg = pi/180;
defaultJLim = [ -pi,  pi; -170*deg, 170*deg; -170*deg, 170*deg; -170*deg, 170*deg ];
if ~isfield(params,'jposlim'), params.jposlim = defaultJLim; end

L = double(params.L(:).'); m = double(params.mass(:).'); c = double(params.com_ratio(:).');
assert(numel(L)==3 && numel(m)==3 && numel(c)==3, 'L/mass/com_ratio must be 1x3.');

% ---- build tree ----
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',4); robot.Gravity = params.gravity;

% base yaw
b1 = rigidBody('link_baseYaw');
j1 = rigidBodyJoint('joint_baseYaw','revolute'); j1.JointAxis = [0 0 1];
j1.PositionLimits = params.jposlim(1,:); setFixedTransform(j1, eye(4)); b1.Joint = j1;
mr = max(params.base_mass,1e-3); R = max(params.base_radius,1e-3); H = max(params.base_height,1e-3);
Ixx = (1/12)*mr*(3*R^2 + H^2); Iyy = Ixx; Izz = 0.5*mr*R^2;
b1.Mass = mr; b1.CenterOfMass = params.base_com; b1.Inertia = [Ixx Iyy Izz 0 0 0];
addBody(robot, b1, robot.BaseName);
try, Tbase_vis = trvec2tform([0 0 H/2]); addVisual(b1, 'Cylinder', [R H], Tbase_vis); catch, end

% helper inertia for rod along X
rodInertia = @(mi, Li, r) [ max(1e-5, 0.5*mi*max(r,0.01)^2), mi*Li^2/12, mi*Li^2/12, 0,0,0 ];
Rlink = 0.015;

% shoulder
b2 = rigidBody('link_shoulder');
j2 = rigidBodyJoint('joint_shoulder','revolute'); j2.JointAxis = [0 1 0];
j2.PositionLimits = params.jposlim(2,:); setFixedTransform(j2, trvec2tform([0 0 params.base_z])); b2.Joint = j2;
b2.Mass = max(m(1),1e-3); b2.CenterOfMass = [c(1)*L(1), 0, 0]; b2.Inertia = rodInertia(b2.Mass, L(1), Rlink);
addBody(robot, b2, 'link_baseYaw');
try, Lseg=L(1); Rz2x=axang2tform([0 1 0 pi/2]); Tvis=Rz2x; Tvis(1:3,4)=[Lseg/2,0,0]; addVisual(b2, 'Cylinder', [Rlink Lseg], Tvis); catch, end

% elbow
b3 = rigidBody('link_elbow');
j3 = rigidBodyJoint('joint_elbow','revolute'); j3.JointAxis = [0 1 0];
j3.PositionLimits = params.jposlim(3,:); setFixedTransform(j3, trvec2tform([L(1), 0, 0])); b3.Joint=j3;
b3.Mass = max(m(2),1e-3); b3.CenterOfMass = [c(2)*L(2), 0, 0]; b3.Inertia = rodInertia(b3.Mass, L(2), Rlink);
addBody(robot, b3, 'link_shoulder');
try, Lseg=L(2); Rz2x=axang2tform([0 1 0 pi/2]); Tvis=Rz2x; Tvis(1:3,4)=[Lseg/2,0,0]; addVisual(b3, 'Cylinder', [Rlink Lseg], Tvis); catch, end

% wrist
b4 = rigidBody('link_wrist');
j4 = rigidBodyJoint('joint_wrist','revolute'); j4.JointAxis = [0 1 0];
j4.PositionLimits = params.jposlim(4,:); setFixedTransform(j4, trvec2tform([L(2), 0, 0])); b4.Joint=j4;
b4.Mass = max(m(3),1e-3); b4.CenterOfMass = [c(3)*L(3), 0, 0]; b4.Inertia = rodInertia(b4.Mass, L(3), Rlink);
addBody(robot, b4, 'link_elbow');
try, Lseg=L(3); Rz2x=axang2tform([0 1 0 pi/2]); Tvis=Rz2x; Tvis(1:3,4)=[Lseg/2,0,0]; addVisual(b4, 'Cylinder', [Rlink Lseg], Tvis); catch, end

% tool
tool = rigidBody('tool'); setFixedTransform(tool.Joint, trvec2tform([L(3), 0, 0]));
addBody(robot, tool, 'link_wrist'); try, addVisual(tool,'Sphere',0.02,eye(4)); catch, end

% —— 惯量体检/修复（关键） ——
robot = local_sanitize_inertial(robot);

kin = struct('L', L, 'base_z', params.base_z);
end

function robot = local_sanitize_inertial(robot)
    minM = 1e-3; minI = 1e-5;
    for i=1:numel(robot.Bodies)
        b = robot.Bodies{i};
        if strcmpi(b.Joint.Type,'fixed'), continue; end
        if ~isfinite(b.Mass) || b.Mass<=0, b.Mass = minM; end
        I = b.Inertia;
        if numel(I)~=6 || any(~isfinite(I)), I = [minI minI minI 0 0 0];
        else, I(1:3) = max(I(1:3), minI); I(4:6) = arrayfun(@(x) (isfinite(x)*x), I(4:6)); end
        b.Inertia = I;
        if any(~isfinite(b.CenterOfMass)), b.CenterOfMass = [0 0 0]; end
    end
end
