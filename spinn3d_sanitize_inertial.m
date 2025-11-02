function robot = spinn3d_sanitize_inertial(robot)
% 修 RBT 易错惯性参数，避免 forwardDynamics/velocityProduct 出 NaN/Inf
    minM = 1e-3;    % kg
    minI = 1e-5;    % kg·m^2
    for i = 1:numel(robot.Bodies)
        b = robot.Bodies{i};
        if ~strcmpi(b.Joint.Type,'fixed')
            if ~isfinite(b.Mass) || b.Mass<=0, b.Mass = minM; end
            I = b.Inertia;
            if numel(I)~=6 || any(~isfinite(I))
                I = [minI minI minI 0 0 0];
            else
                I(1:3) = max(I(1:3), minI);
                I(4:6) = arrayfun(@(x) (isfinite(x)*x), I(4:6));  % 清 NaN/Inf
            end
            b.Inertia = I;
            if any(~isfinite(b.CenterOfMass)), b.CenterOfMass = [0 0 0]; end
        end
    end
end
