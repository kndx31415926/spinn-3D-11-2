function issues = spinn3d_check_inertial(robot)
    issues = [];
    for i=1:numel(robot.Bodies)
        b = robot.Bodies{i};
        badM = (~isfinite(b.Mass) || b.Mass<=0);
        I = b.Inertia; badI = (numel(I)~=6) || any(~isfinite(I)) || any(I(1:3)<=0);
        badC = any(~isfinite(b.CenterOfMass));
        if ~strcmpi(b.Joint.Type,'fixed') && (badM||badI||badC)
            issues = [issues; struct('name',b.Name,'badMass',badM,'badInertia',badI,'badCOM',badC)]; %#ok<AGROW>
        end
    end
end
