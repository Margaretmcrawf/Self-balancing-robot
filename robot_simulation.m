function robot_simulation()
%uncontrolled self balancing robot.

    function res = derivs(~, vals)
        x = vals(1);
        theta = vals(2);
        vx = vals(3);
        vtheta = vals(4);

        res = vals; %this is just plain wrong.
    end


vals0 = [0,0,0,0];
[T, V] = ode45(@derivs, [0,10], vals0);
end


