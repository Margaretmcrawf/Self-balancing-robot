%uncontrolled self balancing robot simulation.


function robot_simulation()

    
%initial variables

%%%%% the starting constants and other unknown quantities go here


% options = odeset('Events',@events);

%initial conditions
close all;

%%%%% all starting state variables go here


%%%%% also vector of starting conditions also go in here


%ODE computation
[T,U] = ode45(@balance, [0:.01:10], []);


%%%%%% Visualizations go here

% comet3(U);
% grid on;
title('Self Balancing Robot');
xlabel('X (m)');
ylabel('Y (m)');

figure
plot(T, X);
title('X pos');
xlabel('time (s)');
ylabel('X (m)');
figure
plot(T, Y);
title('Y pos');
xlabel('time (s)');
ylabel('Y (m)');
figure
plot(T, Z);
title('Z pos');
xlabel('time (s)');
ylabel('Z (m)');


vals0 = [0,0,0,0];
[T, V] = ode45(@derivs, [0,10], vals0);

%functions
function res = balance(t, vals)

    x = vals(1);
    theta = vals(2);
    vx = vals(3);
    vtheta = vals(4);
    
    dTdt = vtheta;
    dXdt = vx;
  
    dV = acceleration(x, theta, vx, vtheta);
    res = [dTdt; dXdt;dV(1);dV(2)];
end

function res = acceleration(x, theta,vx, vtheta)

    %%%%%  calculations for accelerations and other variables go here
    Taccel = 0;
    Xaccel = 0;
    res = [Xaccel; Taccel];
    
end

%%%% events function in case we want to limit how far the platform can
%%%% rotate

%  function [value, isterminal, direction] = events(t,y)
%         isterminal = 1;
%         direction = -1;
%         value = y(2);
%  end

end




