%controlled self balancing robot simulation.


function robot_simulation()

%initial variables

%%%%% the starting constants and other unknown quantities go here

% options = odeset('Events',@evel;

%initial conditions
close all;

%%%%% all starting state variables go here

%values are all bs for now
% g = 9.8;
% m_wheel = .076; %kg for two wheels
% m_plat = 0.368; %kg
% r1 = .023; %inner radius of wheel
% r2 = .0325; %outer radius of wheel
% l = .125; %m from base of platform to COM
% I_wheel = m_wheel/2 * (r1^2 + r2^2);
% I_plat = (m_plat*l^2)/3;
% Mmotor = 0;

g = 9.8;
m_wheel = 0.2; %kg
m_plat = 0.5; %kg
r1 = .02; %inner radius of wheel
r2 = .04; %outer radius of wheel
l = .15;
I_wheel = m_wheel/2 * (r1^2 + r2^2);
I_plat = (m_plat*l^2)/3;
Mmotor = 0;


%PID 
settheta = pi/2;
Pgain = -256;
Igain = -.01;
Dgain = -2.5;
Istate = 0;
error = 0;



%ODE computation
vals0 = [0, 2*pi/3,0,0, Mmotor];

options = odeset('Events', @events);
[T, U] = ode45(@balance, [0,.5], vals0, options);
X = U(:,1);
Theta = U(:,2);
Xplat = cos(Theta);
Yplat = sin(Theta);

%%%%%% Visualizations go here

COMX = ((l)*cos(Theta)) + X;
COMY = (l)*sin(Theta);
% for i=1:1:length(Theta)
%     cla
%     hold on
%     refline(0, -r2);
%     plot(COMX(i), COMY(i), '*k');
%     plot(COMX(i) - l*cos(Theta(i)), COMY(i) - l*sin(Theta(i)), '*r');
%     pos = [COMX(i) - l*cos(Theta(i))-r2 COMY(i) - l*sin(Theta(i))-r2 r2*2 r2*2];
%     rectangle('Position', pos, 'Curvature', [1, 1]);
%     line([COMX(i) - l*cos(Theta(i)), COMX(i) + l*cos(Theta(i))], [COMY(i) - l*sin(Theta(i)), COMY(i) + l*sin(Theta(i))]);
%     axis([-.5 .5 -.5 .5]);
%     drawnow;
% %     pause(.05);
% end

title('Self Balancing Robot');
xlabel('X (m)');
ylabel('Y (m)');

figure
plot(T, COMX-X);
title('X pos');
xlabel('time (s)');
ylabel('X (m)');

figure
plot(T, COMY);
title('Y pos');
xlabel('time (s)');
ylabel('Y (m)');

figure
plot(T, U(:,2));
title('Theta');
xlabel('time (s)');
ylabel('Theta (rad)');

%functions
function res = balance(t, vals)

    x = vals(1);
    theta = vals(2);
    vx = vals(3);
    vtheta = vals(4);
    motor = vals(5);
    
    error = theta - settheta;
    if (abs(error) > 0.0001)
        Pterm = Pgain * error;
        Istate = Istate + error;
        Iterm = Igain * Istate;
        Dterm = Dgain * vtheta;
        motor = Pterm + Iterm + Dterm;
        disp(motor);
    
    else
        motor = 0;
        disp(motor);

    
    end
    Mmotor = motor;
    
    %equations matrix
    %   x'', theta'', Fpx, Fpy, Ff
    A = [m_plat, -m_plat*l*sin(theta), 1, 0, 0;...
        0, -m_plat*l*cos(theta), 0, -1, 0;...
        0, -I_plat, -l *sin(theta), l* cos(theta), 0;...
        m_wheel, 0, -1, 0, -1;...
        I_wheel/r2, 0, 0, 0, r2];
    
    B = [m_plat*l*vtheta^2*cos(theta);...
        m_plat*g - m_plat*l*vtheta^2*sin(theta);...
        Mmotor;...
        0;...
        Mmotor];
    
    Xvals = A^(-1)*B;
    dvx = Xvals(1);
    dvtheta = Xvals(2);
    res = [vx; vtheta; dvx; dvtheta; Mmotor];
end

%%%% events function in case we want to limit how far the platform can
%%%% rotate

 function [value, isterminal, direction] = events(t, y)
        x = y(1);
        theta = y(2);
        ypos = (2*l)*sin(theta);
        isterminal = 1;
        direction = -1;
        value = ypos + r2;
 end

end




