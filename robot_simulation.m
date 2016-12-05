%uncontrolled self balancing robot simulation.


function robot_simulation()

%initial variables

%%%%% the starting constants and other unknown quantities go here

% options = odeset('Events',@evel;

%initial conditions
close all;

%%%%% all starting state variables go here

%values are all bs for now
g = 9.8;
m_wheel = 0.2; %kg
m_plat = 0.5; %kg
r1 = .02; %inner radius of wheel
r2 = .04; %outer radius of wheel
l = .15;
I_wheel = m_wheel/2 * (r1^2 + r2^2);
I_plat = (m_plat*l^2)/3;
Mmotor = 1;

%%%%% also vector of starting conditions also go in here

%ODE computation
vals0 = [0, 2*pi/3,0,0];

options = odeset('Events', @events);
[T, U] = ode23s(@balance, [0,1], vals0, options);
X = U(:,1);
Theta = U(:,2);
Xplat = cos(Theta);
Yplat = sin(Theta);

%%%%%% Visualizations go here

% comet3(U);
% grid on;
COMX = ((l)*cos(Theta)) + X;
COMY = (l)*sin(Theta);
for i=1:length(Theta)
    cla
    hold on
    refline(0, -r2);
    plot(COMX(i), COMY(i), '*k');
    plot(COMX(i) - l*cos(Theta(i)), COMY(i) - l*sin(Theta(i)), '*r');
    line([COMX(i) - l*cos(Theta(i)), COMX(i) + l*cos(Theta(i))], [COMY(i) - l*sin(Theta(i)), COMY(i) + l*sin(Theta(i))]);
    axis([-.5 .1 -.1 .4]);
    drawnow;
    pause(.1);
end
title('Self Balancing Robot');
xlabel('X (m)');
ylabel('Y (m)');

% figure
% plot(T, Xplat);
% title('X pos');
% xlabel('time (s)');
% ylabel('X (m)');
% 
% figure
% plot(T, Yplat);
% title('Y pos');
% xlabel('time (s)');
% ylabel('Y (m)');



%functions
function res = balance(~, vals)

    x = vals(1);
    theta = vals(2);
    vx = vals(3);
    vtheta = vals(4);
    
    %equations matrix
    %   x'', theta'', Fpx, Fpy, Ff
    A = [m_plat, m_plat*l*sin(theta), 1, 0, 0;...
        0, m_plat*l*cos(theta), 0, -1, 0;...
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
    res = [vx; vtheta; dvx; dvtheta];
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




