close all
clear all
clc

kp = 0.3;
ka = 2;
kb = -1.5;

rob_pos = [-2, 3];
rob_th = pi;
goal_pos = [0, 0];
goal_th = 0;

dt = 0.1;
t = 0:dt:20;

figure(1)
hold on
grid on
plot(goal_pos(1),goal_pos(2),'g*')
xlabel('x')
ylabel('y')

for i = 1:length(t)
    
    dx = goal_pos(1) - rob_pos(i,1);
    dy = goal_pos(2) - rob_pos(i,2);
    
    p = sqrt(dx^2 + dy^2);
    a = -rob_th(i) + atan2(dy,dx);
    b = -rob_th(i) - a + goal_th;
    
    if (a > pi)
        a = -(2*pi - a);
    end
    if (a < -pi)
        a = 2*pi + a;
    end
    
    if (b > pi)
        b = -(2*pi - b);
    end
    if (b < -pi)
        b = 2*pi + b;
    end
    
    v = kp*p;
    w = ka*a + kb*b;
    
    rob_th(i+1) = rob_th(i) + dt*w;
    rob_pos(i+1,1) = rob_pos(i,1) + dt*v*cos(rob_th(i+1));
    rob_pos(i+1,2) = rob_pos(i,2) + dt*v*sin(rob_th(i+1));
    
    plot(rob_pos(i+1,1),rob_pos(i+1,2),'r.')
    drawnow
end

