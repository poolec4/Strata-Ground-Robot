%close all
clf(1)
clear all
clc

kp = 0.3;
ka = 0.6;
kb = -0.2;

rob_pos = [5, 6];
rob_th = 0;
goal_pos = [0, 0];
goal_th = 0;

dt = 0.1;
t = 0:dt:20;

for i = 1:length(t)
    
    dx = goal_pos(1) - rob_pos(i,1);
    dy = goal_pos(2) - rob_pos(i,2);
    
    p = sqrt(dx^2 + dy^2);
    a = -rob_th(i) + atan2(dy,dx);
    b = -rob_th(i) - a;
    
    v = kp*p;
    w = ka*a + kb*b;
    
    rob_th(i+1) = rob_th(i) + dt*w;
    rob_pos(i+1,1) = rob_pos(i,1) + dt*v*cos(rob_th(i+1));
    rob_pos(i+1,2) = rob_pos(i,2) + dt*v*sin(rob_th(i+1));
end

figure(1)
hold on
grid on
plot(rob_pos(:,1),rob_pos(:,2))
xlabel('x')
ylabel('y')
