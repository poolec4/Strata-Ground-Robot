close all
clear all
clc

kp = 2;
ka = 2;
kb = -1;

kpi = 0.1;
kai = -0.1;
kbi = -0.1;

p_sum = 0;
a_sum = 0;
b_sum = 0;

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
    
    p_sum = p_sum + p*dt;
    a_sum = a_sum*0.9 + a*dt;
    b_sum = b_sum*0.9 + b*dt;
    
    if (p < 0.1)
        p_sum = 0;
        break
    end
    
    if (abs(a) < 0.1)
        a_sum = 0;
    end
    
    if (abs(b) < 0.1)
        b_sum = 0;
    end
    
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
    
    v = kp*p + kpi*p_sum;
    w = ka*a + kb*b + kai*a_sum + kbi*b_sum;
    
    rob_th(i+1) = rob_th(i) + dt*w;
    rob_pos(i+1,1) = rob_pos(i,1) + dt*v*cos(rob_th(i+1));
    rob_pos(i+1,2) = rob_pos(i,2) + dt*v*sin(rob_th(i+1));
    
    plot(rob_pos(i+1,1),rob_pos(i+1,2),'r.')
    drawnow
end

