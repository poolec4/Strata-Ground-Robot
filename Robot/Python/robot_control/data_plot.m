close all
clear all
clc 

load('init_world.mat')
init_world = imresize(init_world, 20, 'nearest');
init_world = imrotate(init_world, 90);
figure
imshow(init_world)

%%
load('pcl.mat')
figure
hold on
grid on
for i = 1:length(pcl)
    if ~isequal(pcl(i,:),[0,0,0])
        disp(i)
        plot3(pcl(i,1), pcl(i,3), pcl(i,2),'r.','MarkerSize',5)
    end
end
title('Point Cloud Data')
xlabel('x')
ylabel('y')
zlabel('z')

%%
load('world.mat')
world = imresize(world, 20, 'nearest');
world = imrotate(world, 90);
figure
imshow(world)

%%
load('traj.mat')
load('vicon_data.mat')
traj = cell2mat(traj);
figure
hold on
grid on
plot(traj(1,:),traj(2,:))

title('Trajectory Tracking')
xlabel('x')
ylabel('y')