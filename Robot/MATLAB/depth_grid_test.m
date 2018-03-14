% Script to test the conversion of a depth map from the kinect to an
% occupancy frid with cell probabilitites
%   Written by Chris Poole - Spring 2018

close all
clear all
clc

MAX_ROVER_STEP = 20; % pixels? centimeters? no clue how to implement this yet
block_prob = @(m)exp(-0.5/255*m);

im = imread('test_depth6.png', 'png');
im = im(:,:,1);
[m,n] = size(im);

figure
imshow(im)

occ_grid = zeros([255,n]);

tic
for j = 1:n
    for i = 1:m
        dist = im(i,j); % val out of 255
        if dist ~= 0
            k = (256-im2double(dist)*255):255;
            occ_grid(k,j) = occ_grid(k,j) + block_prob(i);
        end
    end
end
toc

[m_occ,n_occ] = size(occ_grid);
[x,y] = meshgrid(1:n_occ,1:m_occ);

figure
colormap('gray')
h = pcolor(x,y,occ_grid);
set(h, 'EdgeColor', 'none');

figure
colormap(parula(255/15))
s = plot3(x,y,occ_grid,'.');
s.EdgeColor = 'none';
s.CData = occ_grid;
xlabel('x'); ylabel('y'); zlabel('z');

