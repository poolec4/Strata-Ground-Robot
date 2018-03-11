% Script to test the conversion of a depth map from the kinect to an
% occupancy frid with cell probabilitites
%   Written by Chris Poole - Spring 2018

close all
clear all
clc

im = imread('test_depth1.png', 'png');
im = im(:,:,1);
[m,n] = size(im);

figure
imshow(im)

occ_grid = zeros([255,640]);

for j = 1:n
    for i = 1:m
        dist = im(i,j); % val out of 255
        if dist ~= 0
            for k = (256-im2double(dist)*255):255
                occ_grid(k,j) = occ_grid(k,j)+(1/m)*255;
            end
        end
    end
end

[m_occ,n_occ] = size(occ_grid);
[x,y] = meshgrid(1:n_occ,1:m_occ);
figure
colormap('default')
h =pcolor(x,y,occ_grid);
set(h, 'EdgeColor', 'none');


