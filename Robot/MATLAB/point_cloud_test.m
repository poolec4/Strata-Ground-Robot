close all
clear all
clc

im = imread('test_depth6.png', 'png');
im = im(:,:,1);
[m,n] = size(im);

figure
imshow(im)

im = im2double(im);
pcloud = zeros(m*n,3);

count = 1;

for y=1:n
    for x=1:m
       pcloud(count,:) = depthToPointCloudPos(x,y,im(x,y));
       count = count + 1;
    end
end

figure
plot3(pcloud(:,1),pcloud(:,2),pcloud(:,3),'k.','MarkerSize',1)


% https://www.mathworks.com/help/vision/ref/pcfitplane.html
