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

%%
close all
clear all
clc

load('object3d.mat')
figure
pcshow(ptCloud)
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
title('Original Point Cloud')

maxDistance = 0.02;
referenceVector = [0,0,1];
maxAngularDistance = 5;
[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);

figure
pcshow(plane1)
title('Ground Plane')






