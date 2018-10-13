close all;
clear;

img = imread('DSC01000.JPG');
% converts RGB to CIE 1976 L*a*b* values.
lab_he = rgb2lab(img);
% get colour in ab colour space
ab = lab_he(:,:,2:3);
ab = im2single(ab);
nColors = 3;

% repeat the clustering 3 times to avoid local minima
pixel_labels = imsegkmeans(ab,nColors,'NumAttempts',3);
figure;
imshow(img);
figure;
imshow(pixel_labels,[])

% show image in the red colour / the apple
mask3 = pixel_labels==3;
cluster3 = img .* uint8(mask3);
figure;
imshow(cluster3);