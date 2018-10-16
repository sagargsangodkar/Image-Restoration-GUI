ref = im2double(imread('/home/biometrics/Desktop/IP_assignment2/unwanted/GroundTruth_pngs_Image1/Image1/Kernel1/GroundTruth1_1_1.png'));
M = size(ref,1);
N = size(ref,2);
%%
C = [0 1 0; 1 -4 1; 0 1 0];
C = double(C);
C_fft = fft2(C,M,N);
kernel = fspecial('gaussian',7,2);
%%
blurred = imfilter(ref, kernel, 'conv', 'circular');
sigma_n = 10^(-40/20)*abs(1-0);
noise = sigma_n * randn(size(ref));
A = imnoise(blurred,'gaussian',0,sigma_n);
%%
figure;imshow(blurred)
figure;imshow(ref)
figure;imshow(A)
%%
op = sagarclsfilter(fft2(A,M,N),fft2(kernel,M,N),C_fft,1);
immse(op, ref);
figure; imshow(op); figure; imshow(ref); 
%%
j = logspace(-2,4,100);
for i=1:100
    op = sagarclsfilter(fft2(A,M,N),fft2(kernel,M,N),C_fft,j(i));
    err(i) = immse(op, ref);
end
figure;stem(log(j),err.*255);