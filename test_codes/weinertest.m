clear
cam = im2double(imread('cameraman.tif'));
%blurkernel = fspecial('gaussian',7,2);
%blurkernel = im2double(imread('../blur_kernels/blurkernel1.png'));
%blurkernel = blurkernel/sum(blurkernel);
%blurkernel_fft = fft2(blurkernel,size(cam,1),size(cam,2));
%%
cam_blur = real(ifft2(blurkernel_fft.*fft2(cam)));
figure; imshow(cam_blur)
figure; imshow(cam)

%%
inv_filter_fft = ones(size(blurkernel_fft))./blurkernel_fft;
image_inv = real(ifft2(fft2(cam_blur).*inv_filter_fft));
figure; imshow(image_inv)

%%
noise_mean = 0;
noise_var = 0.0001;
blurred_noisy = imnoise(cam_blur, 'gaussian', noise_mean, noise_var);
%blurred_noisy = imnoise(cam, 'gaussian', noise_mean, noise_var);
figure, imshow(blurred_noisy)
%%
K = 0.01;
%K = noise_var / var(cam(:));
num = (abs(blurkernel_fft)).^2;
den = blurkernel_fft .* ((abs(blurkernel_fft)).^2 + K);
wiener_filt_fft = num./den;
image_inv = real(ifft2(fft2(blurred_noisy).*wiener_filt_fft));
figure; imshow(image_inv);
%%
wnr1 = deconvwnr(blurred_noisy, blurkernel, 0.01);
imshow(wnr1);
title('Restored Image');
