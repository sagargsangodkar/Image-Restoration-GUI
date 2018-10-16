close all
clear
%%
I = im2double(imread('cameraman.tif'));
imshow(I);
title('Original Image (courtesy of MIT)');
%%
PSF = im2double(imread('../blur_kernels/blurkernel1.png'));
PSF = PSF/sum(PSF(:));
blurred = imfilter(I, PSF, 'conv', 'circular');
figure;imshow(blurred);
title('Blurred Image');
%%
wnr1 = deconvwnr(blurred, PSF, 0);
imshow(wnr1);
title('Restored Image');

%%
noise_mean = 0;
noise_var = 0.0001;
blurred_noisy = imnoise(blurred, 'gaussian', noise_mean, noise_var);
figure, imshow(blurred_noisy)
title('Simulate Blur and Noise')
%%
estimated_nsr = 0;
wnr2 = deconvwnr(blurred_noisy, PSF, estimated_nsr);
figure, imshow(wnr2)
title('Restoration of Blurred, Noisy Image Using NSR = 0')

%%
estimated_nsr = noise_var / var(I(:));
wnr3 = deconvwnr(blurred_noisy, PSF, estimated_nsr);
figure, imshow(wnr3)
title('Restoration of Blurred, Noisy Image Using Estimated NSR');