% Inverse filtering
disp('Inverse Filtering')
%image = real(ifft2(blurkernel_fft.*fft2(image)));
inv_filter_fft = ones(size(blurkernel_fft))./blurkernel_fft;
image_inv(:,:,1) = real(ifft2(fft2(image(:,:,1)).*inv_filter_fft));
image_inv(:,:,2) = real(ifft2(fft2(image(:,:,2)).*inv_filter_fft));
image_inv(:,:,3) = real(ifft2(fft2(image(:,:,3)).*inv_filter_fft));
figure; imshow(image_inv); 
