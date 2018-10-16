% Truncated Inverse Filter
disp('Truncated Inverse Filtering')
prompt = {'Enter a threshold value (range 0 to 1)'};
title = 'Threshold Value';
definput = {'0.1'};
opts.Interpreter = 'tex';
threshold = inputdlg(prompt,title,[1 40],definput,opts);

inv_filter_fft = ones(size(blurkernel_fft))./blurkernel_fft;
mask = abs(blurkernel_fft) > str2double(cell2mat(threshold));
image_tinv(:,:,1) = real(ifft2(mask.*fft2(image(:,:,1)).*inv_filter_fft));
image_tinv(:,:,2) = real(ifft2(mask.*fft2(image(:,:,2)).*inv_filter_fft));
image_tinv(:,:,3) = real(ifft2(mask.*fft2(image(:,:,3)).*inv_filter_fft));
figure; imshow(image_tinv);