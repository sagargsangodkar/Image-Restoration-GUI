function opimage = sagarweiner(im_fft, kernel_fft, estimated_nsr)
% Numerator and denominator of frequency response of CLS filter.
estimated_nsr = double(estimated_nsr);
num = (abs(kernel_fft)).^2;
den = kernel_fft .* ((abs(kernel_fft)).^2 + estimated_nsr);
wiener_filt_fft = num./den;
opimage = real(ifft2(im_fft.*wiener_filt_fft));