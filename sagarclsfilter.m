function opimage = sagarclsfilter(im_fft,kernel_fft,C_fft,gamma)
% Numerator and denominator of frequency response of CLS filter.
gamma = double(gamma);
numerator = conj(kernel_fft);
denominator = abs(kernel_fft).^2 + gamma*(abs(C_fft).^2);

% CLS filter response
cls_filter_fft = numerator./denominator;

opimage = real(ifft2(im_fft.*cls_filter_fft));
