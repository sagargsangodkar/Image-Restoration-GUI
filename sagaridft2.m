function Idft2 = sagaridft2(X,M,N)
if nargin<2
  M = size(X,1);
  N = size(X,2);
end
Y = padarray(X,[M-size(X,1),N-size(X,2)],0,'post');
WM = exp(1i*2*pi/M);
WN = exp(1i*2*pi/N);
m = 0:1:M-1;
DF = WM.^(m'*m);
n = 0:1:N-1;
DG = WN.^(n'*n);
if size(X,3) > 1
    Idft2(:,:,1) = DF*Y(:,:,1)*DG/(M*N); 
    Idft2(:,:,2) = DF*Y(:,:,2)*DG/(M*N); 
    Idft2(:,:,3) = DF*Y(:,:,3)*DG/(M*N); 
else
    Idft2 = DF * Y * DG /(M*N);
%tic; Xfft_inbuilt = fft2(X,M,N); toc;
end