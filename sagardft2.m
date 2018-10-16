function Xdft2 = sagardft2(X,M,N)
if nargin<2
  M = 1024;
  N = 1024;
end
Y = padarray(X,[M-size(X,1),N-size(X,2)],0,'post');
WM = exp(-1i*2*pi/M);
WN = exp(-1i*2*pi/N);
u = 0:1:M-1;
DF = WM.^(u'*u);
v = 0:1:N-1;
DG = WN.^(v'*v);
%tic; 
if size(X,3) > 1
    Xdft2(:,:,1) = DF*Y(:,:,1)*DG; 
    Xdft2(:,:,2) = DF*Y(:,:,2)*DG; 
    Xdft2(:,:,3) = DF*Y(:,:,3)*DG; 
else
    Xdft2 = DF * Y * DG;
%toc;
%tic; Xdft2_inbuilt = fft2(X,M,N); toc;
end