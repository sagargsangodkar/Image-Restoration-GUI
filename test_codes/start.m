[file,path] = uigetfile('*.png','Select Blurry Image');
if isequal(file,0)
   disp('User selected Cancel');
else
   disp(['User selected ', fullfile(path,file)]);
end
image = im2double(imread(fullfile(path,file)));

[file,path] = uigetfile('*.png','Select Blur Kernel');
if isequal(file,0)
   disp('User selected Cancel');
else
   disp(['User selected ', fullfile(path,file)]);
end
blurkernel = im2double(imread(fullfile(path,file)));
blurkernel = blurkernel/sum(blurkernel(:));
blurkernel_fft = sagardft2(blurkernel,size(image,1),size(image,2));

figure; imshow(image); title('Original Image');