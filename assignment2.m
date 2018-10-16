function varargout = assignment2(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @assignment2_OpeningFcn, ...
                   'gui_OutputFcn',  @assignment2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

% --- Executes just before assignment2 is made visible.
function assignment2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to assignment2 (see VARARGIN)

% Choose default command line output for assignment2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = assignment2_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes on button press in loadimage.
function loadimage_Callback(hObject, eventdata, handles)
global im immodified path im_fft M N file path;

[file,path] = uigetfile('*','Select Blurry Image');
if isequal(file,0)
   disp('User selected Cancel');
   msgbox(sprintf('User Pressed Cancel'),'Error','Error')
   %msgbox('String','Title','modal')
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
im = im2double(imread(fullfile(path,file)));
immodified = im;

axes(handles.axes1); imshow(im); 

axes(handles.axes2); imshow(immodified)

M = size(im,1); N = size(im,2);
% DFT (MxN) of image
im_fft = sagardft2(im,M,N);


% --- Executes on button press in loadkernel.
function loadkernel_Callback(hObject, eventdata, handles)
global kernel kernel_fft M N l m;
l = size(kernel,1);
m = size(kernel,2);
[file,path] = uigetfile('*','Select Blur Kernel');
if isequal(file,0)
   disp('User selected Cancel');
   msgbox(sprintf('User Pressed Cancel'),'Error','Error')
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
kernel = im2double(imread(fullfile(path,file)));
kernel = kernel/sum(kernel(:));

%kernel = [0 0 0 0.1 0.1; 0 0 0.1 0.1 0; 0 0.1 0.1 0 0; 0.1 0.1 0 0 0; 0.1 0 0 0 0];

% DFT (MxN) of blur kernel
kernel_fft = sagardft2(kernel,M,N);


% --- Executes on button press in gt.
function gt_Callback(hObject, eventdata, handles)
global gt gtflag
[file,path] = uigetfile('*','Select Groudn Truth (if available)');
if isequal(file,0)
   disp('User selected Cancel');
   msgbox(sprintf('User Pressed Cancel'),'Error','Error')
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
gt = im2double(imread(fullfile(path,file)));
gtflag = 1;


% --- Executes on button press in fullinverse.
function fullinverse_Callback(hObject, eventdata, handles)
%tic;
global immodified im_fft kernel_fft;
%disp('Inverse Filtering')
inv_filter_fft = ones(size(kernel_fft))./kernel_fft;
immodified = real(sagaridft2(im_fft.*inv_filter_fft));
%toc;
axes(handles.axes2); imshow(immodified)



% --- Executes on button press in truncated_inverse.
function truncated_inverse_Callback(hObject, eventdata, handles)
%disp('Truncated Inverse Filtering')
global immodified im_fft kernel_fft threshold;
%tic;
inv_filter_fft = ones(size(kernel_fft))./kernel_fft;

% Mask for truncation of the inverse filter
mask = abs(kernel_fft) > str2double(cell2mat(threshold));

% Truncating the inverse filter abouve threshold
truncated_filter_fft = mask.*inv_filter_fft;

% Filtering image using truncated inverse filter
immodified = real(sagaridft2(im_fft.*truncated_filter_fft));

%toc;
axes(handles.axes2); imshow(immodified)


% --- Executes on button press in threshold.
function threshold_Callback(hObject, eventdata, handles)
global threshold;
prompt = {'Enter a threshold value (range 0 to 1)'};
title = 'Threshold Value';
definput = {'0.1'};
opts.Interpreter = 'tex';
threshold = inputdlg(prompt,title,[1 40],definput,opts);


% --- Executes on button press in k_weiner.
function k_weiner_Callback(hObject, eventdata, handles)
global k_weiner;
prompt = {'Enter K (Estimated Noise to Signal Ratio)'};
title = 'K Value';
definput = {'0.01'};
opts.Interpreter = 'tex';
input = inputdlg(prompt,title,[1 40],definput,opts);
k_weiner = str2double(cell2mat(input));

% --- Executes on button press in weinerfilter.
function weinerfilter_Callback(hObject, eventdata, handles)
disp('Weiner Filtering')
global immodified im_fft kernel_fft k_weiner;

% Applying Weiner filter
immodified = sagarweiner(im_fft,kernel_fft, k_weiner);
axes(handles.axes2); imshow(immodified)

% --- Executes on button press in gamma_cls.
function gamma_cls_Callback(hObject, eventdata, handles)
global gamma_cls;
prompt = {'Enter a gamma value'};
title = 'Gamma Value';
definput = {'1'};
opts.Interpreter = 'tex';
input = inputdlg(prompt,title,[1 40],definput,opts);
gamma_cls = str2double(cell2mat(input));


% --- Executes on button press in clsfilter.
function clsfilter_Callback(hObject, eventdata, handles)
disp('CLS Filtering')
global immodified im_fft kernel_fft gamma_cls M N;
% Prior information (assumed): Image is smooth

% Laplacian kernel (High Pass Filter)
C = [0 1 0; 1 -4 1; 0 1 0];

% DFT (MxN) of C
C_fft = sagardft2(C,M,N);

% Applying CLS filter
immodified = sagarclsfilter(im_fft,kernel_fft,C_fft, gamma_cls);
axes(handles.axes2); imshow(immodified)



% --- Executes on button press in save.
function save_Callback(hObject, eventdata, handles)
global file path immodified count
% imwrite(A,filename) writes image data A to the file specified by filename.
% File format is inferred from the extension in filename.
opfilepath = sprintf('%sdeblurred_%d_%s',path,count,file);
count = count+1;
imwrite(immodified, opfilepath); 
msgbox('Image Saved Successfully','Success');


% --- Executes on button press in ssim.
function ssim_Callback(hObject, eventdata, handles)
global gt immodified gtflag
if gtflag
    ux = mean(gt(:));
    uy = mean(immodified(:));
    cov_matrix = cov(gt(:),immodified(:));
    sigmax = sqrt(cov_matrix(1,1));
    sigmay = sqrt(cov_matrix(2,2));
    sigmaxy = cov_matrix(1,2);
    num = (2*ux*uy + 0.0001)*(2*sigmaxy + 0.0009);
    den = (ux^2 + uy^2 + 0.0001)*(sigmax^2 + sigmay^2 + 0.0009);
    ssimvalue = num./den;
    set(handles.ssim_disp,'String',sprintf('%.3f',ssimvalue))
else
    msgbox(sprintf('Ground Truth Not Available'),'Error','Error')
    set(handles.ssim_disp,'String','NA')
end


function ssim_disp_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ssim_disp_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in mse.
function mse_Callback(hObject, eventdata, handles)
global gt immodified gtflag mse
if gtflag
    mse = immse(gt, immodified);
    set(handles.mse_display,'String',sprintf('%.3f',mse))
else
    msgbox(sprintf('Ground Truth Not Available'),'Error','Error')
    set(handles.mse_display,'String','NA')
end


function mse_display_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function mse_display_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in showgt.
function showgt_Callback(hObject, eventdata, handles)
global gt gtflag
if gtflag
    figure; imshow(gt);
else
    msgbox(sprintf('Ground Truth Not Available'),'Error','Error')
end


% --- Executes on button press in psnr.
function psnr_Callback(hObject, eventdata, handles)
global gt immodified gtflag mse
if gtflag
    mse = immse(gt, immodified);
    psnr = 10*log10(1/mse);
    set(handles.psnr_disp,'String',sprintf('%.3f',psnr))
else
    msgbox(sprintf('Ground Truth Not Available'),'Error','Error')
    set(handles.psnr_disp,'String','NA')
end



function psnr_disp_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function psnr_disp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psnr_disp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
