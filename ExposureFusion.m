function [img] = ExposureFusion(Frames, Cw, Sw, Ew)
% ExposureFusion: piple line of exposure fusion using weightmaps and
% pyramid reconstruction technique
% @param Frames: individual frames from video
% @param Cw: the contrast map weight
% @param Sw: Saturation map weight
% @param Ew: Exposedness map weight
[h, w, c] = size(Frames{1});
[x N] = size(Frames);
% initialize weight map for N frames
W = zeros(h,w,N);
W_sum = zeros(h,w);
pyr = gaussian_pyramid(zeros(h,w,3),5);
for i = 1:N
    I = im2double(Frames{i});
    W(:,:,i) = compWeightMap(I, Cw, Sw, Ew, i);
    W_sum(:,:) = W_sum(:,:)+W(:,:,i);
    W(:,:,i) = W(:,:,i)./W_sum(:,:);  
    pyrW = gaussian_pyramid(W(:,:,i),5);
	pyrI = laplacian_pyramid(I,5);
    for l = 1:5
        w = repmat(pyrW{l},[1 1 3]);
        pyr{l} = pyr{l} + w.*pyrI{l};
    end
end
img = reconstruct_laplacian_pyramid(pyr);
img(:,:,1) = normalize(img(:,:,1));
img(:,:,2) = normalize(img(:,:,2));
img(:,:,3) = normalize(img(:,:,3));
end
function [nor] = normalize(I)
I = I - min(I(:)) ;    
I = I / max(I(:)) ;
nor = I;
end
%%%%%%%%%
function [wMap] = compWeightMap(Frame,Cw, Sw, Ew, Frame_index)
% compWeightMap computes the Weight Map of the input frame 
% @param Frame: individual frame from video
% @param Cw: the contrast map weight
% @param Sw: Saturation map weight
% @param Ew: Exposedness map weight
% @param Frame_index: index of current frame
% @return wMap: weight map of the frame (product c s e maps)
fprintf('computing weight map of index %d\n', Frame_index);
% compute individual maps 
cm = cMap(Frame, Cw);
sm = sMap(Frame, Sw);
em = eMap(Frame, Ew);
wMap = (cm.*sm.*em+1e-12); % add a small value to avoid division by zero
end
function f = pyramid_filter;
f = [.0625, .25, .375, .25, .0625];
end
function [cm] = cMap(Frame,Cw)
% cMap computes the contrast map of the input frame given a contrast map
% weight.It provide a edge struction of the frame.
% @param Frame: input video Frame
% @param Cw: input contrast weight
% @return cm: contrast map (edge)
kernal = [0 1 0; 1 -4 1; 0 1 0]; % 2D laplacian filter
cm = Frame;
cm = rgb2gray(cm); % convert rgb frame to grayscale normalized image
cm = imfilter(cm,kernal);  % convolve laplacian operator on image
cm = abs(cm); % absolute value of the filtered image
%cm = normalize(cm);
cm = cm.^Cw; % apply weight 
end

function [sm] = sMap(Frame,Sw)
% sMap computes the saturation map of the input frame given a saturation map
% where saturation is defined as the deviation between each color channels
% and the mean value at each pixel.
% weight
% @param Frame: input video Frame
% @param Sw: input saturation weight
% @return cm: saturation map
[h,w,c] = size(Frame);
sm = zeros(h,w);
for i = 1:h
    for j = 1:w
        R = Frame(i,j,1);
        G = Frame(i,j,2);
        B = Frame(i,j,3);
        tmp_avg = (R+G+B)/3.0;
        sm(i,j) = sqrt(((R-tmp_avg)^2+(G-tmp_avg)^2+(B-tmp_avg)^2)/3.0); %compute the standard deviation of 3 channel at given pixel
    end
end
%sm = sm - min(sm(:)) ;
%sm = sm / max(sm(:)) ;
sm = (sm).^Sw; %  normalize the map and apply weight       
end

function [em] = eMap(Frame,Ew)
% eMap computes the exposedness map of the input frame given a map
% weight
% @param Frame: input video Frame
% @param Cw: input exposedness weight
% @return cm: exposedness map
variance = 0.2*0.2; % using the paper's implementation parameter
[h,w,c] = size(Frame);
em = zeros(h,w);
for i = 1:h
    for j = 1:w
        R = Frame(i,j,1);
        G = Frame(i,j,2);
        B = Frame(i,j,3);
        R = exp(-0.5*(R-0.5)^2/variance); % apply gaussian curve to each channel of the pixel
        G = exp(-0.5*(G-0.5)^2/variance);
        B = exp(-0.5*(B-0.5)^2/variance);
        em(i,j) = (R*G*B)^Ew; % multiply the result and apply weight pixelwise
    end
end
end

function R = downsample(I, filter)
border_mode = 'symmetric';
R = imfilter(I,filter,border_mode);     %horizontal
R = imfilter(R,filter',border_mode);    %vertical
r = size(I,1);
c = size(I,2);
R = R(1:2:r, 1:2:c, :);  
end

function pyr = laplacian_pyramid(I,nlev)

% recursively build pyramid
pyr = cell(nlev,1);
filter = pyramid_filter;
J = I;
for l = 1:nlev - 1
    % apply low pass filter, and downsample
    I = downsample(J,filter);
    odd = 2*size(I) - size(J);  % for each dimension, check if the upsampled version has to be odd
    % in each level, store difference between image and upsampled low pass version
    pyr{l} = J - upsample(I,odd,filter);
    J = I; % continue with low pass image
end
pyr{nlev} = J; % the coarest level contains the residual low pass image
end


function pyr_g = gaussian_pyramid(I,nlev)
% start by copying the image to the finest level
pyr_g = cell(nlev,1);
pyr_g{1} = I;

% recursively downsample the image
filter = pyramid_filter;
for l = 2:nlev
    I = downsample(I,filter);
    pyr_g{l} = I;
end
end

function R_u = upsample(I,odd,filter)

% increase resolution
I = padarray(I,[1 1 0],'replicate'); % pad the image with a 1-pixel border
r = 2*size(I,1);
c = 2*size(I,2);
k = size(I,3);
R_u = zeros(r,c,k);
R_u(1:2:r, 1:2:c, :) = 4*I; % increase size 2 times; the padding is now 2 pixels wide

% interpolate, convolve with separable filter
R_u = imfilter(R_u,filter);     %horizontal
R_u = imfilter(R_u,filter');    %vertical

% remove the border
R_u = R_u(3:r - 2 - odd(1), 3:c - 2 - odd(2), :);
end

function R_r = reconstruct_laplacian_pyramid(pyr)

nlev = length(pyr);

% start with low pass residual
R_r = pyr{nlev};
filter = pyramid_filter;
for l = nlev - 1 : -1 : 1
    % upsample, and add to current level
    odd = 2*size(R_r) - size(pyr{l});
    R_r = pyr{l} + upsample(R_r,odd,filter);
end
end

