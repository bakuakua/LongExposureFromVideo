function [img] = ExposureFusion(Frames, Cw, Sw, Ew)
[h, w, c] = size(Frames{1});
W = zeros(h,w,20);
for i = 1:20
    W(:,:,i) = compWeightMap(im2double(Frames{i}), Cw, Sw, Ew, i);
end
W = W./repmat(sum(W,3),[1,1,20]);
img = W;
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


function [cm] = cMap(Frame,Cw)
% cMap computes the contrast map of the input frame given a contrast map
% weight.It provide a edge struction of the frame.
% @param Frame: input video Frame
% @param Cw: input contrast weight
% @return cm: contrast map (edge)
kernal = fspecial('laplacian',0.2); % 2D laplacian filter with default alpha
cm = Frame;
cm = rgb2gray(cm); % convert rgb frame to grayscale normalized image
cm = imfilter(cm,kernal);  % convolve laplacian operator on image
cm = abs(cm); % absolute value of the filtered image
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
sm = sm.^Sw; %  normalize the map and apply weight       
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




