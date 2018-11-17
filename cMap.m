function [cm] = cMap(Frame,Cw)
% cMap computes the contrast map of the input frame given a contrast map
% weight.It provide a edge struction of the frame.
% @param Frame: input video Frame
% @param Cw: input contrast weight
% @return cm: contrast map (edge)
kernal = fspecial('laplacian',0.2); % 2D laplacian filter with default alpha
cm = Frame;
cm = im2double(rgb2gray(cm)); % convert rgb frame to grayscale normalized image
cm = imfilter(cm,kernal);  % convolve laplacian operator on image
cm = abs(cm); % absolute value of the filtered image
cm = cm.^Cw; % apply weight 
end

