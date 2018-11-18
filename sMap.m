function [sm] = sMap(Frame,Sw, Frame_index)
% sMap computes the saturation map of the input frame given a saturation map
% where saturation is defined as the deviation between each color channels
% and the mean value at each pixel.
% weight
% @param Frame: input video Frame
% @param Sw: input saturation weight
% @return cm: saturation map
fprintf('computing saturation map from frame %d\n', Frame_index);
[h,w,c] = size(Frame);
Frame = im2double(Frame);
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
sm = (im2double(sm)).^Sw; %  normalize the map and apply weight       
end

