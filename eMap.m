function [em] = eMap(Frame,Ew,Frame_index)
% eMap computes the exposedness map of the input frame given a map
% weight
% @param Frame: input video Frame
% @param Cw: input exposedness weight
% @return cm: exposedness map
fprintf('computing Well-exposedness map from frame %d\n', Frame_index);
variance = 0.2*0.2; % using the paper's implementation parameter
[h,w,c] = size(Frame);
Frame = im2double(Frame);
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

