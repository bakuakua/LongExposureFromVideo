% This script encapsulates the whole long exposure creation pipeline
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
fontSize = 22;

FramesPath = 'Videos/waterfall1'; %path to input video

%% image stablization
% this section of the pipeline intends to remove the movement of the
% background to create pre-processed frames with static background





%% create Long exposure image from frames
% this section of the pipeline intends to create a simulated long exposure
% image from frames.
Frames_all = loadFrames(FramesPath);
ds = 5;
[h,len] = size(Frames_all);
Frames = cell(1,floor(len/ds));
j = 1;
for i = 1:ds:len
    Frames{j} = Frames_all{i};
    j = j+1;
end
img_r = ExposureFusion(Frames,1.0,1.0,0);
img_c = average(Frames,floor(len/ds));
figure
title('Time lapse fused vs averaged');
imshowpair(img_r,uint8(img_c), 'montage');
%% moving object detection and removal
% 




%% simply averaging the frames







