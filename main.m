% This script encapsulates the whole long exposure creation pipeline
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
fontSize = 22;
VideoPath = 'Videos/P7230137.MOV'; %path to input video
[Frames,info] = videoToFrames(VideoPath);
%% image stablization
% this section of the pipeline intends to remove the movement of the
% background to create pre-processed frames with static background





%% create Long exposure image from frames
% this section of the pipeline intends to create a simulated long exposure
% image from frames.




%% moving object detection and removal
% this section of the pipeline intends to detect all the moving object and
% remove them in the video.

downsample = 5;   
subplot(221)
median(Frames,info,downsample)
title('Average Pic')
subplot(222)
ObjRemoval_ave(Frames,info,downsample)
title('removed obj with average diff')
subplot(223)
ObjRemoval(Frames,info,downsample)
title('removed obj with Gaussian model')



