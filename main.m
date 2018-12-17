% This script encapsulates the whole long exposure creation pipeline
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
fontSize = 22;
Path1 = 'Videos/night_car2.MOV'; %path to input video
[Frames_obj,info] = videoToFrames(Path1);
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
median(Frames_obj,info,downsample)
title('Average Pic')
subplot(222)
ObjRemoval_ave(Frames_obj,info,downsample)
title('removed obj with average diff')
subplot(223)
ObjRemoval(Frames_obj,info,downsample)
title('removed obj with Gaussian model')
%% adding weight to a moving object to make a clear long exposure photo

downsample = 5; 
weight = 5;
addWeight(Frames_obj,info,downsample,weight)


