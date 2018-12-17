% This script encapsulates the whole long exposure creation pipeline
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;  % Make sure the workspace panel is showing.
fontSize = 22;
VideoPath = 'Videos/P7230137.MOV'; %path to input video
%[Frames,info] = videoToFrames(VideoPath);
%% image stablization
% this section of the pipeline intends to remove the movement of the
% background to create pre-processed frames with static background
output = stabilization(VideoPath);




%% create Long exposure image from frames
% this section of the pipeline intends to create a simulated long exposure
% image from frames.




%% moving object detection and removal
% 







