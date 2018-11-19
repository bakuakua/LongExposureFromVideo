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
ObjRemoval(Frames,info,downsample)
%%
VideoPath = 'Videos/night_car2.MOV';
obj = VideoReader(VideoPath);
a = read(obj,[1 200]);
FrameCount = 200;
detector = vision.ForegroundDetector(...
       'NumTrainingFrames', 5, ...
       'InitialVariance', 30*30);
rgbSum = 0;
for fr = 1:FrameCount
    frame = read(obj,fr);
    for i = 1:3
        fgMask(:,:,i) = detector.step(frame(:,:,i));
        bgMask(:,:,i) = 1- fgMask(:,:,i);
        frameNo(:,:,i) = double(frame(:,:,i)) .* double(1-bgMask(:,:,i));
        thisFrame(:,:,i) = double(frameNo(:,:,i));
    end
    rgbSum = rgbSum + thisFrame;
end
rgbMean = rgbSum / FrameCount;
imshow(uint8(rgbMean));








