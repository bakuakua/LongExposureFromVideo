function [Frames,videoInfo] = videoToFrames(VideoPath)
% videoToFrames converts video file at input path to Frames
% depending on the size of the input video, downsampling and resizing might
% be applied.
% @param VideoPath: Path to input video file
% @return Frames: An array of images
% @return videoInfo: [height, width, numberofframes, duration]. 
fprintf('Converting input video into frames\n');
videoObject = VideoReader(VideoPath);
Frames = readFrame(videoObject);
FrameCount = 1;
while hasFrame(videoObject)
    currFrame = readFrame(videoObject);
    Frames = cat(4,Frames,currFrame);
    FrameCount=FrameCount+1;
end
videoInfo = [videoObject.Height, videoObject.width, FrameCount, videoObject.duration];
fprintf('video to frames completed with %d frames read into memory\n', FrameCount);
end
