function average_frame = average(Frames,nFrames)
% @param Frames: An array of images
% @param nFrames: number of Frames 
% @param ds: down sample the video, only take frame for each n frames
[h, w, c] = size(Frames{1}); 
average_frame = zeros(h,w,c);     % initialize average frame
for k = 1:nFrames
    average_frame = average_frame + double(Frames{k});
end
average_frame = average_frame/(nFrames); 

