function median(Frames,info,ds)
% @param Frames: An array of images
% @para info: [height, width, numberofframes, duration] 
% @param ds: down sample the video, only take frame for each n frames

nFrames = info(3);
average_frame = zeros(info(1),info(2),3);     % initialize average frame
for k = 1:ds:nFrames
    average_frame = average_frame + double(Frames(:,:,:,k));
end
average_frame = average_frame/(nFrames/ds); 
imshow(uint8(average_frame))