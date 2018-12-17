function addWeight(Frames,info,ds,weight)
% ObjRemoval detect the non moving objects as background, remove the moving
% part.
% @param Frames: An array of images
% @param info: [height, width, number of frames, duration] 
% @param ds: down sample the video, only take frame for each n frames
% @param weight: weight that add to the moving object

nFrames = info(3);
average_frame = zeros(info(1),info(2),3);     % initialize average frame
for k = 1:ds:nFrames
    average_frame = average_frame + double(Frames(:,:,:,k));
end
average_frame = average_frame/(nFrames/ds);        % take average

% Re-calculates the background frame and remove moving objects in the calculation
bg_frame = zeros(info(1),info(2),3);     % initialize background frame
pixel_sample_density = im2bw(bg_frame);  % binary value
diff_frame = zeros(info(1),info(2),3);   % initialize 
for k = 1:ds:nFrames
    diff_frame = imabsdiff(double(Frames(:,:,:,k)), average_frame); % absolute difference between current frame and average
    level = graythresh(Frames(:,:,:,k));
    diff_frame = im2bw(uint8(diff_frame),level);     % substract foreground
    pixel_sample_density = pixel_sample_density + diff_frame;
    
    nonmoving = double(Frames(:,:,:,k));
    nonmoving(:,:,1) = nonmoving(:,:,1) + weight*nonmoving(:,:,1).*diff_frame; % apply mask
    nonmoving(:,:,2) = nonmoving(:,:,2) + weight*nonmoving(:,:,2).*diff_frame;
    nonmoving(:,:,3) = nonmoving(:,:,3) + weight*nonmoving(:,:,3).*diff_frame;
    bg_frame = bg_frame + nonmoving;
end

bg_frame = bg_frame/(nFrames/ds); 
imshow(uint8(bg_frame))

end