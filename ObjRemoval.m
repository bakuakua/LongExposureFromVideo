function ObjRemoval(Frames,info,ds)
% ObjRemoval detect the non moving objects as background, remove the moving
% part. Using Gaussian model.
% @param Frames: An array of images
% @para info: [height, width, numberofframes, duration] 
% @param ds: down sample the video, only take frame for each n frames

nFrames = info(3);
bg_frame = zeros(info(1),info(2),3);     % initialize background frame
pixel_sample_density = im2bw(bg_frame);  % binary value

foregroundDetector = vision.ForegroundDetector('NumGaussians', 3, ...
    'NumTrainingFrames', 50);

for k = 1:ds:nFrames
    frame = double(Frames(:,:,:,k));
    foreground = step(foregroundDetector, frame);
    diff_frame = 1-foreground;
    pixel_sample_density = pixel_sample_density + diff_frame;
    
    nonmoving = double(Frames(:,:,:,k));
    nonmoving(:,:,1) = nonmoving(:,:,1).*diff_frame; % apply background mask
    nonmoving(:,:,2) = nonmoving(:,:,2).*diff_frame;
    nonmoving(:,:,3) = nonmoving(:,:,3).*diff_frame;
    bg_frame = bg_frame + nonmoving;
end
bg_frame(:,:,1) = bg_frame(:,:,1)./pixel_sample_density;
bg_frame(:,:,2) = bg_frame(:,:,2)./pixel_sample_density;
bg_frame(:,:,3) = bg_frame(:,:,3)./pixel_sample_density;
imshow(uint8(bg_frame))

end