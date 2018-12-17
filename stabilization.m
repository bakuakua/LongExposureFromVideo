function [output] = stabilization(VideoPath)

hVideoSrc = vision.VideoFileReader(VideoPath, 'ImageColorSpace', 'Intensity');
count = 0;
while ~isDone(hVideoSrc)
    videoFrame = step(hVideoSrc);
    count = count + 1;
end           
reset(hVideoSrc);

output = cell(1,count);
% Process all frames in the video
movMean = step(hVideoSrc);
output{1} = movMean;
imgB = movMean;
imgBp = imgB;
output{2} = movMean;
correctedMean = imgBp;
ii = 2;
Hcumulative = eye(3);


while ~isDone(hVideoSrc) && ii < count
    % Read in new frame
    fprintf('processing at %d frames\n', ii);
    imgA = imgB; % z^-1
    imgAp = imgBp; % z^-1
    imgB = step(hVideoSrc);
    movMean = movMean + imgB;

    % Estimate transform from frame A to frame B, and fit as an s-R-t
    H = cvexEstStabilizationTform(imgA,imgB);
    HsRt = cvexTformToSRT(H);
    Hcumulative = HsRt * Hcumulative;
    imgBp = imwarp(imgB,affine2d(Hcumulative),'OutputView',imref2d(size(imgB)));
    output{ii} = imgBp;
    correctedMean = correctedMean + imgBp;   
    ii = ii+1;
    
end
correctedMean = correctedMean/(ii-2);
movMean = movMean/(ii-2);

%release memory
release(hVideoSrc);
end

