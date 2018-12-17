filename = "Videos/P7230137.MOV";
hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'RGB');
videoCol = step(hVideoSrc);
red_frames = videoCol(:,:,1);
green_frames = videoCol(:,:,2);
blue_frames = videoCol(:,:,3);

count = 1;
while ~isDone(hVideoSrc)
    videoFrame = step(hVideoSrc);
    count = count + 1;
end           
reset(hVideoSrc);

output = cell(1,count);
% Process all frames in the video
movMean = step(hVideoSrc);
movMean_r = movMean(:,:,1);
movMean_g = movMean(:,:,2);
movMean_b = movMean(:,:,3);

output{1} = movMean;
imgB_r = movMean_r;
imgB_g = movMean_g;
imgB_b = movMean_b;
imgBp_r = movMean_r;
imgBp_g = movMean_g;
imgBp_b = movMean_b;

output{2} = movMean;
correctedMean_r = imgBp_r;
correctedMean_g = imgBp_g;
correctedMean_b = imgBp_b;
ii = 2;
Hcumulative_r = eye(3);
Hcumulative_g = eye(3);
Hcumulative_b = eye(3);

while ~isDone(hVideoSrc) && ii < 5
    % Read in new frame

    imgA_r = imgB_r; 
    imgA_g = imgB_g; 
    imgA_b = imgB_b; 

    imgAp_r = imgBp_r; 
    imgAp_g = imgBp_g; 
    imgAp_b = imgBp_b; 

    imgB = step(hVideoSrc);
    imgB_r = imgB(:,:,1);
    imgB_g = imgB(:,:,2);
    imgB_b = imgB(:,:,3);   
    movMean_r = movMean_r + imgB_r;
    movMean_g = movMean_g + imgB_g;
    movMean_b = movMean_b + imgB_b;

    % Estimate transform from frame A to frame B, and fit as an s-R-t
    H_r = cvexEstStabilizationTform(imgA_r,imgB_r);
    H_g = cvexEstStabilizationTform(imgA_g,imgB_g);
    H_b = cvexEstStabilizationTform(imgA_b,imgB_b);

    HsRt_r = cvexTformToSRT(H_r);
    HsRt_g = cvexTformToSRT(H_g);
    HsRt_b = cvexTformToSRT(H_b);

    Hcumulative_r = HsRt_r * Hcumulative_r;
    Hcumulative_g = HsRt_g * Hcumulative_g;
    Hcumulative_b = HsRt_b * Hcumulative_b;

    imgBp_r = imwarp(imgB_r,affine2d(Hcumulative_r),'OutputView',imref2d(size(imgB_r)));
    imgBp_g = imwarp(imgB_g,affine2d(Hcumulative_g),'OutputView',imref2d(size(imgB_g)));
    imgBp_b = imwarp(imgB_b,affine2d(Hcumulative_b),'OutputView',imref2d(size(imgB_b)));
    imgout = cat(3,imgBp_r, imgBp_g, imgBp_b);
    output{ii} = imgout;    
    ii = ii+1;
    
end



