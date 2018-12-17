% %% Step 1. Read Frames from a Movie File
% filename = "stable.MOV"
% hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
% 
% imgA = step(hVideoSrc); % Read first frame into imgA
% imgB = step(hVideoSrc); % Read second frame into imgB
% 
% %show difference between first two image
% figure; imshowpair(imgA,imgB,'ColorChannels','red-cyan');
% title('Color composite (frame A = red, frame B = cyan)');
% 
% %% Step 2. Collect Salient Points from Each Frame
% ptThresh = 0.35;
% pointsA = detectFASTFeatures(imgA, 'MinContrast', ptThresh);
% pointsB = detectFASTFeatures(imgB, 'MinContrast', ptThresh);
% 
% % Display corners found in images A and B.
% figure; imshow(imgA); hold on;
% plot(pointsA);
% title('Corners in A');
% 
% figure; imshow(imgB); hold on;
% plot(pointsB);
% title('Corners in B');
% 
% % Extract FREAK descriptors for the corners
% [featuresA, pointsA] = extractFeatures(imgA, pointsA);
% [featuresB, pointsB] = extractFeatures(imgB, pointsB);
% 
% %%
% % Match features which were found in the current and the previous frames.
% % Since the FREAK descriptors are binary, the |matchFeatures| function 
% % uses the Hamming distance to find the corresponding points.
% indexPairs = matchFeatures(featuresA, featuresB);
% pointsA = pointsA(indexPairs(:, 1), :);
% pointsB = pointsB(indexPairs(:, 2), :);
% %%
% figure; showMatchedFeatures(imgA, imgB, pointsA, pointsB);
% legend('A', 'B');
% 
% %% Step 4. Estimating Transform from Noisy Correspondences
% [tform, pointsBm, pointsAm] = estimateGeometricTransform(...
%     pointsB, pointsA, 'affine');
% imgBp = imwarp(imgB, tform, 'OutputView', imref2d(size(imgB)));
% pointsBmp = transformPointsForward(tform, pointsBm.Location);
% 
% figure;
% showMatchedFeatures(imgA, imgBp, pointsAm, pointsBmp);
% legend('A', 'B');
% 
% %% Step 5. Transform Approximation and Smoothing
% % Extract scale and rotation part sub-matrix.
% H = tform.T;
% R = H(1:2,1:2);
% % Compute theta from mean of two possible arctangents
% theta = mean([atan2(R(2),R(1)) atan2(-R(3),R(4))]);
% % Compute scale from mean of two stable mean calculations
% scale = mean(R([1 4])/cos(theta));
% % Translation remains the same:
% translation = H(3, 1:2);
% % Reconstitute new s-R-t transform:
% HsRt = [[scale*[cos(theta) -sin(theta); sin(theta) cos(theta)]; ...
%   translation], [0 0 1]'];
% tformsRT = affine2d(HsRt);
% 
% imgBold = imwarp(imgB, tform, 'OutputView', imref2d(size(imgB)));
% imgBsRt = imwarp(imgB, tformsRT, 'OutputView', imref2d(size(imgB)));
% 
% figure(2), clf;
% imshowpair(imgBold,imgBsRt,'ColorChannels','red-cyan'), axis image;

%%


filename = "Videos/P7230137.MOV";
hVideoCol = vision.VideoFileReader(filename, 'ImageColorSpace', 'RGB');
videoCol = step(hVideoCol);

hVideoSrc = vision.VideoFileReader(filename, 'ImageColorSpace', 'Intensity');
% Reset the video source to the beginning of the file.
%reset(hVideoSrc);
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


while ~isDone(hVideoSrc) && ii < 5
    % Read in new frame

    imgA = imgB; % z^-1
    imgAp = imgBp; % z^-1
    imgB = step(hVideoSrc);
    movMean = movMean + imgB;

    % Estimate transform from frame A to frame B, and fit as an s-R-t
    H = cvexEstStabilizationTform(imgA,imgB);
    HsRt = cvexTformToSRT(H);
    Hcumulative = HsRt * Hcumulative;
    imgBp = imwarp(imgB,affine2d(Hcumulative),'OutputView',imref2d(size(imgB)));
    rgbImage2 = ind2rgb(imgBp, map);
    output{ii} = rgbImage2;
    correctedMean = correctedMean + imgBp;   
    ii = ii+1;
    
end
correctedMean = correctedMean/(ii-2);
movMean = movMean/(ii-2);

%release memory
release(hVideoSrc);

% Here you call the release method on the objects to close any open files
% and release memory.
%release(hVPlayer);
