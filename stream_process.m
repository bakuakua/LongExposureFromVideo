
function [color_channel,maxi_width, maxi_height] = stream_process(info, color_frames)
%%
% Create a template matcher System object to compute the location of the
% best match of the target in the video frame. We use this location to find
% translation between successive video frames.
hTM = vision.TemplateMatcher('ROIInputPort', true, ...
                            'BestMatchNeighborhoodOutputPort', true);
                        
%%
% Create a System object to display the original video and the stabilized
% video.
hVideoOut = vision.VideoPlayer('Name', 'Video Stabilization');
hVideoOut.Position(1) = round(0.4*hVideoOut.Position(1));
hVideoOut.Position(2) = round(1.5*(hVideoOut.Position(2)));
hVideoOut.Position(3:4) = [650 350];

%%
% Here we initialize some variables used in the processing loop.
pos.template_orig = [150 5]; % [x y] upper left corner
pos.template_size = [22 18];   % [width height]
pos.search_border = [15 10];   % max horizontal and vertical displacement
pos.template_center = floor((pos.template_size-1)/2);
pos.template_center_pos = (pos.template_orig + pos.template_center - 1);
%fileInfo = info(hVideoSource);
W = info(2); % Width in pixels
H = info(1); % Height in pixels
BorderCols = [1:pos.search_border(1)+4 W-pos.search_border(1)+4:W];
BorderRows = [1:pos.search_border(2)+4 H-pos.search_border(2)+4:H];
sz = [info(2), info(1)];
TargetRowIndices = ...
  pos.template_orig(2)-1:pos.template_orig(2)+pos.template_size(2)-2;
TargetColIndices = ...
  pos.template_orig(1)-1:pos.template_orig(1)+pos.template_size(1)-2;
SearchRegion = pos.template_orig - pos.search_border - 1;
Offset = [0 0];
Target = zeros(18,22);


%% Stream Processing Loop
% This is the main processing loop which uses the objects we instantiated
% above to stabilize the input video.
maxi_width = 0;
maxi_height = 0;

for i=1:info(3)
    %%%%%%%%%%%
    input = color_frames(:,:,i);
    % Find location of Target in the input video frame
    if i == 1
      Idx = int32(pos.template_center_pos);
      MotionVector = [0 0];
    else
      IdxPrev = Idx;

      ROI = [SearchRegion, pos.template_size+2*pos.search_border];
      Idx = hTM(input,Target,ROI);
      
      MotionVector = double(Idx-IdxPrev);
    end

    [Offset, SearchRegion] = updatesearch(sz, MotionVector, ...
        SearchRegion, Offset, pos);
    
    tmp_width = [maxi_width Offset(1)];
    tmp_height = [maxi_height Offset(2)];

    maxi_width = min(tmp_width);
    maxi_height = min(tmp_height);

    % Translate video frame to offset the camera motion
    Stabilized = imtranslate(input, Offset, 'linear');
   
    Target = Stabilized(TargetRowIndices, TargetColIndices);

    % Add black border for display
    Stabilized(:, BorderCols) = 0;
    Stabilized(BorderRows, :) = 0;

    TargetRect = [pos.template_orig-Offset, pos.template_size];
    SearchRegionRect = [SearchRegion, pos.template_size + 2*pos.search_border];
    color_channel(:,:,i) = Stabilized;
    color_channel(:,:,i) = uint8(color_channel(:,:,i));

end
