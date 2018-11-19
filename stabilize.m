% Input video file which needs to be stabilized.
VideoPath = 'P7230138.MOV';
[Frames,info] = videoToFrames(VideoPath);
%%
%red_frames = zeros(info(1),info(2),info(3));
%green_frames = zeros(info(1),info(2),info(3));
%blue_frames = zeros(info(1),info(2),info(3));

for i =1:info(3)
    tmp = Frames(:,:,:,i);
    red_frames(:,:,i) = tmp(:,:,1);
    green_frames(:,:,i) = tmp(:,:,2);
    blue_frames(:,:,i) = tmp(:,:,3);
end
%% stream_process

[r_channel, max_wr ,max_hr] = stream_process(info, red_frames);
[g_channel, max_wg ,max_hg] = stream_process(info, green_frames);
[b_channel, max_wb ,max_hb] = stream_process(info, blue_frames);
%% output
for i = 1:info(3)
    res_img(:,:,:,i) = cat(3,r_channel(:,:,i),g_channel(:,:,i),b_channel(:,:,i));
end
%%
W = info(2); % Width in pixels
H = info(1); % Height in pixels
for i = 1:info(3)
    img_crop(:,:,:,i) = imcrop(res_img(:,:,:,i),[20 20 W-50 H-50]);
end

