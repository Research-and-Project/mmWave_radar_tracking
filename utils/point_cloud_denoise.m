function frame_clean = point_cloud_denoise(frame, param)
% POINT_CLOUD_DENOISE - denoise the point cloud from the mmWave radar based
% on manual designed feature
% 
% Input
%	frame		point cloud in a frame, 2D matrix
%	param		denoising params, struct
%		dpl_thr - dopple threshold, 2d vector
%		
% Output
%	frame_clean	denoised point cloud in the input frame, 2D matrix
% 
% Reference
%	data format: 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
%				 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS
% 
% Zhihong Zhang, 2021-07-30

%% param assign
if ~isfield(param, 'dpl_thr') || isempty(param.dpl_thr); param.dpl_thr = [-0.1 0.1]; end

% doppler threshold
frame_now = frame(param.dpl_thr(1)<frame(:,7)<param.dpl_thr(2),:);




frame_clean = frame_now;

end