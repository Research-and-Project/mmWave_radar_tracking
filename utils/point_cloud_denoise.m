function frame_clean = point_cloud_denoise(frame, param)
% POINT_CLOUD_DENOISE - denoise the point cloud from the mmWave radar based
% on manual designed feature
% 
% Input
%	frame		point cloud in a frame, 2D matrix
%	param		denoising params, struct
%		dpl_thr - dopple threshold, + scalar
%		loc_thr - location threshold, 6d vector, [xmin, xmax, ymin, ymax,
%		zmin, zmax]
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
if ~isfield(param, 'dpl_thr') || isempty(param.dpl_thr); param.dpl_thr = 0; end
if ~isfield(param, 'loc_thr') || isempty(param.loc_thr); param.loc_thr = [-50, 50, -50, 50, -50, 50]; end

% init
frame_now = frame;

% doppler threshold
frame_now = frame_now(abs(frame_now(:,7))>param.dpl_thr,:);

% location threshold
loc_condition = param.loc_thr(1)<frame_now(:,1) & frame_now(:,1)<param.loc_thr(2) &...
				param.loc_thr(3)<frame_now(:,2) & frame_now(:,2)<param.loc_thr(4) &...
				param.loc_thr(5)<frame_now(:,3) & frame_now(:,3)<param.loc_thr(6);
frame_now = frame_now(loc_condition,:);

% final
frame_clean = frame_now;

end