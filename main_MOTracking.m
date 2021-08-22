%% MOTracking - Multiple Object Tracking
% data format
% *data format:* 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
% 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS

%% env init
clear, clc, close
addpath(genpath('./utils'));

%% param
% path and data
result_dir = './result/';
data_dir = './data/mmWave_radar_data/';
data_item = 'MOT/';

start_frame = 1;
end_frame = 10000;
traj_dim = 2; % 2d/3d trajectory 

% denoise
param_denoise.dpl_thr = 0.1;

% detection
param_det.minObjPoints = 30;
param_det.DBSCAN_epsilon = 0.3;
param_det.DBSCAN_MinPts = 30;
% param_det.max_obj_count = 2;

% Kalman filter
motion_type = 'ConstantVelocity'; % 'ConstantVelocity' | 'ConstantAcceleration'
param_kf = getDefaultKFParameters(motion_type);
param.initialEstimateError  = [200 50];
param.motionNoise           = [100 25];
param.measurementNoise      = 600;

% show
axis_range = [-5, 5, 0, 20, -2, 5];
% axis_range = [-20, 20, 0, 20, -10, 10];



%% denoise, cluster, KF_tracking
% ---- file info ----
datas = dir([data_dir data_item '*.txt']);
data_names = {datas.name};
data_num = length(data_names);
end_frame = min(data_num, end_frame);
if start_frame>end_frame
	error("start frame over range")
end

% ---- init ----
KF = []; % KF handle
tracks = initializeTracks(); % object tracks
nextId = 1; % next track id
meas_traj = NaN(start_frame-1,traj_dim); % trajectory points
% isDetected = false; % detected flag

figure;

for k = start_frame:end_frame
	% ---- load data
    frame = importdata([data_dir data_item data_names{k}]);
	
	% ---- denoise ----
	frame_clean = point_cloud_denoise(frame, param_denoise);
    disp(['clean points num: ' num2str(size(frame_clean,1))])
	
	% ---- detect ----
	[centroids, bboxes, obj_frame, obj_idx, obj_features] = getDetections(frame_clean,param_det);
	disp(['cluster count:' num2str(size(centroids,1))])
	
	% ---- track ---- 
	% predict new locations of last location (for cost calculation)
	tracks = predictNewLocationsOfTracks(tracks);

	% determine assignment of detection to tracks
	[assignments, unassignedTracks, unassignedDetections] = ...
	detectionToTrackAssignment(tracks, centroids, obj_frame, obj_idx, obj_features);

	% undate assigned tracks
    tracks = updateAssignedTracks(tracks, assignments, centroids, bboxes);
	
	% update unassigned tracks;
    tracks = updateUnassignedTracks(tracks, unassignedTracks);
	
	% update track states
	tracks = updateTrackStates(tracks);
	
	% create new tracks(tracks);
    [tracks,nextId] = createNewTracks(tracks, unassignedDetections, ...
		centroids, bboxes, obj_features, param_kf, nextId, k);

	% display track results
    showTrackingResults(obj_frame, obj_idx, tracks, k, axis_range, data_item(1:end-1))


	end


%% save data
data_save_dir = [result_dir data_item 'ResData/'];
if ~exist(data_save_dir,'dir')
	mkdir(data_save_dir)
end
save([data_save_dir 'track.mat'], 'meas_traj', 'tracks')
disp(['result data saved to: ' data_save_dir])




%% -------------------------------------------------------
%% sub functions
% get KF default parameters
function param = getDefaultKFParameters(motion_type)
	if nargin<1
		motion_type = 'ConstantVelocity';
	end
	
	param.motionModel = motion_type;
	param.initialLocation       = 'Same as first detection';
	
	if strcmp(motion_type, 'ConstantAcceleration')
	  param.initialEstimateError  = 1E5 * ones(1, 3);
	  param.motionNoise           = [25, 10, 1];
	  param.measurementNoise      = 25;
	elseif strcmp(motion_type, 'ConstantVelocity')
	  param.initialEstimateError  = 1E5 * ones(1, 2);
	  param.motionNoise           = [25, 10];
	  param.measurementNoise      = 25;		
	else
		error(['No assigned motion type - ' motion_type])
	end
end



% show tracking results
function showTrackingResults(obj_frame, obj_idx, tracks, frame_idx, axis_range, fig_name)
	% init
	% default bbox color
	clr = [1 0 0;
		   0 1 0;
		   0 0 1;
		   0 1 1;
		   1 0 1;
		   1 1 0];
	show_delay = 0.0;
	% show 3d condition
	minVisibleCount = 5;   % minimal consecutive appearing frame count
	maxInvisibleCount = 5; % maximal consecutive disappearing frame count
	
	% get normal tracks & effect tracks
	normal_track_ind = ...
			[tracks(:).totalVisibleCount] > minVisibleCount &...
			[tracks(:).consecutiveInvisibleCount] < maxInvisibleCount &...
			strcmp([tracks(:).state],"normal"); 
		
	normalTracks = tracks(normal_track_ind);

	effect_track_ind = ...
			[tracks(:).totalVisibleCount] > minVisibleCount &...
			~strcmp([tracks(:).state],"noise"); 
	effectTracks = tracks(effect_track_ind);
	
	clf(gcf) % clear figure before new display
	if ~ isempty(effectTracks) && ~isempty(obj_idx)
		% show 3d
		subplot(121)		
		% scatter3(obj_frame(:,1),obj_frame(:,2),obj_frame(:,3),10,'filled')
		gscatter3(obj_frame(:,1),obj_frame(:,2),obj_frame(:,3),obj_idx,0.3*ones(10,3),[],10,'off')

		for m = 1:length(normalTracks)
			plotBoundingbox(normalTracks(m).bbox(1:3), normalTracks(m).bbox(4:6), clr(normalTracks(m).id,:), ['obj' num2str(normalTracks(m).id)], axis_range)
		end
		legend
 		% view(2) % 2D view for debug

		% show 2D
		subplot(122)
		hold on
		for m = 1:length(effectTracks)
			plotTraj(effectTracks(m).traj_rec(:,1:2), axis_range, ['obj' num2str(effectTracks(m).id)], clr(effectTracks(m).id,:))
		end
		hold off
		legend
		
	end
	
	% fig info
	figtitle([fig_name ' - Frame #' num2str(frame_idx)],'color','blue','linewidth',4,'fontsize',15);
    drawnow
    pause(show_delay)

end