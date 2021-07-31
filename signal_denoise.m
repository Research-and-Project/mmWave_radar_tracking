%% Data cluster for millimeter radar data
% data format
% *data format:* 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
% 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS

%% env init
clear, clc, close all
addpath(genpath('./utils'));

%% param
% path and data
result_dir = './result/';
data_dir = './data/weifu_zyk_radar_data/';
data_item = '单人8字1pcd/';
% data_item = '单人横穿1pcd/';
% data_item = '单人直行1pcd/';
% data_item = '两人交互pcd/';

start_frame = 1;
end_frame = 1000000;
traj_dim = 2; % 2d/3d trajectory 

% denoise
param_denoise.dpl_thr = [0.15 Inf];

% cluster
epsilon = 4;
MinPts = 20;
obj_count = 2;

% Kalman filter
motion_type = 'ConstantVelocity'; % 'ConstantVelocity' | 'ConstantAcceleration'
param_kf = getDefaultKFParameters(motion_type);
% param.initialEstimateError  = 1E5 * ones(1, 2);
% param.motionNoise           = [25, 10];
% param.measurementNoise      = 25;	

% show
axis_range = [];
% axis_range = [-20, 40, 0, 80, -5, 10];
% axis_range = [-20, 20, 0, 20, -10, 10];
show_delay = 0.0;



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
det_loc = []; % detected location
meas_traj = NaN(start_frame-1,traj_dim); % trajectory points
kf_traj = NaN(start_frame-1,traj_dim);   % KF corrected trajectory points
isDetected = false; % detected flag

figure;

for k = start_frame:end_frame
	% ---- load data
    frame = importdata([data_dir data_item data_names{k}]);
	
	% ---- denoise ----
	frame_clean = point_cloud_denoise(frame, param_denoise);
    disp(['doppler points num: ' num2str(size(frame_clean,1))])
	
	% denoise result
	figure(1)
	subplot(121) % 3d
	hold on
	scatter3(frame(:,1),frame(:,2),frame(:,3),'go')
	scatter3(frame_clean(:,1),frame_clean(:,2),frame_clean(:,3),'ro','filled')
	axis(axis_range); grid on, view(3)
	title(['Frame #' num2str(k) ' 3D view']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	
	subplot(122) % 2d
	hold on
	scatter3(frame(:,1),frame(:,2),frame(:,3),'go')
	scatter3(frame_clean(:,1),frame_clean(:,2),frame_clean(:,3),'ro','filled')
	axis(axis_range); grid on, view(2)
	title(['Frame #' num2str(k) ' 2D view']);
	xlabel('X'), ylabel('Y'), zlabel('Z');	
	
	
	%{
	%  [ToDo] TBD
	if size(frame_clean, 1) < 4
		isDetected = false;
	end

	idx = DBSCAN(frame_clean(:,[1,2]),epsilon,MinPts); % DBSCAN Cluster
	
	% delete noise points cluster(idx==0)
	frame_clean(idx==0,:) = []; 
	idx(idx==0,:) = [];

% 	[idx,C] = kmeans(frame_doppler(:,[1,2]),2); % K-Means Cluster

	[idx, Dg] = cluster_idx_arranege(frame_clean(:,[1,2]), idx);
	disp(['cluster count:' num2str(numel(unique(idx)))])

	if isempty(idx)
		isDetected = false;
	else
		isDetected = true;
	end
	
	if isDetected
	% 	min_idx = min(idx,[],'all');
	% 	frame_obj = frame_doppler(idx==min_idx,:);
		frame_obj = frame_clean(idx<=obj_count,:);

		subplot(121)
	% 	scatter3(frame_doppler(:,1),frame_doppler(:,2),frame_doppler(:,3))
	% 	gscatter(frame_doppler(:,1),frame_doppler(:,2),idx,'rgbcmykw')

		gscatter3(frame_clean(:,1),frame_clean(:,2),frame_clean(:,3),idx,[],[],10,'on')

		% calc bounding box
		rect_min = min(frame_obj(:,1:3),[],1);
		rect_max = max(frame_obj(:,1:3),[],1);
		rect_size = rect_max - rect_min;
		rect_center = (rect_min + rect_max)/2;
		det_loc = rect_center(1:traj_dim); 

		% show bounding box
		plotBoundingbox(rect_min, rect_size, [0 0 1], 'obj1', k, axis_range)
	
	else
		det_loc = NaN(1,traj_dim);
	end
	
	% Kalman Filter
	[kf_loc, KF, states] = KF_tracking(det_loc, KF, param_kf);
	if isempty(kf_loc)
		kf_loc = NaN(1,traj_dim);
	end
	
	meas_traj(k,:) = det_loc;
	kf_traj(k,:) = kf_loc;
	
	
	% show trajectory
	subplot(122)
% 	cmpTraj(meas_traj, kf_traj, 'plot', 'xlim', axis_range(1:2), 'ylim', axis_range(3:4));
	plotTraj(kf_traj, k, axis_range)

	%}
	
	figtitle(data_item(1:end-1),'color','blue','linewidth',4,'fontsize',15);
    drawnow
    pause(show_delay)
	
end

%% save data
% data_save_dir = [result_dir data_item 'ResData/'];
% if ~exist(data_save_dir,'dir')
% 	mkdir(data_save_dir)
% end
% save([data_save_dir 'traj.mat'], 'meas_traj', 'kf_traj')
% disp(['result data saved to: ' data_save_dir])




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

function plotBoundingbox(rect_p, rect_size, clr, lgd, frame_idx, axis_range)
	plotcube(rect_size, rect_p, 0.1, clr,lgd)
	title(['Frame #' num2str(frame_idx) ' - 3D detection']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	axis(axis_range);
	view(3);
	grid on
end

function plotTraj(traj, frame_idx, axis_range)
	if size(traj, 2)==2
		plot(traj(:,1),traj(:,2),'r-x','MarkerSize',5,'LineWidth',1)
	elseif size(traj, 2)==3
		plot3(traj(:,1),traj(:,2),traj(:,3),'r-x','MarkerSize',5,'LineWidth',1)
	end
	
	title(['Frame #' num2str(frame_idx) ' - XY trajectory']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	legend('KF Traj.')
	axis(axis_range);  
	view(2);
	grid on
end