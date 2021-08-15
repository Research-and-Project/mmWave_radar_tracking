%% Data cluster for millimeter radar data
% data format
% *data format:* 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
% 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS

%% env init
clear, clc, close all
addpath(genpath('./utils'));

%% param
% path and data
result_dir = './analysis/';
data_dir = './data/weifu_zyk_radar_data/';
% data_item = '单人8字2pcd/';
% data_item = '单人横穿2pcd/';
% data_item = '单人直行2pcd/';
data_item = '两人交互pcd/';

start_frame = 1;
end_frame = 1000000;
traj_dim = 2; % 2d/3d trajectory 

% analysis setting
loc_analysis_flag = 0;
doppler_analysis_flag = 1;
noise_analysis_flag = 0;

rec_point = start_frame:250:end_frame; % record per 250 frame


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



%% data init
% ---- file info ----
datas = dir([data_dir data_item '*.txt']);
data_names = {datas.name};
data_num = length(data_names);
end_frame = min(data_num, end_frame);
if start_frame>end_frame
	error("start frame over range")
end

% ---- make dir ----
data_save_dir = [result_dir data_item 'data_feature/'];
if ~exist(data_save_dir,'dir')
	mkdir(data_save_dir)
end

% ---- doppler data ----
doppler_range = NaN(end_frame-start_frame+1, 3); % frame_idx, min_abs_value, max_abs_value

% KF = []; % KF handle
% det_loc = []; % detected location
% meas_traj = NaN(start_frame-1,traj_dim); % trajectory points
% kf_traj = NaN(start_frame-1,traj_dim);   % KF corrected trajectory points
% isDetected = false; % detected flag


%% ---- statistic analysis ----
for k = start_frame:end_frame
	% ---- load data
    frame = importdata([data_dir data_item data_names{k}]);
	
	
	% 1 - loc_analysis
	if loc_analysis_flag
		
	end
	
	% 2 - doppler_analysis
	if doppler_analysis_flag
		figure(2)
		set(gcf,'WindowState','maximized')
		
		subplot(311)
		hist(frame(:,7))
		axis([-5,5, 0, 1500]);
		title(['Frame #' num2str(k) ' doppler distribution']);
		xlabel('doppler'), ylabel('count')
		
		
		doppler_range(k-start_frame+1,1) = k;
		doppler_range(k-start_frame+1,2) = min(abs(frame(:,7)), [], 'all');
		doppler_range(k-start_frame+1,3) = max(abs(frame(:,7)), [], 'all');
		
		subplot(312)
		plot(doppler_range(:,2))
		axis([k-50,k,0,5]);
		title(['Frame #' num2str(k) ' doppler min abs']);
		xlabel('frame #'), ylabel('doppler min abs')

		subplot(313)
		plot(doppler_range(:,3))
		axis([k-50,k,0,5]);
		title(['Frame #' num2str(k) ' doppler max abs']);
		xlabel('frame #'), ylabel('doppler max abs')
		
		figtitle([data_item(1:end-1) ' - doppler analysis'],'color','blue','linewidth',4,'fontsize',10);
		
		% save fig
		if ismember(k,rec_point)
			saveas(gcf,[data_save_dir 'doppler_analysis_f' num2str(k) '.png'])
			disp(['doppler_analysis fig saved to: ' data_save_dir])
		end

	end
	
	if noise_analysis_flag
		frame_clean = point_cloud_denoise(frame, param_denoise);
		disp(['doppler points num: ' num2str(size(frame_clean,1))])

		% 3d denoise result
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
		
% 		% save fig
% % 		saveas(gcf,[data_save_dir 'doppler_max_abs.png'])
% % 		disp(['doppler_max_abs saved to: ' data_save_dir])
	end
	


	
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
	
% 	figtitle(data_item(1:end-1),'color','blue','linewidth',4,'fontsize',15);
    drawnow
    pause(show_delay)
	
end


%% statistic analysis
% doppler analysis
figure
plot(doppler_range(:,3))
xlabel('frame #'), ylabel('doppler max abs'),grid on
title('doppler max abs')
% save fig
saveas(gcf,[data_save_dir 'doppler_max_abs.png'])
disp(['doppler_max_abs saved to: ' data_save_dir])

% save data
save([data_save_dir 'doppler_analysis_feature.mat'], 'doppler_range')
disp(['analysis-doppler_range data saved to: ' data_save_dir])




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