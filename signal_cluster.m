%% Data analysis for millimeter radar data
% data format
% *data format:* 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
% 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS

%% init
addpath(genpath('./utils'));

%% param
% path and setting
data_dir = './data/weifu_zyk_radar_data/';
data_item = '单人8字1pcd/';
% data_item = '单人横穿pcd/';
start_frame = 200;
end_frame = 750;
doppler_threshold = 0.1;

% cluster
epsilon = 4;
MinPts = 20;

% show
% view_vec = [0 0 1];
view_vec = [1 1 1];
axis_range = [-5, 5, 0, 20, -2, 5];
% axis_range = [-20, 20, 0, 20, -10, 10];
show_delay = 0.02;

% file info
datas = dir([data_dir data_item '*.txt']);
data_names = {datas.name};
data_num = length(data_names);
end_frame = min(data_num, end_frame);
if start_frame>end_frame
	error("start frame over range")
end

%% doppler analysis
% % ---- single test ----
% k = 250;
% % k = 200;
% frame=importdata([data_dir data_item data_names{k}]);
% frame_doppler = frame(abs(frame(:,7))>doppler_threshold,:); % doppler items
% % frame_doppler = frame;
% % kmeas
% [idx,C] = kmeans(frame_doppler(:,[1,2]),2);
% [idx, Dg] = cluster_idx_arranege(frame_doppler(:,[1,2]), idx);
% figure
% gscatter3(frame_doppler(:,1),frame_doppler(:,2),frame_doppler(:,3),idx)
% title(['K-Means Frame #' num2str(k)])
% axis(axis_range);
% view(view_vec);
% grid on
% 
% % dbscan
% [idx] = DBSCAN(frame_doppler(:,[1,2]),epsilon,MinPts);
% [idx, Dg] = cluster_idx_arranege(frame_doppler(:,[1,2]), idx);
% figure
% gscatter3(frame_doppler(:,1),frame_doppler(:,2),frame_doppler(:,3),idx)
% title(['DBSCAN Frame #' num2str(k)])
% axis(axis_range);
% view(view_vec);
% grid on

% return 

% ---- multi test ----
traj = []; % trajectory points
traj_k = 1;

figure
for k = start_frame:end_frame
    frame=importdata([data_dir data_item data_names{k}]);
    frame_doppler = frame(abs(frame(:,7))>doppler_threshold,:); % doppler items
    disp(['doppler points num: ' num2str(size(frame_doppler,1))])
	
	if size(frame_doppler, 1) < 4
		continue
	end
	
	idx = DBSCAN(frame_doppler(:,[1,2]),epsilon,MinPts);
	% delete noise points and cluster
	frame_doppler(idx==0,:) = []; 
	idx(idx==0,:) = [];
	
	if isempty(idx)
		continue
	end
	
% 	[idx,C] = kmeans(frame_doppler(:,[1,2]),2);
	[idx, Dg] = cluster_idx_arranege(frame_doppler(:,[1,2]), idx);
	min_idx = min(idx,[],'all');
	frame_obj = frame_doppler(idx==min_idx,:);
	
	subplot(121)
% 	scatter3(frame_doppler(:,1),frame_doppler(:,2),frame_doppler(:,3))
% 	gscatter(frame_doppler(:,1),frame_doppler(:,2),idx,'rgbcmykw')
	
	gscatter3(frame_doppler(:,1),frame_doppler(:,2),frame_doppler(:,3),idx,[],[],10,'on')
	
	% bounding box
	rect_min = min(frame_obj(:,1:3),[],1);
	rect_max = max(frame_obj(:,1:3),[],1);
	rect_size = rect_max - rect_min;
	rect_center = (rect_min + rect_max)/2;
	traj(traj_k,:) = rect_center; traj_k = traj_k + 1;
	
	plotcube(rect_size, rect_min, 0.1, [0 0 0],'obj1')
	title(['Frame #' num2str(k) ' - 3D detection']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	axis(axis_range);
	view(view_vec);
	grid on
	
	% show trajectory
	subplot(122)
	plot3(traj(:,1),traj(:,2),traj(:,3),'r-x','MarkerSize',4,'LineWidth',2)
	
	title(['Frame #' num2str(k) ' - XY trajectory']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	axis(axis_range);  
	view(2);
	grid on
	
    drawnow
    pause(show_delay)
	
end








