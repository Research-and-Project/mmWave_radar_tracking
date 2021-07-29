%% signal filter for millimeter radar data
% remove noise and static points

% data format
% *data format:* 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
% 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS

%% init
addpath(genpath('./utils'));


%% param
% path and setting
data_dir = './data/weifu_zyk_radar_data/';
data_item = '单人8字2pcd/';
% data_item = '单人横穿pcd/';
data_num_max = 750;
% show
% view_vec = [0 0 1];
view_vec = [1 1 1];
axis_range = [-5, 5, 0, 15, -5, 5];

% file info
datas = dir([data_dir data_item '*.txt']);
data_names = {datas.name};
data_num = length(data_names);
data_num_max = min(data_num, data_num_max);

%% doppler analysis
doppler_threshold = 0.4;

figure
% hold on
for k = 1:data_num_max
    frame=importdata([data_dir data_item data_names{k}]);
    frame_doppler = frame(abs(frame(:,7))>doppler_threshold,:); % doppler items
    disp(['points num: ' num2str(size(frame_doppler,1))])
	
	if size(frame_doppler, 1) == 0
		continue
	end
	
    scatter3(frame_doppler(:,1),frame_doppler(:,2),frame_doppler(:,3),'r.');
	
	% bounding box
	rect_min = min(frame_doppler(:,1:3),[],1);
	rect_max = max(frame_doppler(:,1:3),[],1);
	rect_size = rect_max - rect_min;
	
	plotcube(rect_size, rect_min, 0.1, [0 0 0])
	
	title(['Frame #' num2str(k)]);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	axis(axis_range);
	view(view_vec);
	grid on
	
    drawnow
%     pause(0.5)
end
