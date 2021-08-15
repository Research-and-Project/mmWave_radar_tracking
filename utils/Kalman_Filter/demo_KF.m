 %% Tracking based on Kalman Filter for millimeter radar data

%% env init
addpath(genpath('./utils'));

%% param
% path
% traj_path = './orig_traj.mat';
traj_path = './orig_traj_withNaN.mat';

% KF paramss
motion_type = 'ConstantVelocity'; % 'ConstantVelocity' | 'ConstantAcceleration'
param = getDefaultKFParameters(motion_type);
% param.initialEstimateError  = 1E5 * ones(1, 2);
% param.motionNoise           = [25, 10];
% param.measurementNoise      = 25;	

%% load data
meas_traj = load(traj_path);
meas_traj = meas_traj.traj;
% scatter(traj(:,1),traj(:,2)); % show trajectory


	
%% KF tracking
% init
KF = []; % KF handle
kf_traj = [];			  % The KF corrected tracked trajectory
isTrackInitialized = false;
isObjectDetected = true;
frame_idx = 1;
frame_num = size(meas_traj,1);

while ~isFinish(frame_idx, frame_num)

    % Detection
% 	detectedLocation = traj(frame_idx,1:3); % 2d loc
	detectedLocation = meas_traj(frame_idx,1:2); % 2d loc
	
	% Kalman filter
	[trackedLocation, KF, states] = KF_step(detectedLocation, KF, param);

	% update
	frame_idx = frame_idx + 1;
    kf_traj = updateTraj(kf_traj, trackedLocation);
end

% show results
figure
cmpTraj(meas_traj, kf_traj, 'scatter');

return 

%% sub functions
% judge whether finish all the frame
function finish_flag = isFinish(frame_idx, frame_num)
	if frame_idx > frame_num
		finish_flag = true;
	else
		finish_flag = false;
	end
end


% update KF corrected trajectory
function kf_traj = updateTraj(kf_traj, kf_loc)
	kf_traj(end+1,:) = kf_loc;
end

