%% Tracking based on Kalman Filter for millimeter radar data
% data format
% *data format:* 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
% 8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS

%% env init
addpath(genpath('./utils'));

%% param
% path
traj_path = './result/traj.mat';

% KF paramss
motion_type = 'ConstantVelocity'; % 'ConstantVelocity' | 'ConstantAcceleration'
param = getDefaultKFParameters(motion_type);
% param.initialEstimateError  = 1E5 * ones(1, 2);
% param.motionNoise           = [25, 10];
% param.measurementNoise      = 25;	

%% load data
traj = load(traj_path);
traj = traj.traj;
% scatter(traj(:,1),traj(:,2)); % show trajectory


	
%% KF tracking
% init
% detectedLocation = [];  % The detected location
% trackedLocation  = [];  % The tracked location
kf_traj = [];			  % The KF corrected tracked trajectory
label            = '';  % Label for the ball
isTrackInitialized = false;
isObjectDetected = true;
frame_idx = 1;
frame_num = size(traj,1);

while ~isFinish(frame_idx, frame_num)

    % Detect the obj.
%     [detectedLocation, isObjectDetected] = detectObject(frame);
	detectedLocation = traj(frame_idx,:); % 3d loc
% 	detectedLocation = traj(frame_idx,1:2); % 2d loc
	if isnan(detectedLocation)
		isObjectDetected = false;
	else
		isObjectDetected = true;
	end
	
    if ~isTrackInitialized
      if isObjectDetected
		% Initialize a track by creating a Kalman filter when the obj detected for the first time.
        initialLocation = computeInitialLocation(param, detectedLocation);
        kalmanFilter = configureKalmanFilter(param.motionModel, ...
          initialLocation, param.initialEstimateError, ...
          param.motionNoise, param.measurementNoise);

        isTrackInitialized = true;
        trackedLocation = correct(kalmanFilter, detectedLocation);
        label = 'Initial';
      else
        trackedLocation = [];
        label = '';
      end

    else
      % Use the Kalman filter to track the ball.
      if isObjectDetected % The ball was detected.
        % Reduce the measurement noise by calling predict followed by
        % correct.
        predict(kalmanFilter);
        trackedLocation = correct(kalmanFilter, detectedLocation);
        label = 'Corrected';
      else % The ball was missing.
        % Predict the ball's location.
        trackedLocation = predict(kalmanFilter);
        label = 'Predicted';
      end
	end

	frame_idx = frame_idx + 1;
    kf_traj = updateTraj(kf_traj, trackedLocation);
% 	annotateTrackedObject();
end

% show results
cmpTraj(kf_traj, traj);

return 

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

% For illustration purposes, select the initial location used by the Kalman
% filter.
function loc = computeInitialLocation(param, detectedLocation)
  if strcmp(param.initialLocation, 'Same as first detection')
    loc = detectedLocation;
  else
    loc = param.initialLocation;
  end
end

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

% compare the KF corrected trajectory and original trajectory
function cmpTraj(kf_traj, orig_traj)
	figure
	hold on
	scatter(kf_traj(:,1),kf_traj(:,2),'bx')
	scatter(orig_traj(:,1),orig_traj(:,2),'filled', 'go')
	title('KF traj v.s. orig traj')
	legend('KF traj','orig traj')
end