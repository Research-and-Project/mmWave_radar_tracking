function [kf_loc, new_KF, states] = KF_step(meas_loc, cur_KF, param)
% KF_TRACKING - Tracking based on Kalman Filter
% Input:
%	- param		KF parameters, struct
%	- cur_KF	current Kalman Filter handle
%	- meas_loc	current measured location, 2D/3D vector
%
% Output:
%	- kf_loc		KF predicted/corrected location, 2D/3D vector
%	- new_KF		updated  Kalman Filter handle
%	- states		output status 'Init' | 'Not Init' |'Predicted' | 'Corrected'
% Zhihong Zhang, 2021-7-30

%% param
% default KF paramss
if nargin<3
	motion_type = 'ConstantVelocity'; % 'ConstantVelocity' | 'ConstantAcceleration'
	param = getDefaultKFParameters(motion_type);
end

if nargin<2 || isempty(cur_KF)
	isTrackInitialized = false;
else
	isTrackInitialized = true;
end

if any(isnan(meas_loc))
	meas_loc = [];
end
	
new_KF = [];
kf_loc = [];

%% KF tracking
if ~isTrackInitialized
	if ~isempty(meas_loc)
		% Initialize a track by creating a Kalman filter when the obj detected for the first time.
		initialLocation = computeInitialLocation(param, meas_loc);
		new_KF = configureKalmanFilter(param.motionModel, ...
		  initialLocation, param.initialEstimateError, ...
		  param.motionNoise, param.measurementNoise);

		kf_loc = correct(new_KF, meas_loc);
		states.KF_action = 'Init';
	else
		states.KF_action = 'Not Init';
	end
	
else
  % Use the Kalman filter to track the object.
  if ~isempty(meas_loc) % The obj was detected.
	% Reduce the measurement noise by calling predict followed by
	% correct.
	predict(cur_KF);
	kf_loc = correct(cur_KF, meas_loc);
	states.KF_action = 'Corrected';
  else % The obj was missing.
	% Predict the ball's location.
	kf_loc = predict(cur_KF);
	states.KF_action = 'Predicted';
  end
  new_KF = cur_KF; % update KF
end


end



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
  if strcmp(param.initialLocation, 'Same as first detection') || isempty(param.initialLocation)
    loc = detectedLocation;
  else
    loc = param.initialLocation;
  end
end



