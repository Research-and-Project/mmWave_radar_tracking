function obj_features = calc_feature(frame_obj)
% calculate features of objects
% Input
%	- frame_obj: frame points belong to a certain object
% 
% Output
%	- obj_features	features of the detected objects, N*1 struct
%		- average_speed: average speed of all the object points
%		- average_power: average power of all the object points

% calc feature
obj_features(kk).average_speed = mean(frame_obj(:,7));
obj_features(kk).average_power = mean(frame_obj(:,8));
	
end