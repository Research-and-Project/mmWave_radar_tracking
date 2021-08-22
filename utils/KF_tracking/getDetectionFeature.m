function obj_feature = getDetectionFeature(frame_obj)
% calculate features of detected objects
% Input
%	- frame_obj: frame points of a certain object
% 
% Output
%	- obj_feature	features of the detected objects, N*1 struct
%		- average_speed: average speed of all the object points
%		- average_power: average power of all the object points


obj_feature.average_speed = mean(frame_obj(:,7));  % aver. speed
obj_feature.average_power = mean(frame_obj(:,8));  % aver. power

end