function tracks = initializeTracks()
% The |initializeTracks| function creates an array of tracks, where each
% track is a structure representing a moving object in the video. The
% purpose of the structure is to maintain the state of a tracked object.
% The state consists of information used for detection to track assignment,
% track termination, and display. 
%
% The structure contains the following fields:
%
% * |id| :                  the integer ID of the track
% * |bbox| :                the current bounding box of the object; used
%                           for display
% * |traj_rec| :			the trajectory record of the object
% * |bbox_rec| :			the bounding box record of the object
% * |kalmanFilter| :        a Kalman filter object used for motion-based
%                           tracking
% * |age| :                 the number of frames since the track was first
%                           detected
% * |appear_frame| :        the frame index that the track was first detected
%             
% * |state| :				track state, "noise" | "lost" | "normal"
% * |totalVisibleCount| :   the total number of frames in which the track
%                           was detected (visible)
% * |consecutiveInvisibleCount| : the number of consecutive frames for 
%                                  which the track was not detected (invisible).
%
% Noisy detections tend to result in short-lived tracks. For this reason,
% the example only displays an object after it was tracked for some number
% of frames. This happens when |totalVisibleCount| exceeds a specified 
% threshold.    
%
% When no detections are associated with a track for several consecutive
% frames, the example assumes that the object has left the field of view 
% and deletes the track. This happens when |consecutiveInvisibleCount|
% exceeds a specified threshold. A track may also get deleted as noise if 
% it was tracked for a short time, and marked invisible for most of the 
% frames.

	tracks = struct(...
		'id', {}, ...
		'bbox', {}, ...
		'traj_rec', {}, ...
		'bbox_rec', {}, ...
		'kalmanFilter', {}, ...
		'appear_frame', {}, ...
		'age', {}, ...
		'state', {}, ...
		'totalVisibleCount', {}, ...
		'consecutiveInvisibleCount', {});
end


