%% Motion-Based Multiple Object Tracking
% This example shows how to perform automatic detection and motion-based
% tracking of moving objects in a video from a stationary camera.
%
% Modified (unfold) based on MATLAB build-in example
% MotionBasedMultiObjectTrackingExample.m

%% env init
clc, clear

%% param
% video path
video_path = './atrium.mp4';
% video_path = './singleball.mp4';

% Kalman filter
motion_type = 'ConstantVelocity'; % 'ConstantVelocity' | 'ConstantAcceleration'
param_kf = getDefaultKFParameters(motion_type);
param.initialEstimateError  = [200 50];
param.motionNoise           = [100 25];
param.measurementNoise      = 100;


%% tracking
% init
obj_video = setupVideoObjects(video_path);
obj_detector = setupDetectorObjects();
tracks = initializeTracks();
nextId = 1;
frame_idx = 1;

% Detect moving objects, and track them across video frames.
while ~isDone(obj_video.reader)
	% read a frame
    frame =  obj_video.reader.step();

	% detect the objects
    [centroids, bboxes, mask] = detectObjects(obj_detector,frame);
	
	% predict new locations of last location (for cost calculation)
    tracks = predictNewLocationsOfTracks(tracks);
	
	% determine assignment of detection to tracks
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment(tracks, centroids);
    
	% undate assigned tracks
    tracks = updateAssignedTracks(tracks, assignments, centroids, bboxes);
	
	% update unassigned tracks;
    tracks = updateUnassignedTracks(tracks, unassignedTracks);
	
	% update track states
    % tracks = deleteLostTracks(tracks);
	tracks = updateTrackStates(tracks);
	% create new tracks(tracks);
    [tracks,nextId] = createNewTracks(tracks, unassignedDetections, centroids, bboxes, param_kf, nextId, frame_idx);
    
	% display track results
    displayTrackingResults(obj_video, frame, mask, tracks);
	
	% update frame index
	frame_idx = frame_idx + 1;
	
% 	if frame_idx == 253
% 		pause
% 	end

% 	% [debug]
% 	if frame_idx==358
% 		pause
% 	end
end

