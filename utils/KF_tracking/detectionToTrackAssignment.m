function [assignments, unassignedTracks, unassignedDetections] = ...
		detectionToTrackAssignment(tracks, centroids)
% Assigning object detections in the current frame to existing tracks is
% done by minimizing cost. The cost is defined as the negative
% log-likelihood of a detection corresponding to a track.  
%
% The algorithm involves two steps: 
%
% Step 1: Compute the cost of assigning every detection to each track using
% the |distance| method of the |vision.KalmanFilter| System object(TM). The 
% cost takes into account the Euclidean distance between the predicted
% centroid of the track and the centroid of the detection. It also includes
% the confidence of the prediction, which is maintained by the Kalman
% filter. The results are stored in an MxN matrix, where M is the number of
% tracks, and N is the number of detections.   
%
% Step 2: Solve the assignment problem represented by the cost matrix using
% the |assignDetectionsToTracks| function. The function takes the cost 
% matrix and the cost of not assigning any detections to a track.  
%
% The value for the cost of not assigning a detection to a track depends on
% the range of values returned by the |distance| method of the 
% |vision.KalmanFilter|. This value must be tuned experimentally. Setting 
% it too low increases the likelihood of creating a new track, and may
% result in track fragmentation. Setting it too high may result in a single 
% track corresponding to a series of separate moving objects.   
%
% The |assignDetectionsToTracks| function uses the Munkres' version of the
% Hungarian algorithm to compute an assignment which minimizes the total
% cost. It returns an M x 2 matrix containing the corresponding indices of
% assigned tracks and detections in its two columns. It also returns the
% indices of tracks and detections that remained unassigned. 
	normalIdIdx = find(strcmp([tracks(:).state],"normal"));
	nTracks = length(normalIdIdx);
	nDetections = size(centroids, 1);

	% Compute the cost of assigning each detection to each track.
	cost = zeros(nTracks, nDetections);
	for i = 1:nTracks
		cost(i, :) = distance(tracks(normalIdIdx(i)).kalmanFilter, centroids);
	end

	% Solve the assignment problem.
	costOfNonAssignment = 20;
	[assignments, unassignedTracks, unassignedDetections] = ...
		assignDetectionsToTracks(cost, costOfNonAssignment);
end