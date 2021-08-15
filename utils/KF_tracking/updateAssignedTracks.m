function tracks = updateAssignedTracks(tracks, assignments, centroids, bboxes)
% The |updateAssignedTracks| function updates each assigned track with the
% corresponding detection. It calls the |correct| method of
% |vision.KalmanFilter| to correct the location estimate. Next, it stores
% the new bounding box, and increases the age of the track and the total
% visible count by 1. Finally, the function sets the invisible count to 0. 
	normalId = strcmp([tracks(:).state],"normal");
	normalId = find(normalId);
	
	numAssignedTracks = size(assignments, 1);
	for i = 1:numAssignedTracks
		trackIdx = normalId(assignments(i, 1));
		detectionIdx = assignments(i, 2);
		centroid = centroids(detectionIdx, :);
		bbox = bboxes(detectionIdx, :);

		% Correct the estimate of the object's location
		% using the new detection. 
		kf_centroid = correct(tracks(trackIdx).kalmanFilter, centroid);

		% Replace predicted bounding box with detected
		% bounding box.
		tracks(trackIdx).bbox = bbox;
		
		% update record for trajectory and bbox
		tracks(trackIdx).traj_rec(end+1,:) = kf_centroid; 
		tracks(trackIdx).bbox_rec(end+1,:) = bbox; 

		% Update track's age.
		tracks(trackIdx).age = tracks(trackIdx).age + 1;

		% Update visibility.
		tracks(trackIdx).totalVisibleCount = ...
			tracks(trackIdx).totalVisibleCount + 1;
		tracks(trackIdx).consecutiveInvisibleCount = 0;
	end
end
