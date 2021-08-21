function tracks = updateTrackStates(tracks)
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. It also deletes recently created tracks
% that have been invisible for too many frames overall. 
	if isempty(tracks)
		return;
	end

	invisibleForTooLong = 20;
	ageThreshold = 5;

	% normal tracks
	normalId = strcmp([tracks(:).state],"normal");
	normal_tracks = tracks(normalId);
	
	% Compute the fraction of the track's age for which it was visible.
% 	normalId = strcmp([tracks(:).state],"normal");
	ages = [normal_tracks().age];
	totalVisibleCounts = [normal_tracks().totalVisibleCount];
	visibility = totalVisibleCounts ./ ages;

	% update tracks states (noise,lost,normal).
	noiseInds = ages < ageThreshold & visibility < 0.5;
	for mm=find(noiseInds)
		normal_tracks(mm).state = "noise";
	end
% 	normalId = strcmp([tracks(:).state],"normal");
	lostInds = [normal_tracks().consecutiveInvisibleCount] >= invisibleForTooLong;
% 	normalId = find(normalId);
	for mm=find(lostInds)
		normal_tracks(mm).state = "lost";
	end
	
	% remap from normal_tracks to tracks
	tracks(normalId) = normal_tracks;

end