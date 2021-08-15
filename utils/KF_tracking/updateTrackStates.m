function tracks = updateTrackStates(tracks)
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. It also deletes recently created tracks
% that have been invisible for too many frames overall. 
	if isempty(tracks)
		return;
	end

	invisibleForTooLong = 20;
	ageThreshold = 8;

	% normal tracks
	normalId = strcmp([tracks(:).state],"normal");
	normal_tracks = tracks(normalId);
	
	% Compute the fraction of the track's age for which it was visible.
% 	normalId = strcmp([tracks(:).state],"normal");
	ages = [normal_tracks().age];
	totalVisibleCounts = [normal_tracks().totalVisibleCount];
	visibility = totalVisibleCounts ./ ages;

	% update tracks states (noise,lost,normal).
	noiseInds = ages < ageThreshold & visibility < 0.6;
	if any(noiseInds)
		[normal_tracks(noiseInds).state] = cellstr(repmat("noise",sum(noiseInds),1));
	end
% 	normalId = strcmp([tracks(:).state],"normal");
	lostInds = [normal_tracks().consecutiveInvisibleCount] >= invisibleForTooLong;
% 	normalId = find(normalId);
	if any(lostInds)
		[normal_tracks(lostInds).state] = cellstr(repmat("lost",sum(lostInds),1));
	end
	
	% remap from normal_tracks to tracks
	tracks(normalId) = normal_tracks;
% 	% Find the indices of 'lost' tracks.
% 	lostInds = (ages < ageThreshold & visibility < 0.6) | ...
% 		[tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
% 
% 	% Delete lost tracks.
% 	tracks = tracks(~lostInds);
end