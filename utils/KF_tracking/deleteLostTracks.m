function tracks = deleteLostTracks(tracks)
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. It also deletes recently created tracks
% that have been invisible for too many frames overall. 
	if isempty(tracks)
		return;
	end

	invisibleForTooLong = 20;
	ageThreshold = 8;

	% Compute the fraction of the track's age for which it was visible.
	ages = [tracks(:).age];
	totalVisibleCounts = [tracks(:).totalVisibleCount];
	visibility = totalVisibleCounts ./ ages;

	% Find the indices of 'lost' tracks.
	lostInds = (ages < ageThreshold & visibility < 0.6) | ...
		[tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

	% Delete lost tracks.
	tracks = tracks(~lostInds);
end