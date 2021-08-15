function tracks = updateUnassignedTracks(tracks, unassignedTracks)
% Mark each unassigned track as invisible, and increase its age by 1.

normalId = strcmp([tracks(:).state],"normal");
normalId = find(normalId);
	for i = 1:length(unassignedTracks)
		ind = normalId(unassignedTracks(i));
		tracks(ind).age = tracks(ind).age + 1;
		tracks(ind).consecutiveInvisibleCount = ...
			tracks(ind).consecutiveInvisibleCount + 1;
	end
end