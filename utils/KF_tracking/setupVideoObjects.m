function obj = setupVideoObjects(video_path)
% Create Video objects used for reading the video frames and displaying results.
	% Initialize Video I/O
	% Create objects for reading a video from a file, drawing the tracked
	% objects in each frame, and playing the video.

	% Create a video file reader.
	obj.reader = vision.VideoFileReader(video_path);
	% Create two video players, one to display the video,
	% and one to display the foreground mask.
	obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);
	obj.videoPlayer = vision.VideoPlayer('Position', [20, 400, 700, 400]);
end