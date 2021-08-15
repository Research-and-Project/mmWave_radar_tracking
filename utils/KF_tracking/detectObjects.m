function [centroids, bboxes, mask] = detectObjects(obj_detector, frame)
% The |detectObjects| function returns the centroids and the bounding boxes
% of the detected objects. It also returns the binary mask, which has the 
% same size as the input frame. Pixels with a value of 1 correspond to the
% foreground, and pixels with a value of 0 correspond to the background.   
%
% The function performs motion segmentation using the foreground detector. 
% It then performs morphological operations on the resulting binary mask to
% remove noisy pixels and to fill the holes in the remaining blobs. 

	% Detect foreground.
	mask = obj_detector.detector.step(frame);

	% Apply morphological operations to remove noise and fill in holes.
	mask = imopen(mask, strel('rectangle', [3,3]));
	mask = imclose(mask, strel('rectangle', [15, 15])); 
	mask = imfill(mask, 'holes');

	% Perform blob analysis to find connected components.
	[~, centroids, bboxes] = obj_detector.blobAnalyser.step(mask);
end