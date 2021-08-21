function plotBoundingboxes(rect_p, rect_size, clr, lgd, frame_idx, axis_range)
	% plot 3d bounding boxes
	obj_num = size(rect_p,1);
	if isempty(clr)
		clr = rand(obj_num,3);
	end
	
	for m = 1:obj_num
		plotcube(rect_size(m,:), rect_p(m,:), 0.1, clr(m,:), [lgd '_' num2str(m)])
	end
	
	title(['Frame #' num2str(frame_idx) ' - 3D detection']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	axis(axis_range);
	view(3);
	grid on
end