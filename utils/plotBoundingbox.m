function plotBoundingbox(rect_p, rect_size, clr, lgd, axis_range)
	% plot 3d bounding box
	if isempty(clr)
		clr = [1 0 0];
	end

	hdl = plotcube(rect_size, rect_p, 0.3, clr);
	hdl.DisplayName = lgd;
	
% 	title(['Frame #' num2str(frame_idx) ' - 3D detection']);
	title(['3D detection']);
	xlabel('X'), ylabel('Y'), zlabel('Z');
	axis(axis_range);
	view(3);
	grid on
end