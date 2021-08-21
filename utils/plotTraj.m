function plotTraj(traj, axis_range, clr)
% plot trajectories
	if nargin < 3 || isempty(clr)
		clr = [1 0 0];
	end
	
	if size(traj, 2)==2
		plot(traj(:,1),traj(:,2),'-x','MarkerSize',5,'LineWidth',1, 'Color',clr)
	elseif size(traj, 2)==3
		plot3(traj(:,1),traj(:,2),traj(:,3),'-x','MarkerSize',5,'LineWidth',1, 'Color',clr)
	end
	
% 	title(['Frame #' num2str(frame_idx) ' - XY trajectory']);
	title('XY trajectory');
	xlabel('X'), ylabel('Y'), zlabel('Z');
	legend('Traj.')
	axis(axis_range);  
	view(2);
	grid on
end