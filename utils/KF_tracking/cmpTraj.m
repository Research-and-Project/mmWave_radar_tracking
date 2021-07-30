function cmpTraj(orig_traj, kf_traj, type,varargin)
% compare the KF corrected trajectory and original trajectory
if nargin<3
	type = 'plot';
end

ax = gca;
if nargin>3
	set(ax,varargin{:});
end

hold on

if strcmp(type,'scatter')
	if size(kf_traj,2) == 2
		scatter(kf_traj(:,1),kf_traj(:,2),'rx')
		scatter(orig_traj(:,1),orig_traj(:,2), 'go')
	elseif size(kf_traj,2) == 3
		scatter3(kf_traj(:,1),kf_traj(:,2),kf_traj(:,3),'rx')
		scatter3(orig_traj(:,1),orig_traj(:,2),orig_traj(:,3), 'go')		
		view(3)
	else
		error('data format wrong, 2D or 3D vector is needed')
	end
elseif strcmp(type,'plot')
	if size(kf_traj,2) == 2
		plot(kf_traj(:,1),kf_traj(:,2),'r-x','MarkerSize',5,'LineWidth',1)
		plot(orig_traj(:,1),orig_traj(:,2), 'g-o','MarkerSize',5,'LineWidth',1)
	elseif size(kf_traj,2) == 3
		plot3(kf_traj(:,1),kf_traj(:,2),kf_traj(:,3),'r-x','MarkerSize',2,'LineWidth',1)
		plot3(orig_traj(:,1),orig_traj(:,2),orig_traj(:,3), 'g-o','MarkerSize',2,'LineWidth',1)		
		view(3)
	else
		error('data format wrong, 2D or 3D vector is needed')
	end		
end

title('K.F. Traj v.s. Meas. Traj.')
xlabel('x'), ylabel('y'), zlabel('z')
legend('K.F. Traj.', 'Meas. Traj.')
hold off

end