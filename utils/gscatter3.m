function gscatter3(x,y,z,group,clr,sym,siz,doleg,xnam,ynam,znam)
%GSCATTER3  3D Scatter plot with grouping variable
%   gscatter3(x,y,z,group,clr,sym,siz,doleg,xnam,ynam,znam)   
%   Designed to work in the exactly same fashion as statistics toolbox's gscatter
%   This function does not require the statistics toolbox installed.
%
%	Input:
%		x,y,z: 3d coordinates of points
%		group: the group index of the points
%		clr,sym,siz: plot color, symbol and size
%		doleg: show legend 'on' | 'off'
%		xnam,ynam,znam: axis label name
%
%   See also GSCATTER, GSCATTER3B
%
%   Copyright 2017 Gustavo Ferraz Trinade.


% Set number of groups 
cgroups = unique(group);

cmap = lines(size(cgroups,1));

% Input variables
if (nargin < 5) || isempty(clr),  clr = lines(max(size(cgroups))); end
if (nargin < 6) || isempty(sym), sym = 'odphs><v^odphs><v^odphs><v^odphs><v^odphs><v^odphs><v^'; end
if (nargin < 7),  siz = 10;           end
if (nargin < 8),  doleg = 'on';        end
if (nargin < 9),  xnam = 'X'; end
if (nargin < 10), ynam = 'Y'; end
if (nargin < 11), znam = 'Z'; end

% Get current axes
newplot;
a = gca;
hold(a,'on')

% Plot
for i=1:max(size(cgroups))    
    
    if iscell(cgroups) || ischar(cgroups)
        gi = find(strcmp(group,cgroups(i)));
    else
        gi = find(group == cgroups(i));
    end   
    
    hdl = scatter3(a,x(gi),y(gi),z(gi),siz,clr(i,:),'filled',sym(i)); 
	if strcmp(doleg,'off')
		hdl.Annotation.LegendInformation.IconDisplayStyle = 'off';
	end
end 

% Axes labels and legend (this bit slows down the function)
xlabel(a,xnam);
ylabel(a,ynam);    
zlabel(a,znam);    
view(3)

if strcmp(doleg,'on')
    if iscell(cgroups) || ischar(cgroups)
        legend(cgroups')
	else
        legend(num2str(cgroups(:)))
	end
end

hold off
end
