function [centroids, bboxes, obj_frame, obj_idx] = getDetections(frame, param_det)
% GETDETECTIONS	-	get detections of the objects
% Input
%	- frame			data of current frame, numerical matrix
%	- param_det		detection parameters, struct

% 
% Output
%	- centroids		centroids of the detections, N*2 matrix
%	- bboxes		bounding boxes of the detections, N*4 matrix
%	- obj_frame		object points in the frame
%	- obj_idx		idx of object points in the frame
% 
% Note
%	- mmWave radar data format: 1-X, 2-Y, 3-Z, 4-RANGE, 5-AZIMUTH, 6-ELEVATION, 7-DOPPLER, 
%	  8-POWER, 9-POWER_VALUE, 10-TIMESTAMP_MS


centroids = [];
bboxes = [];
% obj_features = [];

if size(frame, 1)  < param_det.minObjPoints
	% too few points - no detection
	obj_frame = [];
	obj_idx = [];
	return
end

% clound points clustering with X,Y coordinates as features
idx = DBSCAN(frame(:,[1,2]),param_det.DBSCAN_epsilon,param_det.DBSCAN_MinPts); % DBSCAN Cluster

% delete noise points cluster(idx==0)
obj_frame = frame;
obj_idx = idx;
obj_frame(idx==0,:) = []; 
obj_idx(idx==0,:) = [];

% calc bounding box
unique_class = unique(obj_idx);

for kk = 1:length(unique_class)
	frame_obj = obj_frame(obj_idx==unique_class(kk),:);
	
	% calc bounding box
	rect_min = min(frame_obj(:,1:3),[],1);
	rect_max = max(frame_obj(:,1:3),[],1);
	rect_size = rect_max - rect_min;
	% rect_center = (rect_min + rect_max)/2;
% 	rect_center = calcCentroid(frame_obj(:,1:3));
	rect_center = calcCentroid(frame_obj(:,1:3),frame_obj(:,8));
	
	bboxes(kk,1:3) = rect_min;
	bboxes(kk,4:6) = rect_size;
	centroids(kk,1:3) = rect_center;
	

end

end




%% sub functions
%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPML110
% Project Title: Implementation of DBSCAN Clustering in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function [IDX, isnoise]=DBSCAN(X,epsilon,MinPts)

    C=0;
    
    n=size(X,1);
    IDX=zeros(n,1);
    
    D=pdist2(X,X);
    
    visited=false(n,1);
    isnoise=false(n,1);
    
    for i=1:n
        if ~visited(i)
            visited(i)=true;
            
            Neighbors=RegionQuery(i);
            if numel(Neighbors)<MinPts
                % X(i,:) is NOISE
                isnoise(i)=true;
            else
                C=C+1;
                ExpandCluster(i,Neighbors,C);
            end
            
        end
    
    end
    
    function ExpandCluster(i,Neighbors,C)
        IDX(i)=C;
        
        k = 1;
        while true
            j = Neighbors(k);
            
            if ~visited(j)
                visited(j)=true;
                Neighbors2=RegionQuery(j);
                if numel(Neighbors2)>=MinPts
                    Neighbors=[Neighbors Neighbors2];   %#ok
                end
            end
            if IDX(j)==0
                IDX(j)=C;
            end
            
            k = k + 1;
            if k > numel(Neighbors)
                break;
            end
        end
    end
    
    function Neighbors=RegionQuery(i)
        Neighbors=find(D(i,:)<=epsilon);
    end

end
