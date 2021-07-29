function [sorted_idx, sorted_Dg] = cluster_idx_arranege(data, idx)
% CLUSTER_IDX_ARRANGE	calculate the divergence in each cluster and re-arrange 
% the group index with the ascent order of divergence
% divergence metric: volume or area of the bounding box (square)
% input: 
%	data - data points
%	idx  - group index of data points
% output:
%	sorted_Dg  - cluster_divergence
%	sorted_idx - rearranged idx
% Zhihong Zhang, 20210728
% 

cidx = sort(unique(idx));
Dg = inf(numel(cidx),1);

for i=1:numel(cidx)  
	datai = data(idx == cidx(i),:);
	
	rect_min = min(datai,[],1);
	rect_max = max(datai,[],1);
	rect_size = rect_max - rect_min;
	Dg(i) = prod(rect_size);
end

[sorted_Dg, sorted_order] = sort(Dg);
sorted_idx = zeros(size(idx));

for i=1:numel(sorted_order) 
	sorted_idx(idx==cidx(sorted_order(i))) = cidx(i);
end

end