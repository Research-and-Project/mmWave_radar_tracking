function centroid = calcCentroid(coords, weights)
% calculate centroid with given coordinates and weights

% default weights  = [ 1,1,1,...,1], i.e. average
if nargin<2
	weights = ones(size(coords,1),1);
end

norm_weights = weights./sum(weights);
centroid  = sum(coords.*repmat(norm_weights(:),1,size(coords,2)),1);

end