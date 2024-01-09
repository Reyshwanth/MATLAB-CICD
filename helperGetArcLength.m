function [arcLength, arcLengthRatio] = helperGetArcLength(laneStruct,laneID,point)
%HELPERGETARCLENGTH gets the distance of the point from the start of the
%lane.
%
% Outputs:
%
%       arcLength               Distance from start of the lane.
%
%       arcLengthRatio          Ratio of the arc length to the length of
%                               the lane.
%
% NOTE: The name of this helper function and it's functionality may change
% without notice in a future release, or the helper function itself may be
% removed.

% Copyright 2022 The MathWorks, Inc.

laneGeometry = laneStruct(laneID).Geometry;
% Flip geometry if lane direction is backward
if laneStruct(laneID).TravelDirection ==roadrunner.hdmap.TravelDirection.Backward
    laneGeometry = flipud(laneGeometry);
end
s = [0; cumsum(vecnorm(diff(laneGeometry),2,2))];

d = vecnorm(laneGeometry-point.Position,2,2);
[~,startIdx]= min(d);
% case1 - minimum is left index
%   x....o........x
% case2 - minimum is right index
%   x.........o...x....x
% Get left point index.
if startIdx == size(laneGeometry,1) || (d(startIdx)+d(startIdx+1)-(s(startIdx+1)-s(startIdx)))>0.01
    startIdx = startIdx -1;
end

% Calculate arc length
arcLength = s(startIdx)+d(startIdx);
% Calculate arc length ratio
arcLengthRatio = arcLength/s(end);
end

