function updatedLaneStruct = helperSplitBidirectionalLane(laneStruct)
%HELPERSPLITBIDIRECTIONALLANE updated lane structure by splitting the
%bidirectional lane into forward edge and backward edge.
%     
% NOTE: The name of this helper function and it's functionality may change
% without notice in a future release, or the helper function itself may be
% removed.

% Copyright 2022-2023 The MathWorks, Inc.

% Initialize output with input
updatedLaneStruct = laneStruct;

numLanes = length(laneStruct);
% Get the number of edges in the scene.
numEdges = 0;
for i = 1:numLanes
    numEdges = numEdges + 1;
    if laneStruct(i).TravelDirection == roadrunner.hdmap.TravelDirection.Bidirectional
        numEdges = numEdges + 1;
    end
end

% Assign the lanes information
edgeId = 1;
for i = 1:numLanes
    lane = laneStruct(i);
    updatedLaneStruct(edgeId)=lane;
    travelDir = lane.TravelDirection;
    if travelDir == roadrunner.hdmap.TravelDirection.Bidirectional
        % Update the lane structure with lane information in forward
        % direction.
        updatedLaneStruct(edgeId).TravelDirection=roadrunner.hdmap.TravelDirection.Forward;
        edgeId = edgeId+1;
        % Update the lane structure with lane information in backward
        % direction.
        updatedLaneStruct(edgeId)=lane;
        updatedLaneStruct(edgeId).TravelDirection=roadrunner.hdmap.TravelDirection.Backward;
    end
    edgeId = edgeId+1;
end

end

