function [nodeTable, edgeTable] = helperGetNodesEdges(varargin)
%HELPERGETNODESEDGES gets the nodes and edges of scene by considering mid
%points of the lane center geometry as the node for the lane. It takes an
%optional input of lane structure to get nodes and edges. If this is not
%provided, lane structure is obtained from HD map.
%     
% NOTE: The name of this helper function and it's functionality may change
% without notice in a future release, or the helper function itself may be
% removed.

% Copyright 2022-2023 The MathWorks, Inc.

% Parse input to get the lane structure.
% Single input - lane struct.
% Non-single input - get the lane struct by reading the map.
if nargin == 1
    lanesStruct = varargin{1};
else
    % Get the ScenarioSimulation object.
    scenarioSimulationObj = Simulink.ScenarioSimulation.find("ScenarioSimulation");

    % Get map information from RoadRunner
    map = scenarioSimulationObj.getMap;

    % Get codegen map
    cgMap = matlabshared.roadrunner.codegenHDMap(map.map);

    % Update lane struct to split bidirectional lane
    lanesStruct = helperSplitBidirectionalLane(cgMap.Lanes);
end

numLanes = length(lanesStruct);

laneMid = zeros(numLanes,3);
for i = 1:numLanes
    numGeometry = size(lanesStruct(i).Geometry,1);
    laneMid(i,:) = lanesStruct(i).Geometry(round(numGeometry/2),:);
end
nodeTable = table(laneMid, lanesStruct, 'VariableNames', {'StateVector', 'LanesStruct'});

% Create edges of the graph
endNodes = double.empty(0,2);
edgeType = {};

for i = 1:numLanes
    % Get predecessors and successors
    if lanesStruct(i).TravelDirection==roadrunner.hdmap.TravelDirection.Forward
        predecessors = lanesStruct(i).Predecessors;
        successors = lanesStruct(i).Successors;
    else
        predecessors = lanesStruct(i).Successors;
        successors = lanesStruct(i).Predecessors;
    end

    % Predecessors edges
    [endNodes, edgeType] = getPredecessorSuccessorEdges(endNodes, edgeType, lanesStruct,i,predecessors,'Predecessor');

    % Successor edges
    [endNodes, edgeType] = getPredecessorSuccessorEdges(endNodes, edgeType, lanesStruct,i,successors,'Successor');

    % Adjacency Lanes
    leftLaneBoundaries = [lanesStruct.LeftLaneBoundary];
    rightLaneBoundaries = [lanesStruct.RightLaneBoundary];

    % Identify left lane
    leftLaneIdx = find(strcmp({rightLaneBoundaries.ID}, lanesStruct(i).LeftLaneBoundary.ID));
    %Identify right lane
    rightLaneIdx = find(strcmp({leftLaneBoundaries.ID}, lanesStruct(i).RightLaneBoundary.ID));
    adjacentLaneIdx = [leftLaneIdx rightLaneIdx];
    if ~isempty(adjacentLaneIdx)
        for j = adjacentLaneIdx
            if lanesStruct(j).LaneType==roadrunner.hdmap.LaneType.Driving
                if lanesStruct(i).TravelDirection==lanesStruct(j).TravelDirection
                    endNodes(end+1,1:2) = [i, j];
                    edgeType{end+1,1} = 'Adjacent';
                end
            end
        end
    end
end

% Remove duplicate rows
[endNodes, ind] = unique(endNodes, 'rows');
edgeType = {edgeType{ind}}';

% Create EdgeTable
edgeTable = table(endNodes, edgeType, 'VariableNames', {'EndStates', 'EdgeType'});
end

function [endNodes, edgeType] = getPredecessorSuccessorEdges(endNodes, edgeType, lanesStruct,laneID,edgeConnections,edgeConnectionType)
    if ~isempty(edgeConnections)
        for j = 1:length(edgeConnections)
            edgeConnectionsIdx = find(strcmp({lanesStruct.ID}, edgeConnections(j).ID));
            if length(edgeConnectionsIdx)==1
                if strcmp(edgeConnectionType,'Predecessor')
                    endNodes(end+1,1:2) = [edgeConnectionsIdx, laneID];
                    edgeType{end+1,1} = 'Predecessor';
                else
                    endNodes(end+1,1:2) = [laneID, edgeConnectionsIdx];
                    edgeType{end+1,1} = 'Successor';
                end
            else
                for k=1:length(edgeConnectionsIdx)
                    if lanesStruct(laneID).TravelDirection==lanesStruct(edgeConnectionsIdx(k)).TravelDirection
                        if strcmp(edgeConnectionType,'Predecessor')
                            endNodes(end+1,1:2) = [edgeConnectionsIdx(k), laneID];
                            edgeType{end+1,1} = 'Predecessor';
                        else
                            endNodes(end+1,1:2) = [laneID, edgeConnectionsIdx(k)];
                            edgeType{end+1,1} = 'Successor';
                        end
                    end
                end
            end
        end
    end
end
