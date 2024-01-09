classdef HelperPlanPath
    %HELPERPLANPATH is used to plan the reference path of the vehicle.
    %
    % NOTE: The name of this class and it's functionality may change
    % without notice in a future release, or the class itself may be
    % removed.

    % Copyright 2022-2023 The MathWorks, Inc.

    methods (Static)
        function [refPath, refPathLaneIDs, refPathRange] = planPath(planner, graph, egoStartPosition, egoGoalPosition, nvp)
            % PLANPATH plans the reference path of the vehicle by taking
            % ego start and goal positions as an input.
            %
            %   Inputs:
            %
            %   planner                 A* planner object to plan the
            %                           trajectory.
            %
            %   graph                   navGraph object.
            %
            %   egoStartPosition        Starting position of the ego
            %                           vehicle.
            %
            %   egoGoalPosition         Goal position of the ego vehicle.
            %
            %   Name-Value pair:
            %
            %   QueryAPI                QueryAPI object. If not provided,
            %                           QueryAPI object will be created.
            %
            %   Map                     Codegen map of the scene. If not
            %                           provided, map object will be
            %                           obtained using ScenarioSimulation
            %                           object.
            %
            %   PlotScene               Flag to plot the scene. If true,
            %                           creates a figure window by plotting
            %                           the lane geometry of the scene.
            %
            %   UseCustomWeightFunction Flag to update weights of the
            %                           navGraph to the weights from
            %                           queryAPI. If true, it will consider
            %                           the cost function provided defined
            %                           in the input graph.
            %
            %   LaneChangeDistance      Distance in which lane change needs
            %                           to be completed.
            %
            %   PermissibleDistance     Permissible distance of starting
            %                           point or goal point from the lane.
            %
            %   Outputs:
            %       
            %   refPath                 Reference path planned from
            %                           egoStartPosition to
            %                           egoGoalPosition.
            %
            %   refPathLaneIDs          Lane IDs along the reference path.
            %
            %   refPathRange            For all the lanes in
            %                           refPathLaneIDs, refPathRange stores
            %                           distance to be travelled along the
            %                           refPath until the end of the lane.

            %% Inputs
            arguments
               planner
               graph
               egoStartPosition double
               egoGoalPosition double
               nvp.QueryAPI = []
               nvp.Map = []
               nvp.PlotScene logical = true
               nvp.UseCustomWeightFunction logical = false
               nvp.LaneChangeDistance double {mustBeGreaterThan(nvp.LaneChangeDistance,5)} = 20
               nvp.PermissibleDistance double = 5
            end

            % Get the ScenarioSimulation object.
            scenarioSimulationObj = Simulink.ScenarioSimulation.find("ScenarioSimulation");

            if isempty(nvp.Map)
                % Get map information from RoadRunner
                map = scenarioSimulationObj.getMap;

                % Get codegen map
                cgMap = matlabshared.roadrunner.codegenHDMap(map.map);
            else
                cgMap = nvp.Map;
            end

            % Get Query API
            if ~isempty(nvp.QueryAPI)
                queryAPI = nvp.QueryAPI;
            else
                queryAPI = roadrunner.internal.roadrunnerHDMapQuery(cgMap);
            end

            % Flag to consider user defined weight table for graph
            if ~nvp.UseCustomWeightFunction
                % Get weight matrix
                adjMat = getLaneAdjacency(queryAPI);
                % Create new navGraph object by updating LinkWeightFcn.
                graph = navGraph(graph.States,graph.Links,LinkWeightFcn=@(state1, state2, graph) HelperPlanPath.weightFunction(state1,state2,adjMat));
                % Create planner object with updated navGraph object.
                planner = plannerAStar(graph);
            end

            % Get lane change parameters
            laneShiftNorm = 0.1;
            laneChangeDistance = nvp.LaneChangeDistance;

            % Update lane struct to split bidirectional lane
            lanesStruct = helperSplitBidirectionalLane(cgMap.Lanes);

            %% Plan the path
            vehicleStart = struct('Position', egoStartPosition, 'Yaw', 0, 'Roll', 0, 'Pitch', 0);
            % Ego starting point lane.
            egoStartLane = getCurrentLane(queryAPI,vehicleStart);
            % Get proper pose information of the point on the lane.
            vehicleStart = getClosestPointOnLane(queryAPI,vehicleStart,egoStartLane);

            vehicleGoal = struct('Position', egoGoalPosition, 'Yaw', 0, 'Roll', 0, 'Pitch', 0);
            % Ego goal point lane.
            egoGoalLane = getCurrentLane(queryAPI,vehicleGoal);
            % Get proper pose information of the point on the lane.
            vehicleGoal = getClosestPointOnLane(queryAPI,vehicleGoal,egoGoalLane);
            
            % As egoStartPosition or goal position may not be on the lane,
            % check if it is within the permissible distance from the lane. 
            % Default permissible distance = 5
            permissibleDistance = nvp.PermissibleDistance;
            % Check start position
            egoDistance = norm(egoStartPosition-vehicleStart.Position);
            if egoDistance > permissibleDistance
                error("Unable to find path with the specified start and goal positions");
            end
            % Check end position
            goalDistance = norm(egoGoalPosition-vehicleGoal.Position);
            if goalDistance > permissibleDistance
                error("Unable to find path with the specified start and goal positions");
            end

            % Arc length of the starting point in starting point lane.
            egoStartDistance = helperGetArcLength(lanesStruct,egoStartLane,vehicleStart);
            % Arc length of the goal point in its lane.
            egoGoalDistance = helperGetArcLength(lanesStruct,egoGoalLane,vehicleGoal);

            % If both start and goal lane matches
            if egoStartLane==egoGoalLane
                % If ego goal point is ahead of starting point
                if egoStartDistance < egoGoalDistance
                    laneGeometry = lanesStruct(egoStartLane).Geometry;
                    if lanesStruct(egoStartLane).TravelDirection == roadrunner.hdmap.TravelDirection.Backward
                        laneGeometry = flipud(laneGeometry);
                    end
                    s = [0; cumsum(vecnorm(diff(laneGeometry),2,2))];
                    sNorm = mean(diff(s))/s(end);
                    sRefPath = linspace(egoStartDistance, egoGoalDistance, ceil(1/sNorm)+1);

                    xPath = interp1(s, laneGeometry(:,1), sRefPath);
                    yPath = interp1(s, laneGeometry(:,2), sRefPath);
                    zPath = interp1(s, laneGeometry(:,3), sRefPath);

                    refPath = unique([xPath' yPath' zPath'],'stable','rows');
                    refPathLaneIDs = egoStartLane;
                    refPathRange = [0 egoGoalDistance-egoStartDistance];
                else
                    [refPath,refPathLaneIDs,refPathRange] = getRefPathIncludingSuccessor(lanesStruct,planner,queryAPI,graph,vehicleStart,egoStartLane,egoStartDistance,vehicleGoal,egoGoalLane,laneShiftNorm,laneChangeDistance);
                end
            else
                % Check if goal lane is an adjacent lane
                lateralConnection = 0;
                % Get connection types
                connectionLinks = graph.findlink([egoStartLane, egoGoalLane]);
                if connectionLinks~=0
                    % Connection exists
                    connectionTypes = graph.Links{connectionLinks,'EdgeType'};
                    % Check if goal point is at the back of the ego
                    % vehicle
                    if strcmp(connectionTypes,'Adjacent')
                        projection = getClosestPointOnLane(queryAPI,vehicleStart,egoGoalLane);
                        egoProjectionDistance = helperGetArcLength(lanesStruct,egoGoalLane,projection);
                        if egoGoalDistance<egoProjectionDistance
                            lateralConnection = 1;
                        end
                    end
                end
                if lateralConnection==1
                    [refPath,refPathLaneIDs,refPathRange] = getRefPathIncludingSuccessor(lanesStruct,planner,queryAPI,graph,vehicleStart,egoStartLane,egoStartDistance,vehicleGoal,egoGoalLane,laneShiftNorm,laneChangeDistance);
                else
                    % Find the shortest path using graph-based A* algorithm
                    [~, solnInfo] = plan(planner, egoStartLane, egoGoalLane);

                    % Check if all the lanes are adjacent lanes
                    lateralConnection=1;
                    for i = 1:length(solnInfo.PathStateIDs)-1
                        % Get connection types
                        connectionLinks = graph.findlink([solnInfo.PathStateIDs(i), solnInfo.PathStateIDs(i+1)]);
                        if connectionLinks~=0
                            % Connection exists
                            connectionTypes = graph.Links{connectionLinks,'EdgeType'};
                            % Check if both the lanes are not adjacent
                            if ~strcmp(connectionTypes,'Adjacent')
                                lateralConnection=0;
                                break;
                            end
                        end
                    end
                    % If all lanes are adjacent, check if end point is at
                    % the back of the ego vehicle
                    if lateralConnection
                        projection = getClosestPointOnLane(queryAPI,vehicleStart,egoGoalLane);
                        egoProjectionDistance = helperGetArcLength(lanesStruct,egoGoalLane,projection);
                        % If goal position is ahead of start position,
                        % consider the lane group for reference path.
                        if egoGoalDistance>egoProjectionDistance
                            lateralConnection = 0;
                        end
                    end
                    
                    if ~isempty(solnInfo.PathStateIDs) && ~lateralConnection
                        [refPath,refPathLaneIDs,refPathRange] = getRefPathFromLanes(lanesStruct,queryAPI,graph,vehicleStart,egoStartLane,vehicleGoal,egoGoalLane,solnInfo.PathStateIDs,laneShiftNorm,laneChangeDistance);
                    elseif lateralConnection
                        [refPath,refPathLaneIDs,refPathRange] = getRefPathIncludingSuccessor(lanesStruct,planner,queryAPI,graph,vehicleStart,egoStartLane,egoStartDistance,vehicleGoal,egoGoalLane,laneShiftNorm,laneChangeDistance);
                    else
                        error("Unable to find path with the specified start and goal positions");
                    end
                end
            end

            if nvp.PlotScene
                % Loop through each of the lane specifications and plot their coordinates
                figureName = 'Lane Level Path Planner Status Plot';
                fig = findobj('Type','Figure','Name',figureName);
                if isempty(fig)
                    fig = figure('Name',figureName);
                    fig.NumberTitle = 'off';
                    fig.MenuBar = 'none';
                    fig.ToolBar = 'none';
                end
                % Clear figure
                clf(fig);
                hAxes = axes(fig);
                hold on
                for i = 1 : numel(lanesStruct)
                    laneGeometry = lanesStruct(i).Geometry;
                    plot(hAxes, laneGeometry(:,1), laneGeometry(:,2), 'black');
                end
                xlabel('X(m)')
                ylabel('Y(m)')
            end
        end

        % Weight function for the navGraph object
        function weight = weightFunction(state1,state2,weightMatrix,varargin)
            % Weight function for navGraph object. Gets the weight between state1
            % and state2 from the weight matrix. It takes an optional input,
            % blocked lane ID and updates the weight multiplier of the blocked lane
            % to infinity.

            if ~(size(state1,1)==size(state2,1))
                if size(state1,1) == 1
                    state1 = repmat(state1,[length(state2),1]);
                else
                    state2 = repmat(state2,[length(state1),1]);
                end
            end

            % Get weight multiplier
            weightMultiplier = ones(length(state1),1);
            if nargin==4
                % When blocked lane is provided, weights for connected edges are
                % made infinity.
                blockedNode = varargin{1};
                % Check with state1
                weightMultiplier(state1==blockedNode) = weightMultiplier(state1==blockedNode)+inf;
                % Check with state2
                weightMultiplier(state2==blockedNode) = weightMultiplier(state2==blockedNode)+inf;
            end

            weight = zeros(size(state1,1),1);
            for i = 1:length(state1)
                weight(i) = weightMatrix(state1(i), state2(i))*weightMultiplier(i);
            end
        end

    end
end

% Gets reference path for the provided lane groups.
function [refPath,refPathLaneIDs,refPathRange] = getRefPathFromLanes(lanesStruct, queryAPI, graph, vehicleStart, startLane, vehicleGoal, goalLane, laneIDs, laneShiftNorm, laneChangeDistance)
    % vehicle starting point information
    closestPoint = getClosestPointOnLane(queryAPI,vehicleStart,startLane);
    [~, startLaneArcLengthRatio] = helperGetArcLength(lanesStruct,startLane,closestPoint);

    % vehicle ending point information
    closestPoint = getClosestPointOnLane(queryAPI,vehicleGoal,goalLane);
    [~, goalLaneArcLengthRatio] = helperGetArcLength(lanesStruct,goalLane,closestPoint);

    % Check for lateral connections
    isConnectionLateral = false(length(laneIDs)-1,1);
    for i = 1:length(isConnectionLateral)
        isConnectionLateral(i) = strcmp(graph.Links.EdgeType(graph.findlink([laneIDs(i),laneIDs(i+1)])),'Adjacent');
    end
    % remove consecutive lateral connections only the first one
    % and the last in consecutive lateral connections are
    % needed
    flagKeep = true(length(isConnectionLateral),1);
    for i = 1:length(isConnectionLateral)
        if i>1
            if isConnectionLateral(i) && isConnectionLateral(i-1)
                flagKeep(i) = false;
            end
        end
    end
    PathIDLane = laneIDs([flagKeep;true]);
    isConnectionLateral = isConnectionLateral(flagKeep);

    laneGeometryArcLengths = repmat([0,1],length(PathIDLane),1);
    % Initial lane arclength
    laneGeometryArcLengths(1,1) = startLaneArcLengthRatio;
    % Final lane arclength
    laneGeometryArcLengths(end,2) = goalLaneArcLengthRatio;

    % Check for lane change feasibility and update arc lengths for
    % lane change.
    dropLane = false(length(PathIDLane),1);
    if any(isConnectionLateral)
        idxLateralFirst = find(isConnectionLateral,10,'first');
        for i = 1:length(idxLateralFirst)
            [laneGeometryArcLengths, dropLane] = addIntermediateWaypoints(lanesStruct, queryAPI, idxLateralFirst(i), PathIDLane, laneGeometryArcLengths, dropLane, laneShiftNorm, laneChangeDistance);
        end
    end
    % Update lanes and arc lengths
    PathIDLane = PathIDLane(~dropLane);
    laneGeometryArcLengths = laneGeometryArcLengths(~dropLane,:);
    isConnectionLateral = isConnectionLateral(~dropLane(1:end-1));

    refPath = zeros(0,3);
    refPathLaneIDs = zeros(1,length(PathIDLane));
    refPathRange = zeros(1,length(PathIDLane));
    for i = 1:length(PathIDLane)
        % Get lane geometry
        laneGeometry = lanesStruct(PathIDLane(i)).Geometry;
        % Flip lane center geometry if travel direction is "Backward"
        if lanesStruct(PathIDLane(i)).TravelDirection == roadrunner.hdmap.TravelDirection.Backward
            laneGeometry = flipud(laneGeometry);
        end
        s = [0, cumsum(vecnorm(diff(laneGeometry),2,2))'];

        if ~all(laneGeometryArcLengths(i,:)==[0,1])
            if laneGeometryArcLengths(i,1)==0
                laneGeometryArcLengths(i,1)=min(s(2)/s(end),laneGeometryArcLengths(i,2));
            end
            if i>1 && isConnectionLateral(i-1)
                idStart = find(s<laneGeometryArcLengths(i,1)*s(end),1,'last');
                if laneGeometryArcLengths(i,2)==1
                    idEnd = length(s);
                else
                    idEnd = find(s>laneGeometryArcLengths(i,2)*s(end),1,'first');
                end
                % Merge reference path and lane geometry with lane change
                refPath = mergeReferencePath(refPath, laneGeometry, idStart:idEnd);

                sTotal = sum(vecnorm(diff(laneGeometry),2,2));
                refPathLaneIDs(i) = PathIDLane(i);
                refPathRange(i) = sTotal;
            elseif i==1 && all(isConnectionLateral==1)
                % If there is a lane change from the first lane.
                idStart = find(s<laneGeometryArcLengths(i,1)*s(end),1,'last');
                refPath = laneGeometry(idStart:idStart+1,:);
            else
                sPath = linspace(laneGeometryArcLengths(i,1),laneGeometryArcLengths(i,2),length(s));
                xPath = interp1(s, laneGeometry(:,1), sPath*s(end));
                yPath = interp1(s, laneGeometry(:,2), sPath*s(end));
                zPath = interp1(s, laneGeometry(:,3), sPath*s(end));

                refPath = unique([refPath; [xPath' yPath' zPath']],'stable','rows');
                refPathLaneIDs(i) = PathIDLane(i);
                if i==1
                    refPathRange(i) = diff(laneGeometryArcLengths(i,:))*s(end);
                else
                    refPathRange(i) = refPathRange(i-1)+diff(laneGeometryArcLengths(i,:))*s(end);
                end
            end
        else
            % Append reference path and lane geometry
            refPath = [refPath;laneGeometry(2:end,:)];
            refPathLaneIDs(i) = PathIDLane(i);
            if i==1
                refPathRange(i) = s(end);
            else
                refPathRange(i) = refPathRange(i-1)+s(end);
            end
        end
    end
end

% Gets reference path through the successor of the starting lane
function [refPath,refPathLaneIDs,refPathRange] = getRefPathIncludingSuccessor(lanesStruct, planner, queryAPI, graph, vehicleStart, egoStartLane, egoStartDistance, vehicleGoal, egoGoalLane, laneShiftNorm, laneChangeDistance)
    % Get successor lanes
    [connectedLaneIdx,edgeWeight] = graph.successors(egoStartLane);
    % Get connection types
    connectionLinks = graph.findlink([repmat(egoStartLane, [length(connectedLaneIdx),1]) connectedLaneIdx]);
    connectionTypes = graph.Links{connectionLinks,'EdgeType'};
    % Get successor with least weight
    successorsFound = strcmp(connectionTypes,'Successor');
    if any(successorsFound)
        [~,minWeightIdx] = min(edgeWeight(successorsFound));
        allSuccessorIdx = connectedLaneIdx(successorsFound);
        successorIdx = allSuccessorIdx(minWeightIdx);
        successorStart = getClosestPointOnLane(queryAPI,vehicleStart,successorIdx);

        % Find the path from the successor.
        % Find the shortest path using graph-based A* algorithm
        [~, solnInfo] = plan(planner, successorIdx, egoGoalLane);
        if ~isempty(solnInfo.PathStateIDs)
            [refPath,refPathLaneIDs,refPathRange] = getRefPathFromLanes(lanesStruct,queryAPI,graph,successorStart,successorIdx,vehicleGoal,egoGoalLane,solnInfo.PathStateIDs,laneShiftNorm,laneChangeDistance);
        else
            error("Unable to find path with the specified start and goal positions");
        end

        % Append refPath
        laneGeometry = lanesStruct(egoStartLane).Geometry;
        if lanesStruct(egoStartLane).TravelDirection == roadrunner.hdmap.TravelDirection.Backward
            laneGeometry = flipud(laneGeometry);
        end
        s = [0; cumsum(vecnorm(diff(laneGeometry),2,2))];
        sNorm = mean(diff(s))/s(end);
        sRefPath = linspace(egoStartDistance, s(end), ceil(1/sNorm)+1);

        xPath = interp1(s, laneGeometry(:,1), sRefPath);
        yPath = interp1(s, laneGeometry(:,2), sRefPath);
        zPath = interp1(s, laneGeometry(:,3), sRefPath);
        pathDistance = s(end)-egoStartDistance;

        refPath = unique([[xPath' yPath' zPath'];refPath],'stable','rows');

        refPathLaneIDs = [egoStartLane, refPathLaneIDs];
        refPathRange = [pathDistance refPathRange+pathDistance];
    else
        error("Unable to find path with the specified start and goal positions");
    end
end

% Adds starting and ending points for lane change
function [laneGeometryArcLengths, dropLane] = addIntermediateWaypoints(lanesStruct, queryAPI, idxLateralFirst, refPathLaneID, laneGeometryArcLengths, dropLane, laneShiftNorm, lcDistance)
    % Starting point on the lane for lane change
    startLCArcLength = laneGeometryArcLengths(idxLateralFirst,1)+laneShiftNorm*(1-laneGeometryArcLengths(idxLateralFirst,1));
    laneGeometryArcLengths(idxLateralFirst,2) = startLCArcLength;

    % Get projection of starting point of lane change to the final
    % lane.
    laneGeometry = lanesStruct(refPathLaneID(idxLateralFirst)).Geometry;
    if lanesStruct(refPathLaneID(idxLateralFirst)).TravelDirection == roadrunner.hdmap.TravelDirection.Forward
        AngleSign = 1;
    elseif lanesStruct(refPathLaneID(idxLateralFirst)).TravelDirection == roadrunner.hdmap.TravelDirection.Backward
        AngleSign = -1;
        laneGeometry = flipud(laneGeometry);
    end

    dx = diff(laneGeometry(:,1));
    dy = diff(laneGeometry(:,2));
    temp = atan2(dy,dx);
    heading = [temp(1);temp];
    s = [0; cumsum(vecnorm(diff(laneGeometry),2,2))];

    vehState.Position = [interp1(s, laneGeometry(:,1), startLCArcLength*s(end)),...
                        interp1(s, laneGeometry(:,2), startLCArcLength*s(end)),...
                        interp1(s, laneGeometry(:,3), startLCArcLength*s(end))];
    vehState.Roll     = 0;
    vehState.Pitch    = 0;
    vehState.Yaw      = interp1(s, heading, startLCArcLength*s(end))*AngleSign;

    projection = getClosestPointOnLane(queryAPI,vehState,refPathLaneID(idxLateralFirst+1));
    [projectionArcLength, projectionArcLengthRatio] = helperGetArcLength(lanesStruct,refPathLaneID(idxLateralFirst+1),projection);

    % Remaining distance for lane change
    totalDistance = projectionArcLength/projectionArcLengthRatio;
    remainingDistance = (1-projectionArcLengthRatio)*totalDistance;

    if remainingDistance>lcDistance || idxLateralFirst+1 == length(refPathLaneID)
        % Ending point on the lane for lane change
        endLCArcLengthRatio = projectionArcLengthRatio+lcDistance/totalDistance;
        laneGeometryArcLengths(idxLateralFirst+1,1) = min(endLCArcLengthRatio,laneGeometryArcLengths(idxLateralFirst+1,2));
    else
        dropLane(idxLateralFirst+1) = true;
        % Get end point of the lane
        laneGeometry = lanesStruct(refPathLaneID(idxLateralFirst)).Geometry;
        totalDistance = sum(vecnorm(diff(laneGeometry),2,2));
        % Ending point on the lane for lane change
        endLCArcLengthRatio = (lcDistance-remainingDistance)/totalDistance;
        laneGeometryArcLengths(idxLateralFirst+2,1) = min(endLCArcLengthRatio,laneGeometryArcLengths(idxLateralFirst+2,2));
    end
end

% Merge reference path with lane geometry to get proper path for lane
% change.
function refPath = mergeReferencePath(refPath, laneGeometry, sSectionIdx)
    if isempty(refPath)
        refPath = [refPath; laneGeometry];
    else
        numPoints = min(5,size(refPath,1)-1);
        partialRefPath = refPath(end-numPoints:end,:);
        laneArcLength = [0; cumsum(vecnorm(diff(laneGeometry),2,2))];
        refPathArcLength = [0; cumsum(vecnorm(diff(partialRefPath),2,2))];
        denom = laneArcLength(end);
        if denom ==0
            denom = 0.1;
        end
        sLaneExistNorm = refPathArcLength/denom;
        sLaneNewShift = laneArcLength(sSectionIdx)-laneArcLength(sSectionIdx(1));
        sLaneNewNorm  = sLaneNewShift/(sLaneNewShift(end));
        % mean end arclength
        sEndNorm = (refPathArcLength(end)+sLaneNewShift(end))/2;
        % mean resolution in (0~1)
        sResNorm = mean([diff(sLaneExistNorm); diff(sLaneNewNorm)]);
        sCommonNorm = linspace(0,1,ceil(1/sResNorm)+1)';

        % interpolate reference path.
        xLaneExist = interp1(refPathArcLength, partialRefPath(:,1), sCommonNorm*refPathArcLength(end));
        yLaneExist = interp1(refPathArcLength, partialRefPath(:,2), sCommonNorm*refPathArcLength(end));

        % interpolate lateral lane.
        xLaneNew = interp1(sLaneNewShift, laneGeometry(sSectionIdx,1), sCommonNorm*sLaneNewShift(end));
        yLaneNew = interp1(sLaneNewShift, laneGeometry(sSectionIdx,2), sCommonNorm*sLaneNewShift(end));

        % Get lane change path using clothoid spline.
        [s, X, Y] = clothoidSpline(sResNorm*sEndNorm, [xLaneExist yLaneExist;xLaneNew yLaneNew]);

        Z = interp1([refPathArcLength;s(end)], [partialRefPath(:,3);laneGeometry(sSectionIdx(2),3)], s);
        % Append reference path.
        refPath = [refPath(1:end-numPoints,:); [X(2:end) Y(2:end) Z(2:end)]];
    end
end

function [s,x,y] = clothoidSpline(res, waypointsIn)
% CLOTHOIDSPLINE fits the clothoid for the waypoints
course = nan(size(waypointsIn,1),1);
cspline = matlabshared.tracking.internal.scenario.makeClothoidSpline(waypointsIn, course);
s_res = (0:res:cspline.hcd(end))';
s = [s_res;cspline.hcd(end)];
s = unique(s);
z0 = matlabshared.tracking.internal.scenario.clothoidInterpolate(s(:), cspline);
x = real(z0);
y = imag(z0);
end
