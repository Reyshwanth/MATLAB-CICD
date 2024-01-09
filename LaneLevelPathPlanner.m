classdef LaneLevelPathPlanner < matlab.System
    %LaneLevelPathPlanner is used to plan the path of the vehicle and
    %follow the path in RoadRunner Scenario.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.

    % Copyright 2022-2023 The MathWorks, Inc.

    properties
        % Properties for behavior parameters
        GoalPoint
        LaneChangeDistance = 20;
        EnableGroundFollowing = 0;
        Replan
        ObstacleDistance
    end

    properties(Access = private)
        % ActorSimulation object
        ActorObj

        % ScenarioSimulation object
        ScenarioSimulationObj

        % Simulation step size
        StepSize

        % Path information
        Path
        NumPoints
        PathLaneIDs
        PathRange

        % Path Controller
        PolylineEvaluatorObj

        % Query API
        QueryAPI

        % Codegen Map
        Map

        % navGraph
        Graph

        % Previous values
        PreviousDistance = 0;

        % Axes for plotting the scene and the path
        Haxes
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Find the ActorSimulation object of the runtime actor
            obj.ActorObj = Simulink.ScenarioSimulation.find("ActorSimulation",SystemObject=obj);
            % Get the ScenarioSimulation object.
            obj.ScenarioSimulationObj = Simulink.ScenarioSimulation.find("ScenarioSimulation",SystemObject=obj);

            % Get map information from RoadRunner
            map = obj.ScenarioSimulationObj.getMap;

            % Get codegen map
            obj.Map = matlabshared.roadrunner.codegenHDMap(map.map);

            % Query API
            obj.QueryAPI = roadrunner.internal.roadrunnerHDMapQuery(obj.Map);

            % Update lane struct to split bidirectinal lane
            obj.Map.Lanes = helperSplitBidirectionalLane(obj.Map.Lanes);

            % Get node table and edge table
            [nodeTable, edgeTable] = helperGetNodesEdges(obj.Map.Lanes);

            % Get weight matrix
            adjMat = getLaneAdjacency(obj.QueryAPI);

            % Create navGraph object
            graph = navGraph(nodeTable, edgeTable, LinkWeightFcn=@(state1, state2, graph) HelperPlanPath.weightFunction(state1,state2,adjMat));
            obj.Graph = graph;

            % Path Controller
            obj.PolylineEvaluatorObj = HelperPolylineEvaluator;

            % Step size
            obj.StepSize = get(obj.ScenarioSimulationObj,"StepSize");

            %% Plan the path
            if isempty(obj.Path)
                % Get start position of the vehicle.
                initialPose = getAttribute(obj.ActorObj,"Pose");
                startPosition = initialPose(1:3,4)';

                % Get ego goal point as position of the vehicle named "Goal".
                allActors = get(obj.ScenarioSimulationObj,"ActorSimulation");
                for actor = allActors
                    actorModel = get(actor,"ActorModel");
                    if strcmp(getAttribute(actorModel,"Name"),"Goal")
                        pose = actor.getAttribute("Pose");
                        goalPosition = pose(1:3,4)';
                        obj.GoalPoint = goalPosition;
                        break
                    end
                end
                goalPosition = obj.GoalPoint;

                % Create graph-based A* planner.
                planner = plannerAStar(obj.Graph);

                [refPath, refPathLaneIDs, refPathRange]= HelperPlanPath.planPath(planner, ...
                                    obj.Graph, startPosition, goalPosition, ...
                                    'QueryAPI', obj.QueryAPI, ...
                                    'Map', obj.Map, ...
                                    'LaneChangeDistance',obj.LaneChangeDistance, ...
                                    'UseCustomWeightFunction',true, ...
                                    'PlotScene',false);

                % Update path and number of points
                obj.Path = refPath;
                obj.NumPoints = size(obj.Path,1);
                obj.PathLaneIDs = refPathLaneIDs;
                obj.PathRange = refPathRange;
            end
            %% Plot position on map
            lanes = map.map.lanes;
            % Loop through each of the lane specifications and plot their coordinates
            figureName = "Lane Level Path Planner Status";
            fig = findobj('Type','Figure','Name',figureName);
            if isempty(fig)
                fig = figure('Name',figureName);
                fig.NumberTitle = 'off';
                fig.MenuBar = 'none';
                fig.ToolBar = 'none';
            end
            % Clear figure
            clf(fig);
            obj.Haxes = axes(fig);
            xlabel('X(m)')
            ylabel('Y(m)')
            hold on
            for i = 1 : numel(lanes)
                control_points = lanes(i).geometry.values;
                x_coordinates = arrayfun(@(cp)  cp.x, control_points);
                y_coordinates = arrayfun(@(cp)  cp.y, control_points);
                plot(obj.Haxes, x_coordinates, y_coordinates, 'black');
            end
            % Plot the reference path
            plot(obj.Haxes, refPath(:,1),refPath(:,2),Color="g",LineWidth=2);
            % Plot markers for start and goal position
            plot(obj.Haxes, startPosition(1),startPosition(2),"o",Color="b",MarkerFaceColor="b",MarkerSize=4);
            plot(obj.Haxes, goalPosition(1),goalPosition(2),"o",Color="r",MarkerFaceColor="r",MarkerSize=4);
        end

        function stepImpl(obj)
            % Read current pose and speed information
            speed = norm(getAttribute(obj.ActorObj,"Velocity"));
            pose = getAttribute(obj.ActorObj,"Pose");
            rotation = rotm2eul(pose(1:3,1:3));
            pose = struct('Position', pose(1:3,4)', 'Yaw', rad2deg(rotation(1))+90, 'Roll', rad2deg(rotation(3)), 'Pitch', rad2deg(rotation(2)));

            % Replan the trajectory
            if obj.Replan
                obstacleDistance = obj.ObstacleDistance;
                % Consider that lane at obstacleDistance is blocked.
                replanPath(obj,pose,obstacleDistance);
            end

            % Get target pose
            [posX, posY, posZ, yaw, routeDistance, routeFinished] = obj.PolylineEvaluatorObj.stepImpl(obj.Path, obj.NumPoints, obj.StepSize, speed);

            % Plot ego position
            plot(obj.Haxes,posX,posY,'o',MarkerSize=1,Color='blue')

            if obj.EnableGroundFollowing
                % Get updated pose information with pitch and roll.
                laneID = getCurrentLane(obj.QueryAPI,pose);
                pose = getClosestPointOnLane(obj.QueryAPI,pose,laneID);
            end

            % Calculate tranformation matrix to write to RoadRunner 
            pose = packPose(obj, [posX, posY, posZ], [yaw, deg2rad(pose.Roll), deg2rad(pose.Pitch)]);
            % Calculate velocity from speed and yaw
            % Convert to RR coordinate system by changing yaw by 90
            % degrees.
            velocity = speed*[-sin(yaw) cos(yaw) 0];

            % Update pose
            setAttribute(obj.ActorObj,Pose=pose);
            setAttribute(obj.ActorObj,Velocity=velocity);
            % Update previous distance
            obj.PreviousDistance = routeDistance;
            % Stop simulation if route is completed.
            if routeFinished
                set(obj.ScenarioSimulationObj,SimulationCommand="Stop")
            end
        end

        % Pack the pose into 4x4 matrix.
        function pose = packPose(~, position, rotation)
            % position contains [x y z] information of vehicle in order.
            % rotation contains [yaw roll pitch] information of vehicle in
            % order.
            
            % Get rotation matrix
            rotm = eul2rotm(rotation);
            pose = [[rotm position']; [0 0 0 1]];
        end

        % Replan the trajcectory when there is a lead vehicle in the
        % reference path at a distance of obstacleDistance.
        function replanPath(obj,pose,obstacleDistance)
            % Get the lane which is at a distance of obstacleDistance.
            distanceOnPath = obstacleDistance+obj.PreviousDistance;
            blockedLaneIndex = find(obj.PathRange>distanceOnPath,1);
            blockedLaneID = obj.PathLaneIDs(blockedLaneIndex);
            % Get weight matrix
            adjMat = getLaneAdjacency(obj.QueryAPI);
            % Update navGraph object by updating weight of blockedLaneID
            % to infinite.
            graph = obj.Graph;
            graph.LinkWeightFcn = @(state1, state2, graph) HelperPlanPath.weightFunction(state1,state2,adjMat,blockedLaneID);

            % Create graph-based A* planner.
            planner = plannerAStar(graph);
            [refPath, refPathLaneIDs, refPathRange] = HelperPlanPath.planPath(planner, graph, pose.Position, obj.GoalPoint, ...
                                'QueryAPI', obj.QueryAPI, ...
                                'Map', obj.Map, ...
                                'LaneChangeDistance',obj.LaneChangeDistance, ...
                                'UseCustomWeightFunction',true, ...
                                'PlotScene',false);

            % Merge reference path
            s = [0; cumsum(vecnorm(diff(obj.Path),2,2))];
            currentIdx = find(s<obj.PreviousDistance,1,'last');
            obj.Path = [obj.Path(1:currentIdx,:);refPath];
            obj.NumPoints = size(obj.Path,1);

            % Update PathLaneIds and PathRange
            currentLaneIdx = find(obj.PathRange>obj.PreviousDistance,1);
            obj.PathLaneIDs = [obj.PathLaneIDs(1:currentLaneIdx) refPathLaneIDs(2:end)];
            obj.PathRange = [obj.PathRange(1:currentLaneIdx) refPathRange(2:end)-refPathRange(1)+obj.PathRange(currentLaneIdx)];

            % Plot updated path
            plot(obj.Haxes, refPath(:,1),refPath(:,2),Color=[0.8500 0.3250 0.0980],LineWidth=2);

            % Update the replan flag
            obj.Replan = 0;
        end

    end
end
