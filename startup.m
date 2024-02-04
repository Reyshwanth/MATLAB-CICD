%delete(rrSim);
clear;
rrAppPath = "C:\Program Files\RoadRunner R2023a\bin\win64";
s = settings;
s.roadrunner.application.InstallationFolder.TemporaryValue = rrAppPath;
rrProj = "C:\MATLABautodrive\multistop\Test\Test";

rrApp=roadrunner(rrProj, NoDisplay=true);
%rrApp=roadrunner(rrProj,NoDisplay=false);

openScenario(rrApp,"LeadCutIn3.rrscenario");

open_system("MILSILmultistop")
rrSim = rrApp.createSimulation

hdMap = getMap(rrSim);
lanes = hdMap.map.lanes;


%set(rrSim,"SimulationCommand","Start");
%while strcmp(get(rrSim,"SimulationStatus"),"Running")
%    pause(1);
%end

%rrLog = get(rrSim,"SimulationLog");

%poseActor1 = rrLog.get('Pose','ActorID',1);
%positionActor1_x = arrayfun(@(x) x.Pose(1,4),poseActor1);
%positionActor1_y = arrayfun(@(x) x.Pose(2,4),poseActor1);
%plot(positionActor1_x,positionActor1_y,"r","LineWidth",2)
%
%poseActor2 = rrLog.get('Pose','ActorID',2);
%positionActor2_x = arrayfun(@(x) x.Pose(1,4),poseActor2);
%positionActor2_y = arrayfun(@(x) x.Pose(2,4),poseActor2);
%plot(positionActor2_x,positionActor2_y,"b","LineWidth",2)
%
%title("Agent Positions from RoadRunner Scenario")
%ylabel("Y (m)")
%xlabel("X (m)")

[nodeTable,edgeTable] = helperGetNodesEdges;

graph = navGraph(nodeTable,edgeTable);
%show(graph)

numDestinations = 4
% Choose your method of node selection:
RandomNodes = true
HardCodedNodes = false %technically not used, might be used later

% Below is an if-else statement determining our method of node selection
if RandomNodes 
    numLanes = height(nodeTable);
    % Select random starting node
    astart = randi([1 numLanes],1,1);
    numLaneNodesStart = height(nodeTable.LanesStruct(astart).Geometry);
    bstart = randi([1 numLaneNodesStart],1,1);
    xpointstart = nodeTable.LanesStruct(astart).Geometry(bstart,1)
    ypointstart = nodeTable.LanesStruct(astart).Geometry(bstart,2)
    zpointstart = nodeTable.LanesStruct(astart).Geometry(bstart,3)
    start = [xpointstart,ypointstart,zpointstart]
    % Select random goal node
    % Define four goal positions
    goals = zeros(4,3); % Initialize a 4x3 matrix to store goal positions
    for i = 1:numDestinations
        agoal = randi([1 numLanes],1,1);
        numLaneNodesGoal = height(nodeTable.LanesStruct(agoal).Geometry);
        bgoal = randi([1 numLaneNodesGoal],1,1);
        xpointgoal = nodeTable.LanesStruct(agoal).Geometry(bgoal,1);
        ypointgoal = nodeTable.LanesStruct(agoal).Geometry(bgoal,2);
        zpointgoal = nodeTable.LanesStruct(agoal).Geometry(bgoal,3);
        goals(i,:) = [xpointgoal, ypointgoal, zpointgoal];
    end
    %printing starting position
    % figure
    % hold on
    % for i = 1:numel(lanes)
    %     control_points = lanes(i).geometry.values;
    %     x_coords = arrayfun(@(cp) cp.x,control_points);
    %     y_coords = arrayfun(@(cp) cp.y,control_points);
    %     Z_coords = arrayfun(@(cp) cp.z,control_points); % Make sure cp.Z is correctly cased
    %     %plot(x_coords, y_coords, 'k.-');
    %     plot3(x_coords, y_coords, Z_coords, 'k.-'); % Use plot3 for 3D plotting
    % end
    % plot(xpointstart, ypointstart, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    %plot3(xpointstart, ypointstart, zpointstart, 'go', 'MarkerSize', 10, 'LineWidth', 2); % Add zpointstart for 3D plotting
    
    % goalColors = ['r', 'b', 'm', 'c'];
    % Printing goal positions
%     for i = 1:numDestinations
%         xpointgoal = goals(i, 1);
%         ypointgoal = goals(i, 2);
%         zpointgoal = goals(i, 3); % Add z-coordinate for the goal
%         plot(xpointgoal, ypointgoal, [goalColors(i) 'o'], 'MarkerSize', 10, 'LineWidth', 2);
%         %plot3(xpointgoal, ypointgoal, zpointgoal, [goalColors(i) 'o'], 'MarkerSize', 10, 'LineWidth', 2); % Use plot3 for 3D plotting
%     end
% 
%     hold off
%     axis equal
else 
        % Storing designated locations
        % [Note:The hard-coded destination portion does not work yet]
    locations(1).name = 'Diagonal Parking';
    locations(1).position = [21.2448, 95.449, 236.404];
    
    locations(2).name = 'Curbside Parking';
    locations(2).position = [21.3648, 37.0707, 235.611];
    
    locations(3).name = 'Parallel Parking';
    locations(3).position = [17.6015, 112.126, 236.376];
    
    locations(4).name = 'Main St.';
    locations(4).position = [15.6745, 108.834, 236.407];
    
    locations(5).name = 'Liberty St.';
    locations(5).position = [19.2656, 40.7919, 235.604];
    % 
    % locations(7).name = 'State St.';
    % locations(7).position = [-8.71713, 74.0647];
    % 
    % locations(8).name = 'Pontaic Tr.';
    % locations(8).position = [-44.7704, 154.126];
    % 
    % locations(9).name = 'Railroad Crossing';
    % locations(9).position = [-50.553, 71.8247];
    % 
    % locations(10).name = 'Metal Tunnel';
    % locations(10).position = [-57.2846, 16.6779];
    % 
    % locations(11).name = 'Wolverine Ave.';
    % locations(11).position = [49.8527, 53.1169];
    % 
    % locations(12).name = 'Carrier & Gabble Dr.';
    % locations(12).position = [71.4524, 184.41];
    % 
    % locations(13).name = 'Vine Tunnel';
    % locations(13).position = [21.01, 144.815];
    
    %make a bike lane node and see what it does when it goes there
    
    %Getting a random location
    index = randi(length(locations));
    randomLocation = locations(index);
    disp(randomLocation);
    
    numDestinations = 4;
    
    % Define four random locations
    randomLocations = zeros(numDestinations, 3); % Initialize a 4x2 matrix to store locations
    selectedLocations = cell(numDestinations, 1); % To store names of selected locations
    
    for i = 1:numDestinations
        index = randi(length(locations));
        randomLocations(i, :) = locations(index).position;
        selectedLocations{i} = locations(index).name;
    end
    
    % Print section
    figure
    hold on
    
    for i =1:numel(lanes)
        control_points = lanes(i).geometry.values;
        x_coords = arrayfun(@(cp) cp.x,control_points);
        y_coords = arrayfun(@(cp) cp.y,control_points);
        plot(x_coords,y_coords, 'k.-');
    end
    %plot(xpointstart, ypointstart, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Plot start location (choose one random location as starting point)
    startLocationIndex = randi(length(locations));
    startLocation = locations(startLocationIndex).position;
    startLocationName = locations(startLocationIndex).name;
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Display starting location
    disp(['Starting at ' startLocationName]);
    
    % Plot goal locations
    goalColors = ['r', 'b', 'm', 'c'];
    for i = 1:numDestinations
        plot(randomLocations(i, 1), randomLocations(i, 2), [goalColors(i) 'o'], 'MarkerSize', 10, 'LineWidth', 2);
        % Display each destination
        disp(['Destination ' num2str(i) ': ' selectedLocations{i}]);
        goals(i,:) = [randomLocations(i, 1), randomLocations(i, 2), randomLocations(i, 3)]; %updating the goals for the finder
    end
    
    hold off
    axis equal
end


% egoStart = strjoin(string(start),",");
% setScenarioVariable(rrApp,"EgoInitialPosition",egoStart);
% egoStartPosition = getScenarioVariable(rrApp,"EgoInitialPosition");
% egoStartPosition = str2double(split(egoStartPosition,","))';

%Conversion of the map such that it is code generatable

states=nodeTable{:,1};
[size1,size2]=size(edgeTable);
links=zeros(size1,2);
for i=1:size1
    if (string(edgeTable{i,"EdgeType"})=='Predecessor')
        links(i,:)=flip(edgeTable{i,"EndStates"});
    else
        links(i,:)=edgeTable{i,"EndStates"};
    end
end

links=edgeTable{:,1};

nodeindices=[];
newnodeTable=[];
newedgeTable=[];
initial=1;
for i=1:size(nodeTable,1)
    newnodeTable=cat(1,newnodeTable,nodeTable.LanesStruct(i,1).Geometry);
    final=size(newnodeTable,1);
    nodeindices=cat(1,nodeindices,[initial,final]);
    for j=initial:(final-1)
        newedgeTable=cat(1,newedgeTable,[j,j+1]);
    end
    initial=final+1;
    
end

for i=1:size(links,1)
    newedgeTable=cat(1,newedgeTable,[nodeindices(links(i,1),2),nodeindices(links(i,2),1)]);
end

%This section runs our planner on the nodes stored in goals
% 
% planner = plannerAStar(graph);
% REFPATH = [];
% for i = 1:numDestinations
%     goal = goals(i,:);
% 
%     % Convert goal position to string for scenario variable
%     egoGoal = strjoin(string(goal),",");
%     setScenarioVariable(rrApp,"Goal",egoGoal);
%     egoGoalPosition = getScenarioVariable(rrApp,"Goal");
%     egoGoalPosition = str2double(split(egoGoalPosition,","))';
% 
%     try
%         % Plan path
%         refPath = HelperPlanPath.planPath(planner, graph, start, goal);
%         REFPATH = [REFPATH; refPath];
% 
%         % Plotting the path and markers...
%         pRefPath = plot(refPath(:,1), refPath(:,2), 'Color', "g", 'LineWidth', 2);
%         pStart = plot(start(1), start(2), "o", 'MarkerFaceColor', "b", 'MarkerSize', 4);
%         pEnd = plot(goal(1), goal(2), "o", 'MarkerFaceColor', "r", 'MarkerSize', 4);
%         legend([pRefPath, pStart, pEnd], {"Reference path", "Start position", "Goal position"}, 'Location', "northwest");
% 
%     catch ME
%         fprintf('Error planning path to goal: [%f, %f, %f]\n', goal(1), goal(2), goal(3));
%         fprintf('Error message: %s\n', ME.message);
%         % Optionally, display more detailed error information
%         fprintf('Error identifier: %s\n', ME.identifier);
%         fprintf('Error stack trace:\n');
%         disp(ME.stack);
%     end
% 
%     % Update start position for the next iteration
%     start = goal;
% end
% plot(REFPATH(:,1), REFPATH(:,2), 'c');
% 
% set(rrSim,SimulationCommand="Start")
% while strcmp(get(rrSim,"SimulationStatus"),"Running")
%     pause(0.01)
% end
% 
% setScenarioVariable(rrApp,ChangeSpeed_TargetSpeed="18")
% 
% obstacleDistance = 50;
% setScenarioVariable(rrApp,ReplanTriggerDistance=num2str(obstacleDistance));
% setScenarioVariable(rrApp,ObstacleDistance=num2str(obstacleDistance));
