function [pathOutput,solutionInfo]  = Planner(states,links,goal,start)

%PLANNER Summary of this function goes here
%   Detailed explanation goes here
graphObj = navGraph(states,links);
planner = plannerAStar(graphObj);
[pathOutput,solutionInfo] = plan(planner,start,goal)
 


