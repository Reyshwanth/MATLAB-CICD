load("plannerworkspace.mat");
graphObj = navGraph(newnodeTable,newedgeTable);
planner = plannerAStar(graphObj);
pathOutput=zeros(1000,3);
[pathOutput1,solutionInfo1] = plan(planner,start,goal);
if size(pathOutput1)~=0
    pathOutput([1,size(pathOutput1,1)],:)=pathOutput1([1,size(pathOutput1,1)],:);
end
solutionInfo=solutionInfo1.IsPathFound;


