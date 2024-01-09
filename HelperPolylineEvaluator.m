classdef HelperPolylineEvaluator < matlab.System
    %HelperPolylineEvaluator controls the vehicle to follow a reference path.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.

    % Copyright 2021 The MathWorks, Inc.
    % Pre-computed constants
    properties(Access = private)
        Distance = 0
    end

    methods(Access = protected)
        function [posX, posY, posZ, yaw, routeDistance, routeFinished] = stepImpl(obj, polyline, polylineLength, timestep, speed)
            % Calculate vehicle pose and distance travelled using polyline
            % from RoadRunner Scenario and speed
            targetDistance = obj.Distance + speed * timestep;
            polylineLength=double(polylineLength);
            previousDistance = 0;
            cumulativeDistance = 0;
            previousPointIndex = 0;
            nextPointIndex = 0;
            foundDistance = false;
            
            for idx = 2:polylineLength
                previousDistance = cumulativeDistance;
                cumulativeDistance = cumulativeDistance + norm(polyline(idx, :) - polyline(idx-1, :));
                
                if cumulativeDistance > targetDistance
                    previousPointIndex = idx - 1;
                    nextPointIndex = idx;
                    
                    obj.Distance = targetDistance;
                    foundDistance = true;
                    break;
                end
            end
            
            % Check for end of route
            if ~foundDistance
                obj.Distance = cumulativeDistance;
                nextPointIndex = polylineLength;
                previousPointIndex = nextPointIndex - 1;
                routeFinished = true;
            else
                routeFinished = false;
            end
            
            previousPoint = polyline(previousPointIndex, :);
            nextPoint = polyline(nextPointIndex, :);
            
            %    prev pos   target pos
            %    |          |
            % *--O----*-----X---*----------*
            %         |     | 
            % --------- previousDistance
            % --------------- target distance
            % 
            % s will be the parametric distance between previousPoint and
            % nextPoint.
            %
            s = (obj.Distance - previousDistance) / norm(nextPoint - previousPoint);
            
            interpolatedPosition = previousPoint + s * (nextPoint - previousPoint);
            tangent = (nextPoint - previousPoint)/ norm(nextPoint - previousPoint);
            
            % Blend tangents
            if nextPointIndex <= polylineLength - 1
                nextNextPoint = polyline(nextPointIndex + 1, :);
                nextTangent = (nextNextPoint - nextPoint)/norm(nextPoint - previousPoint);
                tangent = tangent + s *(nextTangent - tangent);
                tangent = tangent / norm(tangent);
            end
            
            % Setting this to the RoadRunner traffic coordinate system.
            % - y : forward
            % - x : right
            % - z : up
            yaw = -atan2(tangent(1), tangent(2));
            posX = interpolatedPosition(1);
            posY = interpolatedPosition(2);
            posZ = interpolatedPosition(3);
            routeDistance = obj.Distance;
        end
    end
end
