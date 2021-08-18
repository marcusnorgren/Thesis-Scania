function isReached = CustomGoalReachedFcn(planner, goalState, newState)
    isReached = false;
    threshold = 5; % distance tolerance
    angle_threshold = 15*pi/180; % angle tolerance
%     if planner.StateSpace.distance(newState, goalState) < threshold
%         isReached = true;
%     end
    if norm(newState(1:2)-goalState(1:2)) < threshold
        if abs(newState(3)-goalState(3)) < angle_threshold
            isReached = true;
        end
    end
end


