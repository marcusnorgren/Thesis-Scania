%   This class defines a template for creating a custom state space definition
%   that can be used by the sampling-based path planners like plannerRRT and
%   plannerRRTStar. The state space allows sampling, interpolation, and calculating
%   the distance between states.
%
%   To access documentation for how to define a state space, enter the following
%   at the MATLAB command prompt:
%
%    >> doc nav.StateSpace
%
%   For a concrete implementation of the same interface, see the following
%   nav.StateSpace class:
%
%    >> edit stateSpaceSE2
%
%
%   To use this custom state space for path planning, follow the steps
%   outlined below and complete the class definition. Then, save this
%   file somewhere on the MATLAB path. You can add a folder to the
%   path using the ADDPATH function.


%global goal; %added,  change later
%global function_counter solverTime_list
classdef Polynomial_StateSpace_acc < nav.StateSpace & ...
        matlabshared.planning.internal.EnforceScalarHandle
    
    %---------------------------------------------------------------------
    % Step 1: Define properties to be used by your state space.
    % All properties in this section are user-defined properties.
    properties
        
        %UniformDistribution - Uniform distribution for sampling
        UniformDistribution
        
        %NormalDistribution - Normal distribution for sampling
        NormalDistribution
        
        %------------------------------------------------------------------
        % Place your properties here or replace default properties.
        %------------------------------------------------------------------
        
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation
    end
    
    %----------------------------------------------------------------------
    % Step 2: Define functions used for managing your state space.
    methods
        % a) Use the constructor to set the name of the state space, the
        %    number of state variables, and to define its boundaries.
        %
        function obj = Polynomial_StateSpace_acc(bounds)
            
            spaceName = "Polynomial_Optimization";
            global numStateVariables 
            numStateVariables = 6; %prev. 4, now 6 w/ acc
            v_eps = 3;
            K_max = 0.17;
            
            v_max = 50/3.6; % keep here for now, only needed if no external statebounds is given when creating the object
            acc_max = 2; % max. long/lat. acceleration [m/s^2]
            acc_max = K_max*v_eps^2; % sample within smaller range to exclude infeasible points
            % sample pos_x, pos_y, angle, speed
            % For each state variable define the lower and upper valid
            % limit (one [min,max] limit per row)
            
            % Default state bounds if none is given, 
            stateBounds = [-100 100; -100 100; -pi pi; -v_max/sqrt(2) v_max/sqrt(2); -acc_max acc_max; -acc_max acc_max]; % (!) might have to be +- v_max/sqrt(2)
            %stateBounds = [-100 100; -100 100; -pi pi; -v_max/sqrt(2) v_max/sqrt(2)];% -acc_max acc_max; -acc_max acc_max];
            % Call the constructor of the base class
            obj@nav.StateSpace(spaceName, numStateVariables, stateBounds);
            
            
            % if state bounds is given as input
            if nargin ~= 0
                obj.StateBounds = bounds;
            end
            
            % Create the probability distributions for sampling.
            obj.NormalDistribution = matlabshared.tracking.internal.NormalDistribution(numStateVariables);
            obj.UniformDistribution = matlabshared.tracking.internal.UniformDistribution(numStateVariables);
            
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
        end
        
        % b) Define how the object is being copied (a new object is created
        %    from the current one). You have to customize this function
        %    if you define your own properties or special constructor.
        %
        %    For more help, see
        %    >> doc nav.StateSpace/copy
        %
        function copyObj = copy(obj)
            
            % Default behavior: Create a new object of the same type with no arguments.
            copyObj = feval(class(obj));
            copyObj.StateBounds = obj.StateBounds;
            copyObj.UniformDistribution = obj.UniformDistribution.copy;
            copyObj.NormalDistribution = obj.NormalDistribution.copy;
            
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
        end
        
        % c) Define how states are forced to lie within the state boundaries.
        %    You have to customize this function if you want to have
        %    special bounding behavior, for example for wrapped states like
        %    angles. The STATE input can be a single state (row) or
        %    multiple states (one state per row).
        %
        %    For more help, see
        %    >> doc nav.StateSpace/enforceStateBounds
        %
        function boundedState = enforceStateBounds(obj, state)
            
            % Default behavior: States are saturated to the [min,max] interval
            nav.internal.validation.validateStateMatrix(state, nan, obj.NumStateVariables, "enforceStateBounds", "state");
            boundedState = state;
            boundedState = min(max(boundedState, obj.StateBounds(:,1)'), ...
                obj.StateBounds(:,2)');
            
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
        end
        
        % d) Define how you can sample uniformly in your state space.
        %    You need to support the following calling syntaxes:
        %    STATE = sampleUniform(OBJ)
        %    STATE = sampleUniform(OBJ, NUMSAMPLES)
        %    STATE = sampleUniform(OBJ, NEARSTATE, DIST)
        %    STATE = sampleUniform(OBJ, NEARSTATE, DIST, NUMSAMPLES)
        %    You have to customize this function if you deal with angular
        %    state variables.
        %
        %    For more help, see
        %    >> doc nav.StateSpace/sampleUniform
        %
        % (!) PlannerRRT uses sampleUniform 
        function state = sampleUniform(obj, varargin)
            
            narginchk(1,4);
            [numSamples, stateBounds] = obj.validateSampleUniformInput(varargin{:});
            
            % Default behavior: Sample uniformly in all state variables
            % based on the user input.a
            obj.UniformDistribution.RandomVariableLimits = stateBounds;
            
            global K_max acc_max;
            sample_acc = true;
            
            zero_curv_sampling = true; % set true to only sample zero curvature
            
            if sample_acc == true
            state = obj.UniformDistribution.sample(numSamples);
            % now sample again y_acc with updated bounds to ensure
            % curvature constraint satisfied
            
            speed = abs(state(4));
            
            % curv. constraint will give a circle w/ radius K_max*speed^2
            % centered at origin
            % as bounds for the acceleration (for all angles)
            acc_lower = -K_max*speed^2;
            acc_upper = K_max*speed^2;
          
            x_acc = state(5);
            y_acc = state(6); 
            
            % resample x_acc if needed
            if x_acc < acc_lower || x_acc > acc_upper
                new_bounds = [stateBounds(1:end-2,:); acc_lower, acc_upper ; stateBounds(end,:)];
                % generate new samples:
                obj.UniformDistribution.RandomVariableLimits = new_bounds;
                new_sample = obj.UniformDistribution.sample(numSamples);
                new_state = [state(1:4),new_sample(5),state(6)]; % add new x_acc to sampled state
                state = new_state;
            end
            % resample y_acc if needed
            if y_acc < acc_lower || y_acc > acc_upper
                new_bounds = [stateBounds(1:end-1,:); acc_lower, acc_upper];
                % generate new samples:
                obj.UniformDistribution.RandomVariableLimits = new_bounds;
                new_sample = obj.UniformDistribution.sample(numSamples);
                new_state = [state(1:5),new_sample(6)]; % add new y_acc to sampled state
                state = new_state;
            end
            
            else
                % w/o sampled acceleration
                state = obj.UniformDistribution.sample(numSamples);
            end
            
            if zero_curv_sampling % samples only zero curvature
                state = [state(1:4),0,0];
            end
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
        end
        
        % e) Define how you can sample a Gaussian distribution in your
        %    state space. You need to support the following calling
        %    syntaxes:
        %    STATE = sampleGaussian(OBJ, MEANSTATE, STDDEV)
        %    STATE = sampleGaussian(OBJ, MEANSTATE, STDDEV, NUMSAMPLES)
        %    You have to customize this function if you deal with angular
        %    state variables.
        %
        %    For more help, see
        %    >> doc nav.StateSpace/sampleGaussian
        %
        function state = sampleGaussian(obj, meanState, stdDev, varargin)
            
            narginchk(3,4);
            
            % Default behavior: Sample from a multi-variate normal
            % distribution based on the user input.
            [meanState, stdDev, numSamples] = obj.validateSampleGaussianInput(meanState, stdDev, varargin{:});
            
            % Configure normal distribution for sampling in all state variables
            obj.NormalDistribution.Mean = meanState;
            obj.NormalDistribution.Covariance = diag(stdDev.^2);
            
            % Sample state(s)
            state = obj.NormalDistribution.sample(numSamples);
            
            % Make sure all state samples are within state bounds. This
            % saturation is not ideal, since it distorts the normal
            % distribution on the state boundaries, but similar to what OMPL is doing.
            state = obj.enforceStateBounds(state);
            
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
        end
        
        % f) Define how you can interpolate between two states in your
        %    state space. FRACTION is a scalar or a vector, where each number
        %    represents a fraction of the path segment length in the [0,1] interval.
        %
        %    For more help, see
        %    >> doc nav.StateSpace/interpolate
        %
        % (!) initially only interpState as output
        
        function interpState = interpolate(obj, state1, state2, fraction)
            % returns matrix of interpolated path states with fraction
            % increment distance
            narginchk(4,4);
            
            %disp('interpolate-function')
            
            state1 = double(state1);
            state2 = double(state2);
            
            global state_matrix % state_matrix obtained from distance function
            global state_matrix_final
            global distance_map distance_list
            global maxConnDist
            global numStateVariables
            global states_in_tree
            
            %global current_distance
            global NaN_distance
            NaN_distance = 1e6;
            global solved_dist_map
            global distance_list_map % map with distance lists
            global distance_map_Map
            
            % return right away if solver has failed:
            if length(fraction)==1
                if fraction <= maxConnDist/NaN_distance
                    %disp('Solver failed: returning interp NaN')
                    interpState = NaN*ones(1,numStateVariables);
                    return
                end
            end
            %interpState =  -1*ones(numStateVariables,1)'; % to prevent error
            
            % set state to infeasible if NaN, (!) depends on bounds of map
%             if any(isnan(state1), 'all')
%                state1 = -1*ones(1,numStateVariables);
%             end
%             if any(isnan(state2), 'all')
%                state2 = -1*ones(1,numStateVariables);
%             end

            % pick out correct state_matrix from state-couple-map:
            global state_matrix_map
            state_couple = [state1 ; state2];
            % return if NaN: otherwise will have error in state_matrix_map,
            % will be state2 that is NaN, state1 should be valid.
            if any(isnan(state_couple), 'all')
               interpState = [state1 ; NaN*ones(1,numStateVariables)];
               return
            end
                        
            %disp('Extract state_matrix')
            try
                state_matrix = state_matrix_map(num2str(state_couple));
            catch
                state_matrix = [state1 ; NaN*ones(1,numStateVariables)];
            end
            
            % Validate and make sure that state vectors are rows, since the builtins
            % require that input format.
            if ~obj.SkipStateValidation
                [state1, state2, fraction] = obj.validateInterpolateInput(state1, state2, fraction);
            end
            
            % if fraction is scalar, then function is used to get state at
            % max.connection distance, interpState should be single vector
            % if fraction is vector, then function is used to validate the
            % whole motion, interpState should be matrix
            if length(fraction)==1 % function is used to find nearest state below max.conn.dist
                % get interpState from list of state nearest below the
                % max.connection distance
                
                % take out correct distance_list
                distance_list = distance_list_map(num2str(state_couple));
                distance_map = distance_map_Map(num2str(state_couple));
                for state_index=1:length(distance_list)
                    % loop backwards to find nearest distance below max.con.dist
                    dist_key = distance_list(length(distance_list)-(state_index-1));
                    % obtain closest state
                    % (!) if just below will be obsolete by code above, can remove
                    if isnan(dist_key) % solver has failed and distance is NaN_distance
                        interpState = ones(1,numStateVariables)*NaN;
                        return;
                    end
                    if dist_key <= maxConnDist %closest state has been found
                        %disp('interpolated state found:')
                        interpState = distance_map(num2str(dist_key));
                        time_index = length(distance_list)+1-(state_index-1); %+1 since distance list is one shorter than state_matrix
                        
                        %check if state already in tree
                        for i = 1:size(states_in_tree,1)
                            tree_state = states_in_tree(i,:);
                            if tree_state == interpState % state already in tree, discard
                                interpState = ones(1,numStateVariables)*NaN;
                                return;
                            end
                        end
                        
                        % store state-couple in map:
                        % pick out polynomial:
                        global poly_map_x poly_map_y time_map_T
                        x = poly_map_x(num2str(state_couple));
                        y = poly_map_y(num2str(state_couple));
                        T = time_map_T(num2str(state_couple));
                        T_list = linspace(0,T,size(state_matrix,1));
                        T_interp = T_list(time_index);
                        
                        % create new state-couple:
                        state_couple = [state1 ; interpState];
                        solved_dist_map(num2str(state_couple)) = dist_key;
                        % update poly-maps to include new state-couple:
                        poly_map_x(num2str(state_couple))= x;
                        poly_map_y(num2str(state_couple))= y;
                        time_map_T(num2str(state_couple))= T_interp;
                        % else return the interpolated state:
                        %state not in tree, return the interpolated state
                        %NaN - will be discarded by motionValidator
                        
                        
                        %end
                        
                        % update interpolated path (state_matrix) to only
                        % include points below max.conn.distance --> needed
                        % for motionValidator - (!) custom validator
                        %disp('INTERPOLATE')
                        % update state_matrix and store it in map
                        state_matrix = state_matrix(1:time_index,:);
                        % store interpolated states in state_matrix_map
                        % state_couple now is [state1 ; interpState]
                        state_matrix_map(num2str(state_couple)) = state_matrix;
                        %state_couple
                        %state_matrix
                        
                        
                        return;
                        %break; % break when closest state is found
                    end
                end    
            else % function is used to check motion validity (isMotionValid), output must be matrix
                %set interpolated states to same states as in distance
                %function. (!) ignore fraction input and set this within
                %function to get same result
                %disp('validate Motion');
                %[state1 ; state2]
                interpState = state_matrix;
                
            end
            %set sampled angle and speed to end angle/speed if sampled
            %point is close enough to goal: 
            % this prevents motionValidator from crashing when the
            % validation step is larger than the interpolated path, length
            
            %if state2(1:2) == round(goal(1:2))

            
            %[state1, state2, fraction] = obj.validateInterpolateInput(state1, state2, fraction);
            
            %stateDiff = state2 - state1;
            %interpState = state1 + ' * stateDiff;
            
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
            
            
        end
        
        % g) Define how you can calculate the distance between two states in
        %    your state space. The STATE1 and STATE2 inputs can be a single
        %    state (row) or multiple states (one state per row).
        %
        %    For more help, see
        %    >> doc nav.StateSpace/distance
        %
        function dist = distance(obj, state1, state2)
             global poly_map_x poly_map_y time_map_T solved_dist_map
             global states_in_tree
             
             %disp('distance!')
   
%              state1
%              state2
             
             % nearestNeighbour will use state in tree as state2 and the
             % random state as state1 --> won't work! must fix:
             % check if state2 is already in tree, if so switch order of
             % state1 and state2:
             for i = 1:size(states_in_tree,1)
                 if state2 == states_in_tree(i,:)
                     state2_new = state1;
                     state1 = state2;
                     state2 = state2_new;
                 end
             end
             

             
             % 6 b.c. --> >= 6 order needed for opt.problem
             N = 10; % order of polynomial, will have to increase w/ no.of b.c
             % use only polynomial:
             %[dist,x,y,T,t_eps,solverStatus] = Steer_acc(obj,state1,state2,N);
             % use Dubin + polynomial:
             %[dist,x,y,T,t_eps,solverStatus] = multiSteer_Dubin(obj,state1,state2,N);
             
             [dist,x,y,T,solverStatus] = Steer_Dubin(obj,state1,state2,N);
             
             
             state_couple = [state1 ; state2];
             
        
%             if dist==0
%                 dist = 1e6 %zero to avoid entering interpolate function, go directly to isMotionValid and dismiss it there
%             end
            %--------------------------------------------------------------
            % Place your code here or replace default function behavior.
            %--------------------------------------------------------------
            
        end
        
    end
end

