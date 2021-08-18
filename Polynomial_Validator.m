%   This class defines a template for creating a custom state validation definition
%   that can be used by the sampling-based path planners like plannerRRT and
%   plannerRRTStar. The state validator allows you to validate states and
%   motions between states.
%
%   To access documentation for how to define a state space, enter the following
%   at the MATLAB command prompt:
%
%    >> doc nav.StateValidator
%
%   For a concrete implementation of the same interface, see the following
%   nav.StateValidator class:
%
%    >> edit validatorOccupancyMap
%
%
%   To use this custom state validator for path planning, follow the steps
%   outlined below and complete the class definition. Then, save this
%   file somewhere on the MATLAB path. You can add a folder to the
%   path using the ADDPATH function.




classdef Polynomial_Validator < nav.StateValidator & ...
        matlabshared.planning.internal.EnforceScalarHandle
    
    %---------------------------------------------------------------------
    % Step 1: Define properties to be used by your state space. These are
    % user-defined properties.
    properties
        
        %------------------------------------------------------------------
        % Place your code here
        %------------------------------------------------------------------
        ValidationDistance
        Map
    end
    
    %----------------------------------------------------------------------
    % Step 2: Define functions used for managing your state validator.
    methods
        % a) Use the constructor to set the name of the state space, the
        %    number of state variables, and to define its boundaries.
        %
        function obj = Polynomial_Validator(space)
            
            narginchk(0,1)
            
            if nargin == 0
                space = stateSpaceSE2;
            end
            
            % The state space object is validated in the StateValidator base class
            obj@nav.StateValidator(space);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % b) Define how the object is being copied (a new object is created
        %    from the current one). You have to customize this function
        %    if you define your own properties or special constructor.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/copy
        %
        function copyObj = copy(obj)
            
            % Default behavior: Create a new object of the same type with no arguments.
            copyObj = feval(class(obj), obj.StateSpace);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % c) Define how a given state is validated. The STATE input can be a
        %    single state (row) or multiple states (one state per row).
        %    You have to customize this function if you want to have
        %    special validation behavior, for example to check for
        %    collisions with obstacles.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isStateValid
        %
        function isValid = isStateValid(obj, state)
            
            narginchk(2,2); 
            
            nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                "isStateValid", "state");
            
            % Default behavior: Verify that state is within bounds
            % (!) only checks position
            bounds = obj.StateSpace.StateBounds';
            global v_max
            %v_max = 50/3.6;
            a_max = 2*sqrt(2); % let be larger to prevent interpState from being invalid. 2 is a conservative bound.
            global sample_acc
            if sample_acc
                new_bounds = [bounds(:,1:3),[-v_max ; v_max],[-a_max ; a_max],[-a_max ; a_max]];
            else
                new_bounds = [bounds(:,1:end-1),[-v_max ; v_max]];
            end
            bounds = new_bounds;
            inBounds = state >= bounds(1,:) & state <= bounds(2,:);
            isValid = all(inBounds, 2);
            if isValid == false
                return
            end
            
            % collision checking: obj is stateValidator object w/ method
            % property Map w/ method
            % checkOccupancy - returns 1 if collision, 0 if free, therefore
            % isValid should be 1 if valid
            isValid = ~obj.Map.checkOccupancy(state(:,1:2));
            
            % vehicle coll.check:
            L_car = 6; % length
            w_car = 2.60; % width
            L_t = 7.82;
            L_t = 1.1*L_t; % inflate  trailer to account for heading angle errors
            w_t = 1.1*w_car;
            d_car = sqrt((0.5*L_car)^2+(0.5*w_car)^2); % diagonal length of car from center to corner
            d_t = sqrt((0.5*L_t)^2+(0.5*w_t)^2); % diagonal length of trailer from center to corner
            a = -0.66; % dist. from tractor axis to hitch
            b = 1.1*7.65; % dist from hitch to trailer rear axis
            alpha_car = atan(w_car/L_car); % angle from center to corner
            alpha_t = atan(w_t/L_t);
            
            x_vec = state(:,1); % flat point, vector of all states
            y_vec = state(:,2); 
            alpha_vec = state(:,3); 
            beta_vec = pi+alpha_vec; % assume same heading as tractor
            % tractor:
            % A-B-C-D according to quadrants w/ 0 heading.
            A_x = x_vec + d_car*cos(alpha_vec-alpha_car);
            A_y = y_vec + d_car*sin(alpha_vec-alpha_car);
            B_x = A_x - w_car*cos(pi/2-alpha_vec);
            B_y = A_y + w_car*sin(pi/2-alpha_vec);
            C_x = B_x - L_car*cos(alpha_vec);
            C_y = B_y - L_car*sin(alpha_vec);
            D_x = A_x - L_car*cos(alpha_vec);
            D_y = A_y - L_car*sin(alpha_vec);
            % trailer:
            x_trailer=x_vec-(a*cos(alpha_vec)+b*cos(pi-beta_vec));
            y_trailer=y_vec-(a*sin(alpha_vec)-b*sin(pi-beta_vec));
            A_x_t = x_trailer + d_t*cos((beta_vec-pi)-alpha_t);
            A_y_t = y_trailer + d_t*sin((beta_vec-pi)-alpha_t);
            B_x_t = A_x_t - w_t*cos(pi/2-(beta_vec-pi));
            B_y_t = A_y_t + w_t*sin(pi/2-(beta_vec-pi));
            C_x_t = B_x_t - L_t*cos((beta_vec-pi));
            C_y_t = B_y_t - L_t*sin((beta_vec-pi));
            D_x_t = A_x_t - L_t*cos((beta_vec-pi));
            D_y_t = A_y_t - L_t*sin((beta_vec-pi));
            
            interp_vec = linspace(0,1,10);
            % create intermediate points on vehicle:
            % (!) only need x or y-position, take x
            line1_x = B_x + interp_vec.*(C_x_t-B_x);
            line1_y = B_y + interp_vec.*(C_y_t-B_y);
            line1 = [line1_x , line1_y];
            line2_x = A_x + interp_vec.*(D_x_t-A_x);
            line2_y = A_y + interp_vec.*(D_y_t-A_y);
            line2 = [line2_x , line2_y];
            
            % check collision for each line for state i
            for i=1:length(A_x)
                line1_x_T = line1_x';
                line1_y_T = line1_y';
                line1_i = [line1_x_T(:,i) , line1_y_T(:,i)];
                line2_x_T = line2_x';
                line2_y_T = line2_y';
                line2_i = [line2_x_T(:,i) , line2_y_T(:,i)];
                
                % check if line is coll.free
                line1_valid = ~obj.Map.checkOccupancy(line1_i(:,1:2)); % return vector of 0 (free) and 1 (occupied) (or -1 if outside map)
                invalid_index1 = find(~line1_valid, 1); % find first invalid state in line
                line2_valid = ~obj.Map.checkOccupancy(line2_i(:,1:2)); % return vector of 0 and 1
                invalid_index2 = find(~line2_valid, 1); % find first collided state in line (not equal to zero)
         
                if isempty(invalid_index1) && isempty(invalid_index2)
                    % whole line is valid, do nothing, that state is valid
                else
                    % crashes, set isValid index to 0
                    isValid(i)=0; % set corresponding state to invalid
                end
            end     
   
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % d) Define how a motion between states is validated.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isMotionValid
        %
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            
            narginchk(3,3);
            
            state1 = nav.internal.validation.validateStateVector(state1, ...
                obj.StateSpace.NumStateVariables, "isMotionValid", "state1");
            state2 = nav.internal.validation.validateStateVector(state2, ...
                obj.StateSpace.NumStateVariables, "isMotionValid", "state2");
            

%             if (~obj.isStateValid(state1))
%                 error("statevalidator:StartStateInvalid", "The start state of the motion is invalid.");
%             end
            
            % Default behavior: Interpolate with some fixed interval
            % between state1 and state2 and validate the interpolated
            % points.
            numInterpPoints = 100; % not used but provides directions for interpolator
            interpStates = obj.StateSpace.interpolate(state1, state2, linspace(0,1,numInterpPoints));

            %interpStates
            %interpStates = interpStates(:,1:2); % only take position
            % above line commented out to avoid calling the solver again
            global free_path_map state_couple;
            state_couple = [state1 ; state2];
            %interpStates = state_matrix; %obtained from distance function
            
            global collision_status;
            
            interpValid = obj.isStateValid(interpStates);
            
            firstInvalidIdx = find(~interpValid, 1); % find first collided state in state_matrix
            global coll_counter
            coll_counter = coll_counter+1
            collision_status = false; % set false to reset
            if isempty(firstInvalidIdx)
                isValid = true;
                lastValid = state2;
                % whole path is coll.free: store path in map
                if (~obj.isStateValid(state1))
                    error("statevalidator:StartStateInvalid", "The start state of the motion is invalid.");
                end
                
                free_path_map(num2str(state_couple)) = interpStates;
                disp('PATH ADDED')
            else
                collision_status = true; % true to discard collided path altogether
                isValid = false;
                lastValid = interpStates(firstInvalidIdx-1,:);
            end
            
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
    end
end

