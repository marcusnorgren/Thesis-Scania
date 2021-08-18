% steering function:
function [dist,x,y,T,solverStatus] = Steer_Dubin(obj,state1,state2,N)
% steering function
% args in: START, GOAL, ORDER OF POLYNOMIALS
% returns distance of curve, and polynomial functions (derivatives as well,
% in x and y)
            global function_counter solverTime_list numStateVariables 
            global yaw_end speed_end start goal accel acc_end_x acc_end_y
            global accel_map
            global distance_map distance_list
            global state_matrix
            global maxConnDist
            global val_distance
            global NaN_distance %represent distance when solver has failed
            
            global states_in_tree
            global acc_max
            global solved_dist_map failed_dist_map
            global solved_list failed_list state_matrix_map
            global distance_list_map % map to store distance lists in
            global distance_map_Map % map to store distance maps
            global poly_travel dubin_travel
            
            global poly_map_x poly_map_y time_map_T % to store polynomials
            
            narginchk(4,4);
            
            x=0;
            y=0;
            T=0; 
            solverStatus = 0;
            
            global dist_limit; global dubin_time_list; global solved_time_list;
            
            % create distance map for finding the state along path at
            % max.connection distance
                 
            %disp('DISTANCE')
            
            state1 = double(state1);
            state2 = double(state2);
            
            NaN_distance = 1e6;
            
            state1_matrix = state1; %created for clarity
            
            % if state1 is matrix, function will be used in near() -->
            % avoid calling steer-function twice:
            nearest_neighbour = false;
            
            if size(state1_matrix,1)>1 
                %disp("!!!!!MATRIX!!!!!")
                nearest_neighbour = true; % function is used in nearest-neighbour
                states_in_tree = state1_matrix; % whole tree will be inputed
                % not accounting for when tree is only start should be ok
            end
            
            % (!) add: state_couple map to obtain distance in near()
            
            % if state1 input is matrix, the planner tries to connect to
            % nearest state in tree --> must handle each state
            % independently
            
            % create distance list, can be either vector or scalar
            % depending on if used in extending or connecting to tree
            no_of_state1 = size(state1,1);
            dist_list = zeros(no_of_state1,1); 
            
            dist_to_state = vecnorm(state1_matrix(:,1:2)-state2(1:2),2,2);
            % take out k smallest distances:
            k_near = 100; % k nearest neighbours to connect to
            if k_near <= no_of_state1
                k = k_near;
            else
                k = no_of_state1;
            end
            
            [~,k_nearest] = mink(dist_to_state,k);
            try_to_connect = zeros(no_of_state1,1);
            for i = k_nearest
                try_to_connect(i) = 1;
            end
            
            if nearest_neighbour
            for state1_number = 1:no_of_state1
                state1 = state1_matrix(state1_number,:);
                % check if reversing:
                % check sign of velocity:
                v1 = state1(4);
                v2 = state2(4);
                reversing = false;
                % ensure that the speeds have the same sign:
                if sign(v1)*sign(v2) >= 0
                    % reversing action:
                    if sign(v1) < 0 || sign(v2) < 0
                        % reversing action: reverse order of state1 and state2
                        state1_old = state1;
                        state2_old = state2;
                        state1 = state2_old;
                        state2 = state1_old;
                        % change sign of speeds:
                        state1(4) = -state2_old(4);
                        state2(4) = -state1_old(4);
                        reversing = true;

                    else
                        % do nothing, standard procedure, forward driving
                        % already
                    end
                
                    % now connect w/ dubin path to find nearest neighbour:
                   %% Connect w/ Dubin path & add Dubin waypoints:
                   %Create a dubinsConnection object.
                   dubConnObj2 = dubinsConnection;
                   K_max = 0.17; % max. curvature [1/m] % define here as well. error otherwise
                   dubConnObj2.MinTurningRadius = (1/K_max)*1.2; %1.2 scale in yaml file = 7m turn radius
                   %Define start and goal poses as [x y theta] vectors.
                   startPose = [state1(1) state1(2) state1(3)];
                   goalPose = [state2(1) state2(2) state2(3)];
                   %Calculate a valid path segment to connect the poses.
                   tic
                   pathSegObj2 = connect(dubConnObj2,startPose,goalPose);
                   dubin_time = toc;
                   dubin_distance = pathSegObj2{1}.Length;
                   dist_list(state1_number) = dubin_distance;
                   dubin_time_list = [dubin_time_list,dubin_time];
                   if state1_number == no_of_state1
                        % has looped through all states, now finished,
                        % return dist_list
                        dist = dist_list;
                        return;
                    end
                    continue % continue to next state1
                else
                    % if speeds have opposite sign - discard and return NaN
                    dist = NaN_distance;
                    dist_list(state1_number) = dist;
                    if state1_number == no_of_state1
                        % has looped through all states, now finished,
                        % return dist_list
                        dist = dist_list;
                        return;
                    end
                    continue % continue to next state1
                end
            end % end of state1-loop
            end %% checks if nearest neighbour function, if not then execute code below
            
            reversing = false; % reset.
            if ~nearest_neighbour
            for state1_number = 1:no_of_state1
                
                state1 = state1_matrix(state1_number,:);
                                
                state_couple = [state1 ; state2];
            
                
                % if dist already exists, just take it and continute to next iteration (next state1)
                try 
                    % pick out the correct state_matrix
                    state_matrix = state_matrix_map(num2str(state_couple));
                    dist = solved_dist_map(num2str(state_couple)); % check if distance is already solved for
                    dist_list(state1_number) = dist;
                    %disp('distance found!');
                    %dist
                    
                   % added, remove later, for visualization %%%%%%%%%%%%
%                    image = imread('parking_harbor.png');
%                    Img = im2bw(image);
%                    im_matrix = abs(Img-1); % convert 0 to 1 and vice versa
%                    % 335 metres top-bottom
%                    cell_par = 1.85; % gives 340 m
%                    harbor_map = occupancyMap(im_matrix,cell_par);
% 
%                    figure()
%                    hold on;
%                    show(harbor_map)
%                    plot(state_matrix(:,1),state_matrix(:,2))
%                    plot(states_in_tree(:,1),states_in_tree(:,2),'o')
%                    hold off;
                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    if state1_number == no_of_state1
                        % has looped through all states, now finished,
                        % return dist_list
                        dist = dist_list;
                        return;
                    end
                    continue % distance exists, continue to next state1
                catch
                    %disp('distance NOT found')
                    % continue solving for the distance, state-couple not
                    % solved, will continue to code below and solve for distance
                    % there
                end
                
              
            % if state1 is a matrix, take each row separately
            % state 2 will always be single row vector
                
           if state1 == state2 % return NaN_distance if sampled state is equal to already existing state
                dist = 0; %prev. NaN_distance
                return 
            end     
            
            
%            % added for avoiding NaN error: let state be invalid
            if any(isnan(state1), 'all')
               state1 = -1*ones(size(state1,1),numStateVariables);
               %state1 = NaN*ones(size(state1,1),numStateVariables);
               x = NaN; y = NaN; T = NaN; t_eps = NaN; solverStatus = 'NaN';
               dist = NaN_distance;
               return
            end
            if any(isnan(state2), 'all')
               state2 = -1*ones(size(state1,1),numStateVariables);
               %state2 = NaN*ones(size(state1,1),numStateVariables);
               x = NaN; y = NaN; T = NaN; t_eps = NaN; solverStatus = 'NaN';
               dist = NaN_distance;
               return
            end
            
            % check if distance b/w states are short enough, otherwise
            % discard: to prevent too many unnec. function calls to tree
            
            % discard if not one of k- nearest neighbours
            %if try_to_connect(state1_number) == 0 
            if norm(state1(1:2)-state2(1:2)) > 1*dist_limit
                dist_list(state1_number) = NaN_distance;
                if state1_number == no_of_state1
                    dist = dist_list; % ready to return, looped trough all states in tree
                    return
                end
                continue
            end
            
            % check if reversing:
            % check sign of velocity:
            v1 = state1(4);
            v2 = state2(4);
            reversing = false;
            % ensure that the speeds have the same sign:
            if sign(v1)*sign(v2) >= 0
                % reversing action:
                if sign(v1) < 0 || sign(v2) < 0
                    % reversing action: reverse order of state1 and state2
                    state1_old = state1;
                    state2_old = state2;
                    state1 = state2_old;
                    state2 = state1_old;
                    % change sign of speeds:
                    state1(4) = -state2_old(4);
                    state2(4) = -state1_old(4);
                    reversing = true;
                
                else
                    % do nothing, standard procedure, forward driving
                    % already
                end
            else
                % if speeds have opposite sign - discard and return NaN
                dist = NaN_distance;
                dist_list(state1_number) = dist;
                if state1_number == no_of_state1
                    % has looped through all states, now finished,
                    % return dist_list
                    dist = dist_list;
                    return;
                end
                continue % continue to next state1
            end  
            
            %if try_to_connect(state1_number) == 0 
            if norm(state1(1:2)-state2(1:2)) > 1*dist_limit
                dist_list(state1_number) = NaN_distance;
                if state1_number == no_of_state1
                    dist = dist_list; % ready to return, looped trough all states in tree
                    return
                end
                continue
            end
            
            accel_i = [0,0]; accel_end = [0,0]; % set accel to zero at both ends, will change if low speed scenario
           
%            [state1, state2] = obj.validateDistanceInput(obj.NumStateVariables, state1, state2);
            
            % test slow mech function: remove later:
            global v_eps 
            v_eps = 3.5; 
            global a0;
            a0 = 2;
            v_i = state1(4);
            v_end = state2(4);
            slow = false; slow_start = false; slow_end = false;
            if v_i < v_eps
                slow_start = true;
            end
            if v_end < v_eps
                slow_end = true;
            end 
            if slow_start == true || slow_end == true
                slow = true;
            end
            if slow
                [state1_new,state2_new,t_i_eps,t_f_eps,x_start,y_start,x_end,y_end,v_start,v_end] = slow_mech(state1,state2);
%                 figure()
%                 hold on;
%                 plot(state1(1),state1(2),'bo');
%                 plot(x_start(0:0.1:t_i_eps),y_start(0:0.1:t_i_eps))
%                 plot(x_end(-t_f_eps:0.1:0),y_end(-t_f_eps:0.1:0))
%                 plot(state1_new(1),state1_new(2),'ro');
%                 plot(state2(1),state2(2),'bo');
%                 plot(state2_new(1),state2_new(2),'ro');
%                 hold off;
            end
 
            % start optimizing from new states if needed
            slow_div = 10; % no. of states in slow section, just take 10
            state1_old = state1;
            state2_old = state2;
            if slow_start
                t_i_eps_list = linspace(0,t_i_eps,slow_div);
                
                state1 = state1_new(1:4); %w/o acc. only 1:4
                start_path = double([x_start(t_i_eps_list)',y_start(t_i_eps_list)',state1(3)*ones(10,1),v_start(t_i_eps_list)']);%,a0*cos(state1(3))*ones(10,1),a0*sin(state1(3))*ones(10,1)]);
                accel_i = [state1_new(5),state1_new(6)];
                %accel_map(num2str(state1_old)) = accel_i;
            else
                start_path = zeros(1,numStateVariables); % to prevent error
            end
            if slow_end
                t_f_eps_list = linspace(-t_f_eps,0,slow_div);
                end_path = double([x_end(t_f_eps_list)',y_end(t_f_eps_list)',state2(3)*ones(10,1),v_end(t_f_eps_list)']);%,-a0*cos(state2(3))*ones(10,1),-a0*sin(state2(3))*ones(10,1)]);
                
                state2 = state2_new(1:4);
                accel_end = [state2_new(5),state2_new(6)];
                %accel_map(num2str(state2_old)) = accel_end;
            else
                end_path = zeros(1,numStateVariables); % to prevent error
            end
         
            %%%%%%%%%% INSERT HERE %%%%%%%%%%%%%%%%%
            function_counter = function_counter+1; % only count function calls to solver
            
            %% initial/end conditions:
            %max speed, m/s
%            v_max = obj.StateBounds(4,2);
            global v_max
            %v_max = (50/3.6)/sqrt(2);
            %v_i = v_max/2; % initial speed
            v_i = state1(4);
            %v_i = speed_i; % passed along from solver
            %v_end = v_max/2; % end speed
            v_end = state2(4);
            %v_end = 4;
            %v_end = speed_end; % passed along from solver
            % initial heading
            alpha_i = state1(3);
            % final heading
            alpha_end = state2(3);

            % Vehicle params from Scania (Robin data):
            yaw_max = 15.708; % 900°
            yaw_rate_max = 7.854; % [rad/s] 450 degrees/s
            u2_max = yaw_rate_max;
            K_max = 0.17; % max. curvature [1/m]
            %acc_max = 2; % max. long. acceleration (from Robin data)
            acc_min = -acc_max;
            
            % acceleration continuity:
            if reversing %(state2 is now state1 --> check for that value in map)
                try
                    accel_end = double(accel_map(num2str(state2_old)));
                    %disp('ACC FOUND')
                    acc_end = accel_end; % value from previous sample, used in b.c.
                catch
                    accel_end = [0,0]; %set to zero if no matching value is found (initial point)
                    acc_end = accel_end;
                    %disp('acc not found');
                end 
                accel_i = [0,0]; % not used but here for error handling
            else
            % set initial acceleration for state1 for continuity
                try
                    accel_i = double(accel_map(num2str(state1_old)));
                    %disp('ACC FOUND')
                catch
                    accel_i = [0,0]; %set to zero if no matching value is found (initial point)
                    %disp('acc not found');
                end
            end
                

            % initial % RRT map is from [0 2.6], so scale w/ some factor  
            pos_i = [state1(1),state1(2)];
            vel_i = [v_i*cos(alpha_i),v_i*sin(alpha_i)]; 
            acc_i = accel_i; % either 0 or a0 from low speed mech.
            %acc_i = [0,0]; 
            
            % end:
            pos_end = [state2(1),state2(2)];
            vel_end = [v_end*cos(alpha_end),v_end*sin(alpha_end)]; 
            acc_end = accel_end; %either zero or taken from low speed mech.
            %acc_end = [0,0]; 
           
            d_k = 2; % order of derivative continuity (x^(k) continuous)
            d_k_all = d_k; 
            % (!) first/last must be equal with this structure, otherwise change
            % A_init etc.
            d_k_first = 2; % satisfy "all" derivatives, prev. 3 (!) 2
            d_k_last = 2; % satisfy vel and acc, now w/ sampled acc

            % RHS in opt.problem. (!) can add acc as sampled
            if d_k_last == 1
                d_init_x = [pos_i(1) ; vel_i(1) ; acc_i(1)];
                d_end_x = [pos_end(1) ; vel_end(1)];%; acc_end(1)];
                d_init_y = [pos_i(2) ; vel_i(2) ; acc_i(2)];
                d_end_y = [pos_end(2) ; vel_end(2)];%; acc_end(2)];
            else % d_k_last = 2 here:
                d_init_x = [pos_i(1) ; vel_i(1) ; acc_i(1)];
                d_end_x = [pos_end(1) ; vel_end(1); acc_end(1)];
                d_init_y = [pos_i(2) ; vel_i(2) ; acc_i(2)];
                d_end_y = [pos_end(2) ; vel_end(2); acc_end(2)];
            end
            % Euclidian distance:
            dist_tot = norm(pos_i-pos_end);
            global state_matrix_final
%             if dist_tot == 0
%                 state_matrix_final = state_matrix; %store final state_matrix
%             end
            
            %% Time allocation:

            M = 1; % no. of segments
            T_list = zeros(M+1,1);
            u2_max_list = zeros(M,1);
            
           %% Connect w/ Dubin path & add Dubin waypoints:
           %Create a dubinsConnection object.
           dubConnObj = dubinsConnection;
           dubConnObj.MinTurningRadius = (1/K_max)*1.2*1.5; %1.2 scale in yaml file = 7m turn radius
           %Define start and goal poses as [x y theta] vectors.
           startPose = [state1(1) state1(2) state1(3)];
           goalPose = [state2(1) state2(2) state2(3)];
           %Calculate a valid path segment to connect the poses.
           tic
           pathSegObj = connect(dubConnObj,startPose,goalPose);
           dubin_time = toc;
          
           dubConnObj2 = dubinsConnection;
           dubConnObj2.MinTurningRadius = (1/K_max)*1.2; %1.2 scale in yaml file = 7m turn radius
           %Define start and goal poses as [x y theta] vectors.
           startPose = [state1(1) state1(2) state1(3)];
           goalPose = [state2(1) state2(2) state2(3)];
           %Calculate a valid path segment to connect the poses.
           pathSegObj2 = connect(dubConnObj2,startPose,goalPose);
           dubin_dist2 = pathSegObj2{1}.Length;
           
           dubin_time_list = [dubin_time_list,dubin_time];
           
           
           dubinObj = pathSegObj{1};
           dubinObj.MotionLengths
           seg1 = dubinObj.MotionLengths(1);
           seg2 = dubinObj.MotionLengths(2);
           seg3 = dubinObj.MotionLengths(3);
           % take intermediate Dubin points:
           % interpolate along dubin path:
           dubin_dist = pathSegObj{1}.Length;
           
           speed = v_max/2;
           speed = 5; 
           % create time points for dubin points
           
           T_list(2) = dubin_dist/speed;
           
           T = T_list(end); % final 
           
           
           no_poses = 5; % no of intermediate points, 5-6-10 prev. good
           T_pose_list = linspace(0,T,no_poses+2);
           T_pose_list = T_pose_list(2:end-1);
           %no_poses = length(T_pose_list);

           % step along equal step length (0:step_length:tot_dist)
           step_length = dubin_dist/(no_poses+1-2);
           poses = interpolate(pathSegObj{1},0:step_length:dubin_dist);
           dubin_theta_list = poses(2:end-1,end);
            
            % v_eps = 3; % 3 ~ walking speed - gives +- 1.53 m/s^2 bounds in perp. acceleration
            %a0 = acc_max; % take max acc, change if needed

            %% MIT - Polynomial trajectory generation with QP

            syms t

            N = 6; % order of polynomials --> N+1 coefficients, comment
            %out if providing as input instead
            
            % d_k = 
            % 0: pos
            % 1: vel
            % 2: acc
            % 3: jerk
            % 4: snap
            % 5: crackle
            % 6: pop
            
            % midpoint constraints (speed, curvature)
            % M_mid corresponds to M in multi-segment formulation
            faculty_factor = @(N,k) factorial(N)/factorial(N-k);
            
            % enforce straight movement to pick up speed:
            % accelerate max (for now) in a straight line
            
            
            %used to set lowest speed in curv-constraint

            % Euclidian distance: %use dubin dist
            dist_tot = dubin_dist;

            % no. of intermediate points should be scaled with the segment
            % distance:
            mid_div = round(10*dist_tot/50); % take 10 points per each 50 metres
            if mid_div<=4
                mid_div=4; %ensure that intermediate points not negative, 4-2 = 2 middle points
            end
            %mid_div = 4;
            %mid_div = 10; % no. of intermediate time points (+2) in segment for constraints
            %T_mid_list = linspace(0,T,mid_div);
            % enforce curvature constraints only when speed is high enough
            T_mid_list = linspace(0,T,mid_div); %if t_eps = 0, speed is already high enough and curvature constraint is enforced on full path
            M_mid = size(T_mid_list,2)-1;
            seg_i = 0; 
            A_seg = zeros(d_k+1,N+1,M_mid-1); % matrix of segment times coefficients
            % (derivative,coefficient,time point)
            
            for time_point=2:length(T_mid_list)-1 %time point
            t = T_mid_list(time_point);
            seg_i=seg_i+1;
            for k=0:d_k %order of derivative
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_seg(k+1,col,seg_i)=faculty_factor(N-(col-1),k)*t^(N-(col-1)-k);
                end
            end
            end
            end
            

            % symbolic matrix, for creating derivatives:
            % change k - loop to change order of derivative satisfied at
            % end points
            syms t_sym
            
            % ignore slow mechanism, causes much complexity that must be handled:
            t_first = 0;
            t_last = T;
            

            if reversing % switch order of order of derivatives satisfied, only if not sampling acc. if so doesn't matter
                d_k_first_old = d_k_first; d_k_last_old = d_k_last;
                d_k_first = d_k_last_old;
                d_k_last = d_k_first_old;
            end
            
            A_first = zeros(d_k_first+1,N+1);
            A_last = zeros(d_k_last+1,N+1);
            
            % initial point
            for k=0:d_k_all %order of derivative, loop to order of derivative satisfied at points
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_first(k+1,col)=faculty_factor(N-(col-1),k)*t_first^(N-(col-1)-k);
                    A_last(k+1,col)=faculty_factor(N-(col-1),k)*t_last^(N-(col-1)-k);
                end
            end
            end
            % end point
            
            A_init = kron([1,zeros(1,M-1)],A_first);
            A_end = kron([zeros(1,M-1),1],A_last);

            % As steering function:
            A_full = [A_init(1:d_k_first+1,:) ; A_end(1:d_k_last+1,:)]; %
            d_fixed_x = [d_init_x ; d_end_x]; % vector of fixed constraints in x
            d_tot_x = d_fixed_x;
            d_fixed_y = [d_init_y ; d_end_y]; % vector of fixed constraints in y
            d_tot_y = d_fixed_y;
            
           
           %T_tot_list = sort([T_list ; T_half_list]);
           T_tot_list = T_pose_list; % from Dubin poses to take out angles
           
           % intermediate pose matrices, uniformly divided on whole path
           seg_i = 0; pt = 0; %reset
           A_poses = zeros(d_k+1,N+1,no_poses); % matrix of interm. segment times coefficients, add one in b/w each segment
           for time_point=1:length(T_pose_list) %time point, now include last time point compared to above
            t = T_pose_list(time_point); 
            pt = pt+1;
            for k=0:d_k %order of derivative
                for col=1:N+1 %polynomial coeff.
                    if (N-(col-1)-k) >= 0
                        A_poses(k+1,col,pt)=faculty_factor(N-(col-1),k)*t^(N-(col-1)-k);
                    end
                end
            end
           end
           
           A_speed_lower = []; A_curv_xy = [];
           pt = 0;
           for i=1:length(dubin_theta_list)
               pt = pt+1;
               theta = dubin_theta_list(i);
               time = T_tot_list(pt);
               %convert to degrees: % dubin uses [0,2pi]
               theta_deg = theta*180/pi;
               delta_deg = 22.5; 
               if cosd(315+delta_deg) <= cosd(theta_deg) && cosd(theta_deg) > cosd(0+delta_deg)
                   region = 0;
               elseif 0+delta_deg <= theta_deg && theta_deg < 45+delta_deg
                   region = 45;
               elseif 45+delta_deg <= theta_deg && theta_deg < 90+delta_deg
                   region = 90;
               elseif 90+delta_deg <= theta_deg && theta_deg < 135+delta_deg
                   region = 135;
               elseif 135+delta_deg <= theta_deg && theta_deg < 180+delta_deg
                   region = 180;
               elseif 180+delta_deg <= theta_deg && theta_deg < 225+delta_deg
                   region = 225;
               elseif 225+delta_deg <= theta_deg && theta_deg < 270+delta_deg
                   region = 270;
               elseif 270+delta_deg <= theta_deg && theta_deg < 315+delta_deg
                   region = 315;
               end
                % only use the quadrants w/ 45 deg angle bounds
%               if 0 <= theta_deg && theta_deg < 90
%                   region = 45;
%               elseif 90 <= theta_deg && theta_deg < 180
%                   region = 135;
%               elseif 180 <= theta_deg && theta_deg < 270
%                   region = 225;
%               elseif 270 <= theta_deg && theta_deg < 360
%                   region = 315;
%               end
                   
           % x_factor = {-1/sqrt(2),-1,0,1,1/sqrt(2)}, y_factor = {...}
           % switch bound depending on heading angle
           switch(region)
               case 0
                   x_factor = 1; y_factor = 0;
               case 45
                   x_factor = 1/sqrt(2); y_factor = 1/sqrt(2);
               case 90
                   x_factor = 0; y_factor = 1;
               case 135
                   x_factor = -1/sqrt(2); y_factor = 1/sqrt(2);
               case 180
                   x_factor = -1; y_factor = 0;
               case 225
                   x_factor = -1/sqrt(2); y_factor = -1/sqrt(2);
               case 270
                   x_factor = 0; y_factor = -1;
               case 315
                   x_factor = 1/sqrt(2); y_factor = -1/sqrt(2);
           end
               A_x = x_factor*A_poses(2,:,pt);
               A_y = y_factor*A_poses(2,:,pt);
               

               vec = 1;
               A_x = kron(vec,A_x); A_y = kron(vec,A_y);
               A_x_curv = A_poses(3,:,pt); A_y_curv = A_poses(3,:,pt);
               
               A_speed_lower = [A_speed_lower ; A_x , A_y];
               A_curv_xy = [A_curv_xy ; A_x_curv , A_y_curv];
           end % end of theta loop
           v_lower = v_eps; % lower speed bound, should ideally be same as v_eps
           d_speed_lower = v_lower*ones(size(A_speed_lower,1),1);
           
           min_speed = v_eps; % take somewhat lower here to be conservative
           acc_max = K_max*min_speed^2;
           %acc_max = 2; % 2 m/s^2 in both long. and lateral acc. is default bounds
%            A_acc_xy = kron(eye(2),A_acc);
%            d_acc_l = -acc_max*ones(size(A_acc_xy,1),1);
%            d_acc_u = acc_max*ones(size(A_acc_xy,1),1);
           d_curv_l = -acc_max*ones(size(A_curv_xy,1),1);
           d_curv_u = acc_max*ones(size(A_curv_xy,1),1);

           %size(A_full)

            %% Objective function:
            % costs for each derivative of position, 0 = position
            k_T = 1; % time-penalty
            c0 = 0; % pos
            c1 = 0; % speed
            c2 = 1; % acc
            c3 = 1; % jerk
            c4 = 0; % snap - osqp says problem non-convex (sometimes) when c4 = 1, why??
            c_vec = [c0 c1 c2 c3 c4]; % (!) must change accordingly if changing d_k

            syms t_sym N_sym
            %C_i = zeros(d_k+1,N+1); % matrix to construct cost Q_i(T_i)-matrices
            for k=0:d_k % order of derivative
            for col=1:N+1 % polynomial coeff.
                if (N-(col-1)-k) >= 0
                    C_i(k+1,col)=sqrt(c_vec(k+1))*faculty_factor(N_sym-(col-1),k)*t_sym^(N_sym-(col-1)-k);
                end
            end
            end

            N_sym = N; %replace with numerical value for polynomial order
            C_i = subs(C_i);
            Q_i_sym = int(C_i.'*C_i,t_sym); % quadratic cost matrix for one time

            % symbolic matrix, for creating derivatives:
            syms t_sym
            for k=0:d_k+1 % (!) this must be same order as desired output functions 
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_seg_sym(k+1,col)=faculty_factor(N-(col-1),k)*t_sym^(N-(col-1)-k);
                end
            end
            end
            % ! check: should have k zeros in last k row

            % As steering function (only one segment):
            t_sym = T_list(1);
            t_sym = T_mid_list(1); % should be t_eps
            Q_i_store_sym = Q_i_sym; %store symbolic matrix
            Q_i_num_lower = subs(Q_i_sym); %replace with numerical values for time
            Q_i_sym = Q_i_store_sym; % retrieve back symbolic matrix
            t_sym = T_list(end); %evaluate at upper time limit
            Q_i_num_upper = subs(Q_i_sym);
            Q_i_num_tot = Q_i_num_upper-Q_i_num_lower; % sum upper - lower
            Q_tot = Q_i_num_tot;

            % both x and y optimization:
            Q_tot_xy = kron(eye(2),Q_tot);
            A_full_xy = kron(eye(2),A_full);
            d_tot_xy = [d_tot_x ; d_tot_y];
            d_size = size(d_tot_xy);

            
            A_speed = zeros(M_mid,M*(N+1));
            A_acc = zeros(M_mid,M*(N+1)); % approx. curvature constraint
            %take into account last time point, but not first (given by i.c.)

            
            %A_curv = []; % curvature constraint no longer enforced at start point
            for i=1:M_mid
                vec = zeros(M_mid,1);
                vec(i) = 1;
                if i==M_mid % add last time point
                    A_speed = A_speed + kron(vec,A_last(2,:));
                    A_acc = A_acc + kron(vec,A_last(3,:));
                else % take middle points
                    A_speed = A_speed + kron(vec,A_seg(2,:,i));
                    A_acc = A_acc + kron(vec,A_seg(3,:,i)); 
                end
            end
              
            %size(A_speed)
            A_speed_xy = kron(eye(2),A_speed);
            A_acc_xy = kron(eye(2),A_acc);
            d_speed = v_max*ones(2*M_mid,1); %in both x and y (!) note v_max is already divided here by sqrt(2), not the same v_max as in test_steer!
            d_speed_l = -d_speed;
            
            % decreasing
            % fix when lower speed bound greater than upper !!!!!
            d_speed_u = v_max*ones(2*M_mid,1);
            d_speed_l = -v_max*ones(2*M_mid,1);
                      
            %% Setup optimization problem:
            %% TODO: 
            % 1. warm-start
            % 2. objective function
            % 3.
            % 4. 

            % using OSQP:
            % min (1/2)*x'Px + q^T x
            % s.t. l <= Ax <= u
            P = sparse(double(2*Q_tot_xy));
            q = zeros(size(Q_tot_xy,1),1);
            
%             A = [A_full_xy ; A_speed_xy ; A_acc_xy ; A_curv];
%             l = [d_tot_xy ; d_speed_l ; d_acc_l ; curv_acc_l];
%             u = [d_tot_xy ; d_speed_u ; d_acc_u ; curv_acc_u];
            
            d_speed_lower_u = Inf*ones(size(d_speed_lower,1),1);
            
            A = [A_full_xy ; A_speed_lower];% ; A_curv_xy];
            l = [d_tot_xy ; d_speed_lower];% ; d_curv_l];
            u = [d_tot_xy ; d_speed_lower_u];% ; d_curv_u];
            
            A = [A_full_xy ; A_speed_lower ; A_curv_xy];
            l = [d_tot_xy ; d_speed_lower ; d_curv_l];
            u = [d_tot_xy ; d_speed_lower_u ; d_curv_u];
            
            % add upper speed constraint, may not be necessary
            A = [A ; A_speed_xy];
            l = [l ; d_speed_l];
            u = [u ; d_speed_u];

            size(A);
            size(l);
            size(u);
            % A = A_full_xy;
            % l = d_tot_xy;
            % u = l;

            % Create an OSQP object
            prob = osqp;
            
            % 'verbose',False - disables print output
            
            try % handle if problem setup fails, can be non-convex
            % Setup workspace and change alpha parameter
                prob.setup(P, q, A, l, u,'alpha',1.6,'time_limit',10e-3)%,'verbose',0);%,'eps_prim_inf',1);
            % higher alpha seems to give faster convergence: 0 < alpha < 2,
            % default is 1.6
            catch
               %disp('Problem setup failed')
               x = NaN; y = NaN; T = NaN; solverStatus = 'NaN';
               dist = NaN_distance;
               distance_list = dist; % added to prevent error
               distance_list_map(num2str(state_couple)) = distance_list;
               dist_list(state1_number) = dist;
               state_matrix = [state1 ; NaN*ones(1,numStateVariables)];
               state_matrix_map(num2str(state_couple))=state_matrix;
               continue
            end    
            % warm-start, x = primal variables, y = dual variables = lambdas
            %prob.warm_start('x', x0, 'y', y0)
            
            %% warm-start: N=6: set a1 and a2 to zero, a5,a6,a7 given by b.c.
            % (!) must change if N is changed! must fix more coeff.
            A_matrix = [0 0 0 0 0 0 1;
                        0 0 0 0 0 1 0;
                        0 0 0 0 2 0 0;
                        T^6 T^5 T^4 T^3 T^2 T 1;
                        6*T^5 5*T^4 4*T^3 3*T^2 2*T 1 0;
                        1 0 0 0 0 0 0;
                        0 1 0 0 0 0 0];
            A_matrix = kron(eye(2),A_matrix);
            RHS = [pos_i(1);vel_i(1);acc_i(1);pos_end(1);vel_end(1); acc_end(1); 0;
                   pos_i(2);vel_i(2);acc_i(2);pos_end(2);vel_end(2); acc_end(2); 0];
            
            p0 = A_matrix\RHS; % solve for 6 degrees, then just add two zeroes to expand to 7 degree, quick fix
            
            if N == 7
                p0 = [0 ; p0(1:7); 0; p0(8:end)]; % use if N = 7
            end
            
            prob.warm_start('x',p0);
            
            % Solve problem
            res = prob.solve();
            solverStatus = res.info.status; %check solver status
            X = res.x;
            
            %return;
            % The inaccurate statuses define when the optimality, primal infeasibility
            % or dual infeasibility conditions are satisfied 
            % with tolerances 10 times larger than the ones set.

            %% construct polynomials from coefficients
            clear t_vec
            syms t_sym
            for i=1:N+1
                t_vec(i)=t_sym^(N-(i-1));
            end
            %t_list = linspace(0,T,M*100+1);
            n_div = 10; %no. of time division for plotting,
            t_lists = zeros(M,n_div);
            t0 = t_first; %before zero, now t_eps w/ initial straight, if needed
            for i=1:M %create list of times
                if i==1
                    t_lists(i,:) = linspace(t0,T_list(2),n_div);
                else
                    t_lists(i,:) = linspace(T_list(i),T_list(i+1),n_div);
                end
            end

            X_x = X(1:size(A_full,2)); X_y = X(size(A_full,2)+1:end);
            X_x_matrix = reshape(X_x,[N+1,M]);
            X_y_matrix = reshape(X_y,[N+1,M]);

            %% construct polynomials:
            % (derivative,segment) = (d_k+1,M)
            der_x_sym = A_seg_sym*X_x_matrix;
            der_y_sym = A_seg_sym*X_y_matrix;

            x = cell(d_k+1,M); % cell of polynom functions
            y = cell(d_k+1,M); % cell of polynom functions
            der_order=3; % need jerk (order = 3) for u2 input
            for i=1:der_order+1
                for j=1:M
                    x{i,j}=@(t) subs(der_x_sym(i,j),t_sym,t);
                    y{i,j}=@(t) subs(der_y_sym(i,j),t_sym,t);
                end
            end
            
             % store polynomials in maps w/ corresponding states
             if strcmp(solverStatus,'solved') == 1 %|| strcmp(solverStatus,'solved inaccurate') == 1
                 poly_map_x(num2str(state_couple)) = x;
                 poly_map_y(num2str(state_couple)) = y;
                 time_map_T(num2str(state_couple)) = T;
                 
             else
                 % do nothing, solver has failed
             end
            
            % obtain function outputs:
            %fraction = 100; % (!) this overwrites the fraction input, must use state_matrix from here in interpolate function
            N_intervals = 100; % can change later
            %N_intervals = fraction;
            
            
            
            % note: t0 = t_eps (!)
            % create time list for obtain state values
            t0 = 0; T_f = T;
%             if slow_start
%                 t0 = t_i_eps;
%             end
%             if slow_end
%                 T_f = T-t_f_eps;
%             end
            
            Time_list2 = linspace(t0,T_f,N_intervals); % input to simulation
            
            if strcmp(solverStatus,'solved') == 1 %|| strcmp(solverStatus,'solved inaccurate') == 1
            
            alpha_i = state1(3); beta_i = pi+alpha_i; % along x-axis is pi, 180+(alpha_i) = same as truck
            
            simulate = false;
            if simulate
                [u1_vec,u2_vec] = get_Inputs_and_Plot(x,y,Time_list2,alpha_i,beta_i);
            end
            
            Time_list = linspace(0,T,N_intervals); % input to simulation
            
            if reversing % invert time_list and change sign of speeds!
                Time_list = fliplr(Time_list);
                theta = double(atan2(y{2,1}(Time_list),x{2,1}(Time_list)));
                vel_x = double(-x{2,1}(Time_list));
                vel_y = double(-y{2,1}(Time_list));
                acc_x = double(-x{3,1}(Time_list));
                acc_y = double(-y{3,1}(Time_list));
                start_path_new = flip(end_path);
                end_path_new = flip(start_path);
                start_path = start_path_new;
                start_path(:,4) = -start_path(:,4); % flip sign of speed
                end_path = end_path_new;
                end_path(:,4) = -end_path(:,4); % flip sign of speed
                a_start = -a0; a_end = a0; % flip sign of accel. this is to convert back to reversing
                % switch slow start and slow end:
                slow_start_new = slow_end;
                slow_end_new = slow_start;
                slow_start = slow_start_new;
                slow_end = slow_end_new;
            else % do nothing
                theta = double(atan2(y{2,1}(Time_list),x{2,1}(Time_list)));
                vel_x = double(x{2,1}(Time_list));
                vel_y = double(y{2,1}(Time_list));
                acc_x = double(x{3,1}(Time_list));
                acc_y = double(y{3,1}(Time_list));
                a_start = a0; a_end = -a0;
            end
            
            pos_x = double(vpa(x{1,1}(Time_list)))'; 
            pos_y = double(vpa(y{1,1}(Time_list)))';
            acc_x = acc_x'; acc_y = acc_y';
            
            % define the signed speed function
            u1 = @(vel_x,vel_y,alpha) ((vel_x ~= 0).*sign(vel_x.*cos(alpha)) + (vel_x == 0).*sign(vel_y.*sin(alpha))).*sqrt(vel_x.^2+vel_y.^2); % signed velocity indicating forward or reverse motion

            vel = u1(vel_x,vel_y,theta)';
            theta = theta';
            
            
            
            if slow_start
                pos_x = [start_path(:,1) ; pos_x]; 
                pos_y = [start_path(:,2) ; pos_y]; 
                theta = [start_path(:,3) ; theta];
                vel = [start_path(:,4) ; vel]; 
                acc_x = [a_start*cos(start_path(:,3)) ; acc_x];
                acc_y = [a_start*sin(start_path(:,3)) ; acc_y];
            end
            if slow_end
                pos_x = [pos_x ; end_path(:,1)]; 
                pos_y = [pos_y ; end_path(:,2)];
                theta = [theta ; end_path(:,3)];
                acc_x = [acc_x ; a_end*cos(end_path(:,3))];
                acc_y = [acc_y ; a_end*sin(end_path(:,3))];
                vel = [vel ; end_path(:,4)]; 
            end
            
           
                    
            
            % outputs matrix of interpolated states along polynomial
            % segment
            global numStateVariables
            global states_in_tree
            
            % curvature function:
            kappa = @(x_vel,y_vel,x_acc,y_acc) (y_acc.*x_vel-y_vel.*x_acc)./(x_vel.^2+y_vel.^2).^(3/2);
            
            
            vel_x = vel.*cos(theta);
            vel_y = vel.*sin(theta);
            
            % compute curvature:
            curv = kappa(vel_x,vel_y,acc_x,acc_y);
            disp('CURVMAX')
            curv_max = double(vpa(max(abs(curv))))
            else
                % do nothing, solver failed.
            end
            
            do_not_store = false; %reset  
            show_plot = false;
            % set path to infeasible if solver fails, prevents crash
            %if any(isnan(X), 'all')  %set path infeasible (NaN) if solver fails
            if strcmp(solverStatus,'solved') == 1 %|| strcmp(solverStatus,'solved inaccurate') == 1
               
               state_matrix = double([pos_x,pos_y,theta,vel]); % w/o accel.
               
               % compute curvature: if ok, then continue, otherwise
               % discard the path and set to nan's
               vel = reshape(vel,max(size(vel)),1); theta = reshape(theta,max(size(theta)),1);
               
               show_plot = false; % set true to plot when solved
               if curv_max > K_max
                   state_matrix = [state1(1:4); NaN*ones(length(X_x)-1,numStateVariables)];
                   failed_list = [failed_list ; [state1(1:4), state2(1:4)]];
                   state_matrix_map(num2str(state_couple)) = state_matrix; % store
                   do_not_store = true;
                   show_plot = false;
               end
               if show_plot
                   legendFS = 22; axFS = 24;
                   figure()
                   hold on;
                   show(pathSegObj2{1})
                   plot(pos_x,pos_y,'DisplayName',strcat('Polynomial : ',' $\kappa_{max} = \;$',num2str(round(curv_max,2))),'Color','b')
                   hold off;
                   legend('FontSize',legendFS);
                   ax = gca;
                   ax.FontSize = axFS; 
                   title('Trajectory','FontSize',axFS)
                   xlabel('x [m]','FontSize',axFS)
                   ylabel('y [m]','FontSize',axFS)
                   axis equal
               end
               solved_list = [solved_list ; [state1(1:4), state2(1:4)]];
            else % solver has failed to find solution
               state_matrix = [state1; NaN*ones(length(X_x)-1,numStateVariables)];
               %failed_list = [failed_list ; [state1(1:4), state2(1:4)]];
            end
            
            solverTime = res.info.run_time; % return solver time also
            solverTime_list = [solverTime_list,solverTime];
            
            

            %%%%%%% Obtain polynomials and calculate distance of
            %%%%%%% curve
            
            % Default behavior: Calculate the Euclidean 2-norm between each pair of
            % state1 and state2 rows
            % (!) added: only take position, not velocity
            distance_map = containers.Map(); % create new map
            distance_list = zeros(size(state_matrix,1)-1,1);
            total_dist = 0;
            distance_list = NaN_distance; % default value, to prevent error

            
            for i=1:size(state_matrix,1)-1
                first_state = state_matrix(i,:);
                second_state = state_matrix(i+1,:);
                stateDiff = bsxfun(@minus, first_state(1:2), second_state(1:2));
                dist = sqrt( sum( stateDiff.^2, 2 ) );
                total_dist = total_dist+dist;
                % store distance along with state in map
                distance_map(num2str(total_dist))=second_state;
                distance_list(i)=total_dist;
            end
            % store distance_list in map:
            distance_list_map(num2str(state_couple))=distance_list;
            distance_map_Map(num2str(state_couple))=distance_map;
            dist = total_dist; %return distance
            
            if strcmp(solverStatus,'solved')
                solved_time_list = [solved_time_list,solverTime];
            end
            show_plot = true; % store even if not showing plot
            if ~do_not_store
                slow_dist = norm(start_path(1,1:2)-start_path(end,1:2))+norm(end_path(1,1:2)-end_path(end,1:2));
                poly_travel = [poly_travel,dist-slow_dist];
                dubin_travel = [dubin_travel,dubin_dist2];
            end
            
            if isnan(dist) % set dist to very large, make it larger than some threshold for the interpolate function
                dist = NaN_distance;
                NaN_state = NaN*ones(1,numStateVariables);
                if do_not_store
                    % do not change state couple
                else
                    state_couple = [state_couple(1,1:4) ; NaN_state];
                end
                solved_dist_map(num2str(state_couple))= dist;
                if do_not_store
                    % do not store, do nothing, already stored above, but
                    % store distance, will prevent solver from running
                    % again  
                else
                    state_matrix_map(num2str(state_couple)) = state_matrix;
                end
            else
                %state_matrix_final = [state_matrix_final,state_matrix];
                solved_dist_map(num2str(state_couple)) = dist;
                state_matrix_map(num2str(state_couple)) = state_matrix;
            end
            
            % store distance in distance list: list will be scalar if only
            % one state1

            dist_list(state1_number) = dist;
            
            end % end of state1-loop
            
            %disp('return distance')
            
           
            dist = dist_list; % distance to return
            
            end
end