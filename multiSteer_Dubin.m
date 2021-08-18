% steering function:
function [dist,x,y,T,t_eps,solverStatus] = multiSteer_Dubin(obj,state1,state2,N)
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
            
            global poly_map_x poly_map_y time_map_T % to store polynomials
            
            narginchk(4,4);
            
            x=0;
            y=0;
            T=0; 
            t_eps=0;
            solverStatus = 0;
            
            % create distance map for finding the state along path at
            % max.connection distance
            
            
            
            %disp('DISTANCE')
            
            state1 = double(state1);
            state2 = double(state2);
            
            NaN_distance = 1e6;
            
            state1_matrix = state1; %created for clarity
            
            % if state1 is matrix, function will be used in near() -->
            % avoid calling steer-function twice:
            if size(state1_matrix,1)>1 
                %disp("!!!!!MATRIX!!!!!")
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

            if norm(state1(1:2)-state2(1:2)) > 1.5*maxConnDist
                dist_list(state1_number) = NaN_distance;
                if state1_number == no_of_state1
                    dist = dist_list;
                    return
                end
                continue
            end
            % discard if angles are too far apart
            if abs(state1(3)-state2(3)) > 90*pi/180
                dist_list(state1_number) = NaN_distance;
                if state1_number == no_of_state1
                    dist = dist_list;
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
            if sign(v1) <= 0 && sign(v2) <= 0
                % reversing action: reverse order of state1 and state2
                state1_old = state1;
                state2_old = state2;
                state1 = state2_old;
                state2 = state1_old;
                % change sign of speeds:
                state1(4) = -state2_old(4);
                state2(4) = -state1_old(4);
                reversing = true;
                
            elseif sign(v1) >= 0 && sign(v2) >= 0
                % do nothing, standard procedure
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
            
            if reversing %(state2 is now state1 --> check for that value in map)
                try
                    accel_end = double(accel_map(num2str(state2)));
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
                    accel_i = double(accel_map(num2str(state1)));
                    %disp('ACC FOUND')
                catch
                    accel_i = [0,0]; %set to zero if no matching value is found (initial point)
                    %disp('acc not found');
                end
            end
           
%            [state1, state2] = obj.validateDistanceInput(obj.NumStateVariables, state1, state2);
            
            %%%%%%%%%% INSERT HERE %%%%%%%%%%%%%%%%%
            function_counter = function_counter+1; % only count function calls to solver
            
            %% initial/end conditions:
            %max speed, m/s
%            v_max = obj.StateBounds(4,2);
            v_max = (50/3.6)/sqrt(2);
            %v_i = v_max/2; % initial speed
            v_i = state1(4);
            %v_i = speed_i; % passed along from solver
            %v_end = v_max/2; % end speed
            v_end = state2(4);
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

            % initial % RRT map is from [0 2.6], so scale w/ some factor  
            pos_i = [state1(1),state1(2)];
            vel_i = [v_i*cos(alpha_i),v_i*sin(alpha_i)]; 
            acc_i = [state1(5),state1(6)]; % using sampled acceleration
            
            % end:
            pos_end = [state2(1),state2(2)];
            vel_end = [v_end*cos(alpha_end),v_end*sin(alpha_end)]; 
            acc_end = [state2(5),state2(6)]; 
           

            % RHS in opt.problem. (!) can add acc as sampled
            sample_acc = true;
            if sample_acc == true 
                d_init_x = [pos_i(1) ; vel_i(1) ; acc_i(1)];
                d_end_x = [pos_end(1) ; vel_end(1) ; acc_end(1)];
                d_init_y = [pos_i(2) ; vel_i(2) ; acc_i(2)];
                d_end_y = [pos_end(2) ; vel_end(2); acc_end(2)];
            else
                if reversing % satsify end accel, but not initial!
                    d_init_x = [pos_i(1) ; vel_i(1)];
                    d_end_x = [pos_end(1) ; vel_end(1) ; acc_end(1)];
                    d_init_y = [pos_i(2) ; vel_i(2)];
                    d_end_y = [pos_end(2) ; vel_end(2) ; acc_end(2)];
                else
                    d_init_x = [pos_i(1) ; vel_i(1) ; acc_i(1)];
                    d_end_x = [pos_end(1) ; vel_end(1)];
                    d_init_y = [pos_i(2) ; vel_i(2) ; acc_i(2)];
                    d_end_y = [pos_end(2) ; vel_end(2)];
                end
            end

            % Euclidian distance:
            dist_tot = norm(pos_i-pos_end);
            global state_matrix_final
%             if dist_tot == 0
%                 state_matrix_final = state_matrix; %store final state_matrix
%             end
            
            %% Time allocation:
            % perform several initial guesses and then use gradient descent to find
            % optimal time allocation based on cost function


            M = 3; % no. of segments
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
           toc
           
           dubinObj = pathSegObj{1};
           dubinObj.MotionLengths
           seg1 = dubinObj.MotionLengths(1);
           seg2 = dubinObj.MotionLengths(2);
           seg3 = dubinObj.MotionLengths(3);
           % take intermediate Dubin points:
           % interpolate along dubin path:
           dubin_dist = pathSegObj{1}.Length;
           
           speed = v_max/2;
           % create time points for dubin points
           t_dubin1 = seg1/speed;
           t_dubin2 = (seg1+seg2)/speed;
           
           
%            t_dubin05 = 0.5*seg1/speed;
%            t_dubin15 = (seg1+0.5*seg2)/speed;
%            t_dubin25 = (seg1+seg2+0.5*seg3)/speed;
           
           T_list(2) = seg1/speed;
           T_list(3) = (seg1+seg2)/speed;
           T_list(4) = T_list(3) + seg3/speed;
           
           T = T_list(end); % final 
           
           
           no_poses = 12; % no of intermediate points
           T_pose_list = linspace(0,T,no_poses+2);
           T_pose_list = T_pose_list(2:end-1);
           %no_poses = length(T_pose_list);

           % step along equal step length (0:step_length:tot_dist)
           step_length = dubin_dist/(no_poses+1-2);
           poses = interpolate(pathSegObj{1},0:step_length:dubin_dist);
           dubin_theta_list = poses(2:end-1,end);
           
           dubin_pt1 = [poses(round(seg1/step_length),1),poses(round(seg1/step_length),2)];
           dubin_pt2 = [poses(round((seg1+seg2)/step_length),1),poses(round((seg1+seg2)/step_length),2)];
           
           dubin_pt05 = [poses(round(0.5*seg1/step_length),1),poses(round(0.5*seg1/step_length),2)];
           dubin_pt15 = [poses(round((seg1+0.5*seg2)/step_length),1),poses(round((seg1+0.5*seg2)/step_length),2)];
           dubin_pt25 = [poses(round((seg1+seg2+0.5*seg3)/step_length),1),poses(round((seg1+seg2+0.5*seg3)/step_length),2)];
           
           % take out dubin_headings to enforce lower speed bounds:
           dubin_theta_05 = poses(round(0.5*seg1/step_length),3);
           dubin_theta_1 = poses(round((seg1+seg2)/step_length),3);
           dubin_theta_15 = poses(round((seg1+0.5*seg2)/step_length),3);
           dubin_theta_2 = poses(round((seg1+seg2)/step_length),3);
           dubin_theta_25 = poses(round((seg1+seg2+0.5*seg3)/step_length),3);
           
           %dubin_theta_list = [dubin_theta_05; dubin_theta_1; dubin_theta_15 ; dubin_theta_2 ; dubin_theta_25];
                   
           
           % add extra middle time points:
           
            %show(pathSegObj{1})

            %No_runs = 1; %no of runs of optimization 
               
            % take speed as average of the two sampled speeds
            
            v_eps = 3; % 2 = walking speed - gives +- 0.7 m/s^2 bounds in perp. acceleration
            %a0 = acc_max; % take max acc, change if needed
            
            % default start/end accelerations if speed too slow at
            % start/end:
            a_start = sign(v_i)*acc_max; % neg. accel if reversing (back up faster)
            a_end = -sign(v_end)*acc_max; % neg. accel if reversing (back up faster)    
            
            
            t_eps = abs(abs(v_end)-v_eps)/abs(a_start);
            t_f_eps = T-abs(abs(v_end)-v_eps)/abs(a_end);
            
            
            %%
            %% MIT - Polynomial trajectory generation with QP

            syms t
            no_dubin_pts = 2;
            %M = 4; % no of path segments
            M = no_dubin_pts+1; % no. of segments = #RRT-points
            N = 5; % order of polynomials --> N+1 coefficients, 
            d_k = 3; % order of derivative, jerk (d_k=3) needed for u2 input, (!) not continuity, only for creating matrices
            % d_k is used for the multi-segment continuity
            % k = 
            % 0: pos
            % 1: vel
            % 2: acc
            % 3: jerk
            % 4: snap
            % 5: crackle
            % 6: pop
            
           
           A_dubin1 = zeros(1,N+1);
           A_dubin2 = zeros(1,N+1);
           
           % continuity constraints:
           faculty_factor = @(N,k) factorial(N)/factorial(N-k);
           
           k=0; % order of derivative, take position
           for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_dubin1(1,col)=faculty_factor(N-(col-1),k)*t_dubin1^(N-(col-1)-k);
                    A_dubin2(1,col)=faculty_factor(N-(col-1),k)*t_dubin2^(N-(col-1)-k);
                end
           end
            
            seg_i = 0; 
            A_seg = zeros(d_k+1,N+1,M-1); % matrix of segment times coefficients
            % (derivative,coefficient,time point)
            for time_point=2:length(T_list)-1 %time point
            t = T_list(time_point);
            seg_i=seg_i+1;
            for k=0:d_k %order of derivative
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_seg(k+1,col,seg_i)=faculty_factor(N-(col-1),k)*t^(N-(col-1)-k);
                end
            end
            end
            end
            
            seg_i = 0; %reset
            A_seg_interm = zeros(d_k+1,N+1,M); % matrix of interm. segment times coefficients, add one in b/w each segment
            for time_point=2:length(T_list) %time point, now include last time point compared to above
            t = T_list(time_point)/2; %take middle time points, so divide by 2 compared to above
            seg_i=seg_i+1;
            for k=0:d_k %order of derivative
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_seg_interm(k+1,col,seg_i)=faculty_factor(N-(col-1),k)*t^(N-(col-1)-k);
                end
            end
            end
            end
            

            % symbolic matrix, for creating derivatives:
            syms t_sym
            for k=0:d_k %order of derivative
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_seg_sym(k+1,col)=faculty_factor(N-(col-1),k)*t_sym^(N-(col-1)-k);
                end
            end
            end
            % ! check: should have k zeros in last k row

            %% Construct mid-point continuity matrices
            t_index=0;
            A_tot = zeros((M-1)*(d_k+1),(M-1)*(N+1));
            no_time_points = M+1;
            for t=1:no_time_points-2 %time points, leave out first/last equation, and add later
                t_index=t_index+1;
                vec = zeros(M-1,1);
                vec2 = zeros(M-2,1);
                vec(t_index)=1;
                if t_index < no_time_points-2 % upper matrix must be one A-dim smaller
                    vec2(t_index)=1;
                end
                diag_matrix = diag(vec);
                upper_diag_matrix = diag(vec2,1);
                A_t = A_seg(:,:,t_index);
                %add = kron(upper_diag_matrix,A_t)
                A_tot = A_tot + kron(diag_matrix,A_t)+kron(upper_diag_matrix,-A_t);
            end
            % now add last column: 
            A_last_col = kron([zeros((M-2),1);1],-A_seg(:,:,end));
            A_tot = [A_tot,A_last_col];


            t_first = 0;
            t_last = T;
            A_first = zeros(d_k+1,N+1); % first and last time point matrices w/ all relevant derivatives
            A_last = zeros(d_k+1,N+1);
            for k=0:d_k %order of derivative to satisfy at initial and end point
            for col=1:N+1 %polynomial coeff.
                if (N-(col-1)-k) >= 0
                    A_first(k+1,col)=faculty_factor(N-(col-1),k)*t_first^(N-(col-1)-k);
                    A_last(k+1,col)=faculty_factor(N-(col-1),k)*t_last^(N-(col-1)-k);
                end
            end
            end
            % define what derivatives should be fixed at start and goal
            d_k_fix = 2;
            A_init = kron([1,zeros(1,M-1)],A_first(1:d_k_fix+1,:));
            A_end = kron([zeros(1,M-1),1],A_last(1:d_k_fix+1,:));

            A_full = [A_init; A_end ; A_tot]; % full constraint matrix
            d_free = zeros(size(A_tot,1),1); % d_free=d_p in MIT paper (free constraints)
            d_fixed_x = [d_init_x ; d_end_x]; % vector of fixed constraints in x
            d_tot_x = [d_fixed_x ; d_free];
            d_fixed_y = [d_init_y ; d_end_y]; % vector of fixed constraints in y
            d_tot_y = [d_fixed_y ; d_free];
            
            % the above only includes initial/end constraints and
            % continuity at middle points, add dubin further down

            size(A_full)
            
            
            
            %% Lower speed bound constraint
           % create Dubin A-matrix list from seg and seg_interm 
%            j = 0; k = 0;
%            for i=1:length(dubin_theta_list)
%                if mod(i,2) == 1 % odd number, take interm points
%                    j = j+1;
%                    A_Dubin_tot(:,:,i) = A_seg_interm(:,:,j);
%                else % even, take seg points
%                    k = k+1;
%                    A_Dubin_tot(:,:,i) = A_seg(:,:,k);
%                end
%            end
%            pt = 0; T_tot_list = [];
%            T_half_list = T_list(2:end)/2;
           
           %T_tot_list = sort([T_list ; T_half_list]);
           T_tot_list = T_pose_list; % from Dubin poses to take out angles
           
           % intermediate pose matrices, uniformly divided on whole path
           seg_i = 0; %reset
           A_poses = zeros(d_k+1,N+1,M); % matrix of interm. segment times coefficients, add one in b/w each segment
           for time_point=1:length(T_pose_list) %time point, now include last time point compared to above
            t = T_pose_list(time_point); 
            seg_i=seg_i+1;
            for k=0:d_k %order of derivative
                for col=1:N+1 %polynomial coeff.
                    if (N-(col-1)-k) >= 0
                        A_poses(k+1,col,seg_i)=faculty_factor(N-(col-1),k)*t^(N-(col-1)-k);
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
               % check which segment to enforce constraint on
               if time <= T_list(2)
                   segment = 1;
               elseif T_list(2) < time && time <= T_list(3)
                   segment = 2;
               elseif T_list(3) < time && time <= T_list(4)
                   segment = 3;
               end
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
               

               vec = zeros(1,M);
               vec(1,segment) = 1;
               A_x = kron(vec,A_x); A_y = kron(vec,A_y);
               A_x_curv = kron(vec,A_poses(3,:,pt)); A_y_curv = kron(vec,A_poses(3,:,pt));
               
               A_speed_lower = [A_speed_lower ; A_x , A_y];
               A_curv_xy = [A_curv_xy ; A_x_curv , A_y_curv];
           end % end of theta loop
           v_lower = 3; % lower speed bound, should ideally be same as v_eps
           d_speed_lower = v_lower*ones(size(A_speed_lower,1),1);
           
           min_speed = 3;
           acc_max = K_max*min_speed^2;
           %acc_max = 2; % 2 m/s^2 in both long. and lateral acc. is default bounds
%            A_acc_xy = kron(eye(2),A_acc);
%            d_acc_l = -acc_max*ones(size(A_acc_xy,1),1);
%            d_acc_u = acc_max*ones(size(A_acc_xy,1),1);
           d_curv_l = -acc_max*ones(size(A_curv_xy,1),1);
           d_curv_u = acc_max*ones(size(A_curv_xy,1),1);
            
            %% Objective function:
            % costs for each derivative of position, 0 = position
            k_T = 1; % time-penalty
            c0 = 0; % pos
            c1 = 0; % speed
            c2 = 1; % acc
            c3 = 0; % jerk
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

            Q_tot = zeros(M*(N+1),M*(N+1)); % full matrix with all costs for all segments
            % evaluate numerically for each segment time:
            for t_index=1:length(T_list)-1
                Q_i_store_sym = Q_i_sym; %store symbolic matrix
                t_sym = T_list(t_index); 
                Q_i_num_lower = subs(Q_i_sym); %replace with numerical values for time
                Q_i_sym = Q_i_store_sym; % retrieve back symbolic matrix
                t_sym = T_list(t_index+1); %evaluate at upper time limit
                Q_i_num_upper = subs(Q_i_sym);
                Q_i_num_tot = Q_i_num_upper-Q_i_num_lower; % sum upper - lower
                vec3 = zeros(length(T_list)-1,1);
                vec3(t_index) = 1;
                diag_m = diag(vec3);
                Q_tot = Q_tot + kron(diag_m,Q_i_num_tot); % put each matrix on diagonal
            end 

            % both x and y optimization:
            Q_tot_xy = kron(eye(2),Q_tot);
            A_full_xy = kron(eye(2),A_full);
            d_tot_xy = [d_tot_x ; d_tot_y];

            %% Add Dubin constraints
            A_dubin_x = zeros(M-1,(M-1)*(N+1));
            for i=1:M-1
                vec = zeros(1,M-1);
                vec(i) = 1;
                diag_m = diag(vec);
                A_dubin_x = A_dubin_x + kron(diag_m,A_seg(1,:,i));
            end
            % Add intermediate Dubin points: % one more segment than above
            A_dubin_x_interm = zeros(M,(M)*(N+1));
            for i=1:M
                vec = zeros(1,M);
                vec(i) = 1;
                diag_m = diag(vec);
                A_dubin_x_interm = A_dubin_x_interm + kron(diag_m,A_seg_interm(1,:,i));
            end
            
            A_dubin_x = [A_dubin_x , zeros(size(A_dubin_x,1),N+1)];
            A_dubin_xy = kron(eye(2),A_dubin_x);
            
            A_dubin_interm_xy = kron(eye(2),A_dubin_x_interm);
            
            A_full_Dubin = [A_full_xy];% ; A_dubin_xy ; A_dubin_interm_xy];
            dubin_interm_x = [dubin_pt05(1) ; dubin_pt15(1) ; dubin_pt25(1)];
            dubin_interm_y = [dubin_pt05(2) ; dubin_pt15(2) ; dubin_pt25(2)];
            RHS_dubin_interm = [dubin_interm_x ; dubin_interm_y];
            
            RHS_dubin_x = [dubin_pt1(1) ; dubin_pt2(1)]; % x-pos
            RHS_dubin_y = [dubin_pt1(2) ; dubin_pt2(2)]; % y-pos
            
            % use instead w/ margins to dubin points
            margin = 2;
            d_tot_Dubin_l = [d_tot_xy];% ; RHS_dubin_x-margin ; RHS_dubin_y-margin ; RHS_dubin_interm-margin];
            d_tot_Dubin_u = [d_tot_xy];% ; RHS_dubin_x+margin ; RHS_dubin_y+margin ; RHS_dubin_interm+margin];

            %% Add input constraints:
            A_speed = zeros(M-1,(M-1)*(N+1)); 
            A_acc = zeros(M-1,(M-1)*(N+1)); 
            A_curv = zeros(M-1,2*(M-1)*(N+1)); % will contain all waypoints, will be double of columns as A_acc and A_speed
            
            A_speed_interm = zeros(M,(M)*(N+1)); 
            A_acc_interm = zeros(M,(M)*(N+1)); 
            A_curv_interm = zeros(M,2*(M)*(N+1)); % M since taking one more time point (and polynomial) than above
            
            % (!) only enforce constraints at Dubin points (M-1 st), since
            % endpoints are fixed already, can add more if you want
            for i=1:M-1
                A_curv_add = []; % matrix for one waypoint, to be added to the full A_curv matrix.
                vec = zeros(1,M-1);
                vec(i) = 1;
                diag_m = diag(vec);
                A_speed = A_speed + kron(diag_m,A_seg(2,:,i));
                A_acc = A_acc + kron(diag_m,A_seg(3,:,i)); 
            end
            % add last column of zeros to fix dimensions (number of polys considered):
            A_speed = [A_speed , zeros(size(A_speed,1),N+1)];
            A_acc = [A_acc , zeros(size(A_acc,1),N+1)];
            
            % now add intermediate time point constraints:
            for i=1:M
                A_curv_add = []; % matrix for one waypoint, to be added to the full A_curv matrix.
                vec = zeros(1,M);
                vec(i) = 1;
                diag_m = diag(vec);
                A_speed_interm = A_speed_interm + kron(diag_m,A_seg_interm(2,:,i));
                A_acc_interm = A_acc_interm + kron(diag_m,A_seg_interm(3,:,i)); 
            end
            
            % add interm constraints to the matrices
            A_speed = [A_speed ; A_speed_interm];
            A_acc = [A_acc ; A_acc_interm];

            A_speed_xy = kron(eye(2),A_speed);
            d_speed = v_max/sqrt(2)*ones(size(A_speed_xy,1),1);
            d_speed_l = -d_speed;
            d_speed_u = d_speed;
            
            % curv. constraints: use acc. instead
%             min_speed = 2; % walking speed, will be very conservative, as low as possible since want speed to be above this limit...
%             curv_acc_l = -K_max*min_speed^2*ones(size(A_curv,1),1); %using speed and angle
%             curv_acc_u = K_max*min_speed^2*ones(size(A_curv,1),1);


            %% Setup optimization problem:
            %% TODO: 
            % 1. warm-start - not really needed, very hard 
            % 2. objective function - what costs?
            % 3. Time allocation - gradient descent?

            % using OSQP:
            % min (1/2)*x'Px + q^T x
            % s.t. l <= Ax <= u

            P = sparse(double(2*Q_tot_xy));
            q = zeros(size(Q_tot_xy,1),1);
            
%             A = [A_full_Dubin ; A_speed_xy ; A_acc_xy];% ; A_curv];
%             l = [d_tot_Dubin_l ; d_speed_l ; d_acc_l];% ; curv_acc_l];
%             u = [d_tot_Dubin_u ; d_speed_u ; d_acc_u];% ; curv_acc_u];
            
            d_speed_lower_u = Inf*ones(size(d_speed_lower,1),1);
            
            A = [A_full_Dubin ; A_speed_lower ; A_curv_xy];% ; A_curv];
            l = [d_tot_Dubin_l ; d_speed_lower ; d_curv_l];% ; curv_acc_l];
            u = [d_tot_Dubin_u ; d_speed_lower_u ; d_curv_u];% ; curv_acc_u];
            
            % add lower speed bound:
%             A = [A ; A_speed_lower ; A_curv_xy];
%             l = [l ; d_speed_lower ; d_curv_l];
%             u = [u ; Inf*ones(size(d_speed_lower,1),1) ; d_curv_u];
            % initial/end/continuity and Dubin points:
%             l = d_tot_Dubin_l;
%             u = d_tot_Dubin_u;
%             A = A_full_Dubin;
%             % add acc. constraints:
%             A = [A ; A_acc_xy];
%             l = [l ; d_acc_l];
%             u = [u ; d_acc_u];
            
            

            % Create an OSQP object
            prob = osqp;
            
            try % handle if problem setup fails, can be non-convex
            % Setup workspace and change alpha parameter
                prob.setup(P, q, A, l, u,'alpha',1.6,'time_limit',10e-3)%,'verbose',0);%,'eps_prim_inf',1);
            % higher alpha seems to give faster convergence: 0 < alpha < 2,
            % default is 1.6
            catch
               %disp('Problem setup failed')
               x = NaN; y = NaN; T = NaN; t_eps = NaN; solverStatus = 'NaN';
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

            % Solve problem
            res = prob.solve();
            X = res.x;
            % solver status:
            solverStatus = res.info.status;
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
            n_div = 200; %no. of time division for plotting, >1000 for good tracking
            t_lists = zeros(M,n_div);
            for i=1:M %create list of times
                if i==1
                    t_lists(i,:) = linspace(0,T_list(2),n_div);
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

            x = cell(d_k+1,M); % cell of polynomial functions
            y = cell(d_k+1,M); % cell of polynomial functions
            for i=1:d_k+1
                for j=1:M
                    x{i,j}=@(t) subs(der_x_sym(i,j),t_sym,t);
                    y{i,j}=@(t) subs(der_y_sym(i,j),t_sym,t);
                end
            end
            
            n_div = 200; %no. of time division for plotting, >1000 for good tracking
            t_lists = zeros(M,n_div);
            for i=1:M %create list of times
                if i==1
                    t_lists(i,:) = linspace(0,T_list(2),n_div);
                else
                    t_lists(i,:) = linspace(T_list(i),T_list(i+1),n_div);
                end
            end
            
           %Show the generated path. Notice the direction of the turns.
           figure()
           hold on;
           show(pathSegObj{1})
           %hold off;
            
            for i=1:M
                plot(x{1,i}(t_lists(i,:)),y{1,i}(t_lists(i,:)))
            end
            hold off;
               

            %% END OF ALGORITHM
            
            % enforce straight movement to pick up speed:
            % accelerate max (for now) in a straight line
            
            
            %used to set lowest speed in curv-constraint
            
            % 'verbose',False - disables print output
            
  
            % warm-start, x = primal variables, y = dual variables = lambdas
            %prob.warm_start('x', x0, 'y', y0)
 
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
            t0 = 0; % (!) remove if using low speed mechanism
            
            pos_x = []; pos_y = []; theta = []; vel_x = []; vel_y = []; acc_x = []; acc_y = [];
                        
            if reversing % invert time_lists and change sign of speeds!
                for i=M:-1:1
                    Time_list = t_lists(i,:);
                    Time_list = fliplr(Time_list);
                    pos_x = [pos_x , x{1,i}(Time_list)];
                    pos_y = [pos_y , y{1,i}(Time_list)];
                    theta = [theta , double(atan2(-y{2,i}(Time_list),-x{2,i}(Time_list)))];
                    vel_x = [vel_x , double(-x{2,i}(Time_list))];
                    vel_y = [vel_y , double(-y{2,i}(Time_list))];
                    acc_x = [acc_x , -x{3,i}(Time_list)];
                    acc_y = [acc_y , -y{3,i}(Time_list)];
                end
            else % do nothing
                for i=1:M
                    Time_list = t_lists(i,:);
                    pos_x = [pos_x , x{1,i}(Time_list)];
                    pos_y = [pos_y , y{1,i}(Time_list)];
                    theta = [theta , double(atan2(y{2,i}(Time_list),x{2,i}(Time_list)))];
                    vel_x = [vel_x , double(x{2,i}(Time_list))];
                    vel_y = [vel_y , double(y{2,i}(Time_list))];
                    acc_x = [acc_x , x{3,i}(Time_list)];
                    acc_y = [acc_y , y{3,i}(Time_list)];
                end
            end
            
            % define the signed speed function
            u1 = @(vel_x,vel_y,alpha) ((vel_x ~= 0).*sign(vel_x.*cos(alpha)) + (vel_x == 0).*sign(vel_y.*sin(alpha))).*sqrt(vel_x.^2+vel_y.^2); % signed velocity indicating forward or reverse motion

            vel = u1(vel_x,vel_y,theta);
            
            
            % outputs matrix of interpolated states along polynomial
            % segment
            global numStateVariables
            global states_in_tree
            
            % curvature function:
            kappa = @(x_vel,y_vel,x_acc,y_acc) (y_acc.*x_vel-y_vel.*x_acc)./(x_vel.^2+y_vel.^2).^(3/2);
            
            % set path to infeasible if solver fails, prevents crash
            %if any(isnan(X), 'all')  %set path infeasible (NaN) if solver fails
            if strcmp(solverStatus,'solved') == 1 % || strcmp(solverStatus,'solved inaccurate') == 1
               state_matrix = double([pos_x',pos_y',theta',vel',acc_x',acc_y']);
               
               % compute curvature: if ok, then continue, otherwise
               % discard the path and set to nan's
               vel_x = vel.*cos(theta); vel_y = vel.*sin(theta);
               curv = kappa(vel_x,vel_y,acc_x,acc_y);
               tic
               vpa(max(abs(curv)))
               toc
               if max(abs(curv)) > K_max
                   state_matrix = [state1; NaN*ones(length(X_x)-1,numStateVariables)];
                   failed_list = [failed_list ; [state1, state2]];
               end
               
               solved_list = [solved_list ; [state1, state2]];
            else % solver has failed to find solution
               state_matrix = [state1; NaN*ones(length(X_x)-1,numStateVariables)];
               failed_list = [failed_list ; [state1, state2]];
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
            %dist
            
            if isnan(dist) % set dist to very large, make it larger than some threshold for the interpolate function
                dist = NaN_distance;
                NaN_state = NaN*ones(1,numStateVariables);
                state_couple = [state1 ; NaN_state];
                solved_dist_map(num2str(state_couple))= dist;
                state_matrix_map(num2str(state_couple)) = state_matrix;
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