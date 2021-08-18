% steering function:
function [dist,x,y,T,t_eps,solverStatus] = Steer_acc(obj,state1,state2,N)
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


            M = 1; % no. of segments
            T_list = zeros(M+1,1);
            u2_max_list = zeros(M,1);
            
            % create initial reeds-shepp curve connecting the pose:
            % Reeds-Shepp-curves:
%             ConnObj = reedsSheppConnection;
%             % Dubin:
%             ConnObj = dubinsConnection;
%             ConnObj.MinTurningRadius = 1/K_max;
%             
%             startPose = [state1(1) state1(2) state1(3)];
%             goalPose = [state2(1) state2(2) state2(3)];
%             tic
%             [pathSegObj,pathCosts] = connect(ConnObj,startPose,goalPose)
%             toc
            %show(pathSegObj{1})

            %No_runs = 1; %no of runs of optimization 
               
            % take speed as average of the two sampled speeds
            
            v_eps = 3; % 2 = walking speed - gives +- 0.7 m/s^2 bounds in perp. acceleration
            %a0 = acc_max; % take max acc, change if needed
            
            % default start/end accelerations if speed too slow at
            % start/end:
            a_start = sign(v_i)*acc_max; % neg. accel if reversing (back up faster)
            a_end = -sign(v_end)*acc_max; % neg. accel if reversing (back up faster)
            
            t_small = 1; % small delta_t for feasible acceleration curves, currently not used       
            
            speed = abs(0.5*(state1(4)+state2(4)));
            speed = v_max;
            if speed == 0
                speed = v_max/2; % set to half max speed if both zero to avoid Inf
            end
            
            speed = v_max/2; % (!) already divided by sqrt(2) later, special case test
            
            % using length of each segment with constant speed
            for i=2:length(T_list)
                T_list(i) = T_list(i-1)+dist_tot/speed;
            end

            % scale with maximum yaw_rate input to increase/decrease time in that
            % segment
            k = 1; % some value [0,1]

            time_scaling = 1; %scale to increase total time
            T_list = time_scaling*T_list;
            T = T_list(end); % final 
            %T = 10;
            
            t_eps = abs(abs(v_end)-v_eps)/abs(a_start);
            t_f_eps = T-abs(abs(v_end)-v_eps)/abs(a_end);

            %% MIT - Polynomial trajectory generation with QP

            syms t

            %N = 6; % order of polynomials --> N+1 coefficients, comment
            %out if providing as input instead
            d_k = 2; % order of derivative continuity (x^(k) continuous)
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
            
            if abs(v_i) < v_eps
                slow_start = true;
            end
            if abs(v_end) < v_eps
                slow_end = true;
            end
            
            %% (!) SLOW MECHANISM SECTION:
            % set false to disable low-start mechanism
            slow_start = false; slow_end = false; slow = false;
            
            % a_start, a_end is the constant accel.
            t0 = 0;
            %t_eps = t_eps; % defined above
            % constant accel: other forms also possible
            delta_t_f = T-t_f_eps;
            % b.c: % satisfy initial pos, vel, end vel, and end accel.
            a1 = 0.5*acc_i(1)*sign(v_i);
            a2 = v_i*cos(alpha_i);
            a3 = pos_i(1);
            b1 = 0.5*acc_i(2)*sign(v_i);
            b2 = v_i*sin(alpha_i);
            b3 = pos_i(2);
            
            c1 = 0.5*a_end*cos(alpha_end);
            c2 = sign(v_end)*v_eps*cos(alpha_end); % break if pos. speed at end
            c3 = state2(1)-(c1*delta_t_f^2+c2*delta_t_f);
            d1 = 0.5*a_end*sin(alpha_end);
            d2 = sign(v_end)*v_eps*sin(alpha_end);
            d3 = state2(2)-(d1*delta_t_f^2+d2*delta_t_f);
            
            % define polynomials
            x_start = @(t) a1*t.^2+a2*t+a3;
            y_start = @(t) b1*t.^2+b2*t+b3;
            v_x_start = @(t) 2*a1*t+a2;
            v_y_start = @(t) 2*b1*t+b2;
            acc_x_start = @(t) 2*a1;
            acc_y_start = @(t) 2*b1;
            
            x_end = @(t) c1*t.^2+c2*t+c3;
            y_end = @(t) d1*t.^2+d2*t+d3;
            v_x_end = @(t) 2*c1*t+c2;
            v_y_end = @(t) 2*d1*t+d2;
            acc_x_end = @(t) 2*c1;
            acc_y_end = @(t) 2*d1;
            
%             % solve for coefficients from b.c.
%             delta_t = t_eps;
%             A_eps_start = [0 0 0 0 1;
%                 0 0 0 1 0;
%                 0 0 2 0 0;
%                 4*delta_t^3, 3*delta_t^2, 2*delta_t, 1, 0;
%                 12*delta_t^2, 6*delta_t, 1, 0, 0];
%             
%             RHS_start_x = [x0; v0_x; acc_i_x; v_eps_x; a_start*cos(alpha_i)];
%             RHS_start_y = [y0; v0_y; acc_i_y; v_eps_y; a_start*sin(alpha_i)];
%             
%             a_vec = A_eps_start\RHS_start_x;
%             b_vec = A_eps_start\RHS_start_y;
%             a1 = a_vec(1); a2 = a_vec(2); a3 = a_vec(3); a4 = a_vec(4); a5 = a_vec(5);
%             b1 = b_vec(1); b2 = b_vec(2); b3 = b_vec(3); b4 = b_vec(4); b5 = b_vec(5);
%             
%             % define start polynomials:
%             x_start = @(t) a1*t.^4+a2*t.^3+a3*t.^2+a4.*t+a5;
%             y_start = @(t) b1*t.^4+b2*t.^3+b3*t.^2+b4.*t+b5;
%             v_x_start = @(t) 4*a1*t.^3+3*a2*t.^2+2*a3.*t+a4;
%             v_y_start = @(t) 4*b1*t.^3+3*b2*t.^2+2*b3.*t+b4;
%             acc_x_start = @(t) 12*a1*t.^2+6*a2.*t+2*a3;
%             acc_y_start = @(t) 12*b1*t.^2+6*b2.*t+2*b3;
%             
%             t_e_list = linspace(0,t_eps,10);
%             disp('hej')
%             acc_x_start(t_e_list)
%             figure()
%             hold on;
%             plot(t_e_list,x_start(t_e_list))
%             hold off;

%            % end polynomials:
%             delta_t = T-t_f_eps; 
%             A_eps_end = [delta_t^4, delta_t^3, delta_t^2, delta_t, 1;
%                 4*delta_t^3, 3*delta_t^2, 2*delta_t, 1, 0;
%                 12*delta_t^2, 6*delta_t, 2, 0, 0;
%                 0, 0, 2, 0, 0;
%                 0, 0, 0, 1, 0];
%             v_f_eps_x = sign(v_end)*v_eps*cos(alpha_end);
%             v_f_eps_y = sign(v_end)*v_eps*sin(alpha_end);
%             RHS_end_x = [state2(1), v_end*cos(alpha_end), state2(5), a_end*cos(alpha_end), v_f_eps_x]';
%             RHS_end_y = [state2(2), v_end*sin(alpha_end), state2(6), a_end*sin(alpha_end), v_f_eps_y]';
%             c_vec = A_eps_end\RHS_end_x;
%             d_vec = A_eps_end\RHS_end_y;
%             c1 = c_vec(1); c2 = c_vec(2); c3 = c_vec(3); c4 = c_vec(4); c5 = c_vec(5);
%             d1 = d_vec(1); d2 = d_vec(2); d3 = d_vec(3); d4 = d_vec(4); d5 = d_vec(5);
%             
%             % define end polynomials:
%             x_end = @(t) c1*t.^4+c2*t.^3+c3*t.^2+c4.*t+c5;
%             y_end = @(t) d1*t.^4+d2*t.^3+d3*t.^2+d4.*t+d5;
%             v_x_end = @(t) 4*c1*t.^3+3*c2*t.^2+2*c3.*t+c4;
%             v_y_end = @(t) 4*d1*t.^3+3*d2*t.^2+2*d3.*t+d4;
%             acc_x_end = @(t) 12*c1*t.^2+6*c2.*t+2*c3;
%             acc_y_end = @(t) 12*d1*t.^2+6*d2.*t+2*d3;
            
            % used for initial conditions of optim. solver, if needed
            x_eps = x_start(t_eps);
            y_eps = y_start(t_eps);
            v_x_eps = v_x_start(t_eps);
            v_y_eps = v_y_start(t_eps);
            acc_x_eps = 0; %acc_x_start(t_eps); % set to zero for always feasible
            acc_y_eps = 0; %acc_y_start(t_eps);
           
            % used as end condition for opt. solver, if needed
            x_f_eps = x_end(0);
            y_f_eps = y_end(0);
            v_x_f_eps = v_x_end(0);
            v_y_f_eps = v_y_end(0);
            acc_x_f_eps = acc_x_end(0);
            acc_y_f_eps = acc_y_end(0);
            
            % end of v_eps handling
            
            % start optimizing from epsilon speed:
            if slow_start == true  
                pos_i = [x_eps,y_eps];
                vel_i = [v_x_eps,v_y_eps]; 
                acc_i = [acc_x_eps,acc_y_eps]; % using sampled acceleration
               
            else
                pos_i = [state1(1),state1(2)];
                vel_i = [v_i*cos(alpha_i),v_i*sin(alpha_i)]; 
                acc_i = [state1(5),state1(6)]; % using sampled acceleration
                if sample_acc == true
                    acc_i = [0,0];
                else
                    acc_i = accel_i;
                end
                
            end
            
            % end:
            
            % if slow end:
            if slow_end == true
                pos_end = [x_f_eps,y_f_eps];
                vel_end = [v_x_f_eps,v_y_f_eps];
                acc_end = [0,0]; % not accounted for if not sampling acc
            else
                pos_end = [state2(1),state2(2)];
                vel_end = [v_end*cos(alpha_end),v_end*sin(alpha_end)];
%                acc_end = [state2(5),state2(6)];   
                acc_end = [0,0];   
            end
            
            
            %acc_x_end(linspace(0,T-t_f_eps,10))
            % RHS in opt.problem. (!) have added acc as sampled
            
%             if sample_acc == true 
%                 d_init_x = [pos_i(1) ; vel_i(1) ; acc_i(1)];
%                 d_end_x = [pos_end(1) ; vel_end(1) ; acc_end(1)];
%                 d_init_y = [pos_i(2) ; vel_i(2) ; acc_i(2)];
%                 d_end_y = [pos_end(2) ; vel_end(2) ; acc_end(2)];
%             else
%                 d_init_x = [pos_i(1) ; vel_i(1) ; acc_i(1)];
%                 d_end_x = [pos_end(1) ; vel_end(1)];
%                 d_init_y = [pos_i(2) ; vel_i(2) ; acc_i(2)];
%                 d_end_y = [pos_end(2) ; vel_end(2)];
%             end

            %dist = norm(pos_i-pos_end);
            %T = dist/(v_max/2); %rough approx. time based on euclidian distance
            %% END OF SLOW MECHANISM SECTION

            % Euclidian distance:
            dist_tot = norm(pos_i-pos_end);
            
%             % create epsilon time_point matrix;
%             A_eps = zeros(d_k+1,N+1);
%             for k=0:d_k %order of derivative
%             for col=1:N+1 %polynomial coeff.
%                 if (N-(col-1)-k) >= 0
%                     A_eps(k+1,col)=faculty_factor(N-(col-1),k)*t_eps^(N-(col-1)-k);
%                 end
%             end
%             end
            
            % enforce straight movement (set direction to same):
%             l_v_eps = [v_eps*cos(alpha_i),v_eps*sin(alpha_i)]'; % both x and y
%             u_v_eps = l_v_eps;
%             A_v_eps = A_eps(2,:);
%             % enforce straight acceleration up to epsilon point:
%             l_acc_eps = [acc_max*cos(alpha_i),acc_max*sin(alpha_i)]';
%             u_acc_eps = l_acc_eps;
%             A_acc_eps = A_eps(3,:);
%             
%             A_eps = [kron(eye(2),A_v_eps)];% ; kron(eye(2),A_acc_eps)];
%             l_eps = [l_v_eps]; % ; l_acc_eps];
%             u_eps = [u_v_eps]; % ; u_acc_eps];
            
            % disregard if already at above speed limit:
            if abs(v_i) >= v_eps
                t_eps = 0;
            end
            
            t_eps = 0; %(!) remove if using low speed mechanism
            % l_v_eps <= A_v_eps*p <= u_v_eps
            
            % no. of intermediate points should be scaled with the segment
            % distance:
            mid_div = round(10*dist_tot/50); % take 10 points per each 50 metres
            if mid_div<=4
                mid_div=10; %ensure that intermediate points not negative, 4-2 = 2 middle points
            end
            %mid_div = 5;
            %mid_div = 10; % no. of intermediate time points (+2) in segment for constraints
            %T_mid_list = linspace(0,T,mid_div);
            % enforce curvature constraints only when speed is high enough
            T_mid_list = linspace(t_eps,T,mid_div); %if t_eps = 0, speed is already high enough and curvature constraint is enforced on full path
            M_mid = length(T_mid_list)-1;
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
            if slow_start == true
                t_first = t_eps; 
            else
                t_first = 0;
            end
            if slow_end == true
                t_last = t_f_eps; % should be T if no slow end
            else
                t_last = T;
            end
            
            % ignore slow mechanism, causes much complexity that must be handled:
            t_first = 0;
            t_last = T;
            
            d_k_all = d_k; 
            % (!) first/last must be equal with this structure, otherwise change
            % A_init etc.
            d_k_first = 2; % satisfy "all" derivatives, prev. 3 (!) 2
            d_k_last = 2; % satisfy vel and acc, now w/ sampled acc
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

            %size(A_full)

            %% Objective function:
            % costs for each derivative of position, 0 = position
            k_T = 1; % time-penalty
            c0 = 0; % pos
            c1 = -0.1; % speed
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

            %% Add input constraints: M = 1 here
            % add curvature constraint:
            % heading angle
            speed_1 = abs(state1(4));
            speed_2 = abs(state2(4));
            s_min = min(speed_1,speed_2);
            if speed_1 < v_eps
                slow_start = true;
            end
            if speed_2 < v_eps
                slow_end = true;
            end
            if slow_start || slow_end
                slow = true;
            end
            if slow == true
                min_speed = v_eps;
            else % take lowest of the two speeds as lower limit
                min_speed = s_min;
            end
            
            if slow_start
                s_x_1 = v_eps*cos(alpha_i);
                s_y_1 = v_eps*sin(alpha_i);
            else
                s_x_1 = state1(4)*cos(alpha_i);
                s_y_1 = state1(4)*sin(alpha_i);
            end
            if slow_end
               s_x_2 = v_eps*cos(alpha_end);
               s_y_2 = v_eps*sin(alpha_end);
            else
               s_x_2 = state2(4)*cos(alpha_end);
               s_y_2 = state2(4)*sin(alpha_end);
            end   
            % set first bound speed to epsilon speed (if needed)
            s_x_list = [s_x_1,s_x_2];
            s_y_list = [s_y_1,s_y_2];
            [~,i_x] = min([abs(s_x_1),abs(s_x_2)]); % take min speed (magnitude), worst case scenario
            [~,i_y] = min([abs(s_y_1),abs(s_y_2)]);
            s_x = s_x_list(i_x);
            s_y = s_y_list(i_y); % take out min speed index
            
            % test: remove later
% %             s_x = sign(s_x)*100;
% %             s_y = sign(s_y)*100;
            
            A_speed = zeros(M_mid,M*(N+1));
            A_acc = zeros(M_mid,M*(N+1)); % approx. curvature constraint
            %take into account last time point, but not first (given by i.c.)
            %A_curv = [-s_y*A_first(3,:), s_x*A_first(3,:)]; %curvature needs all points!
            A_curv = [];
            % add first time point:
            theta_list = linspace(0,pi,5);
            for theta=theta_list
                A_curv = [A_curv ; -sin(theta)*A_first(3,:),cos(theta)*A_first(3,:)];
            end
            % Define linear epsilon speed constraints: 
            % Bounds 1) 2) 3) 4) are linear lower bounds encapsulating
            % speed circle: in order of unit circle direction
            A_1 = [A_first(2,:),A_first(2,:)];
            A_2 = [A_first(2,:),-A_first(2,:)];
            A_3 = [-A_first(2,:),-A_first(2,:)];
            A_4 = [-A_first(2,:),A_first(2,:)];
            
            %A_curv = []; % curvature constraint no longer enforced at start point
            for i=1:M_mid
                vec = zeros(M_mid,1);
                vec(i) = 1;
                if i==M_mid % add last time point
                    A_speed = A_speed + kron(vec,A_last(2,:));
                    A_acc = A_acc + kron(vec,A_last(3,:));
                    %A_curv = [A_curv ; -s_y*A_last(3,:), s_x*A_last(3,:)];
                    A_1 = [A_1 ; A_last(2,:),A_last(2,:)];
                    A_2 = [A_2 ; A_last(2,:),-A_last(2,:)];
                    A_3 = [A_3 ; -A_last(2,:),-A_last(2,:)];
                    A_4 = [A_4 ; -A_last(2,:),A_last(2,:)];
                    for theta=theta_list
                        A_curv = [A_curv ; -sin(theta)*A_last(3,:),cos(theta)*A_last(3,:)];
                    end
                else % take middle points
                    A_speed = A_speed + kron(vec,A_seg(2,:,i));
                    A_acc = A_acc + kron(vec,A_seg(3,:,i)); 
                    %A_curv = [A_curv ; -s_y*A_seg(3,:,i), s_x*A_seg(3,:,i)];
                    A_1 = [A_1 ; A_seg(2,:,i), A_seg(2,:,i)];
                    A_2 = [A_2 ; A_seg(2,:,i), -A_seg(2,:,i)];
                    A_3 = [A_3 ; -A_seg(2,:,i), -A_seg(2,:,i)];
                    A_4 = [A_4 ; -A_seg(2,:,i), A_seg(2,:,i)];
                    A_right = [A_seg(2,:,i), 0*A_seg(2,:,i)]
                    A_top = = [A_seg(2,:,i),
                    for theta=theta_list
                        A_curv = [A_curv ; -sin(theta)*A_seg(3,:,i),cos(theta)*A_seg(3,:,i)];
                    end
                end
            end
            
            % Define linear lower speed bounds:
            speed_eps_l = sqrt(2)*v_eps*ones(M_mid,1); % epsilon lower bound on speed
            speed_eps_u = Inf*ones(length(theta_list)*M_mid,1); % some large speed, no upper bound here    
            %size(A_speed)
            A_speed_xy = kron(eye(2),A_speed);
            A_acc_xy = kron(eye(2),A_acc);
            d_speed = v_max*ones(2*M_mid,1); %in both x and y (!) note v_max is already divided here by sqrt(2), not the same v_max as in test_steer!
            d_speed_l = -d_speed;
            
            % enforce strictly increasing/decreasing motion between nodes
            % (or at least speeds must be greater than lowest sampled speeds)
            d_speed_l = [s_x*ones(M_mid,1) ; s_y*ones(M_mid,1)];
            
            % decreasing
            % fix when lower speed bound greater than upper !!!!!
            d_speed_u = [max(max(s_x_1,s_x_2),v_max)*ones(M_mid,1) ; max(max(s_y_1,s_y_2),v_max)*ones(M_mid,1)];
            d_speed_l = [min(min(s_x_1,s_x_2),-v_max)*ones(M_mid,1) ; min(min(s_y_1,s_y_2),-v_max)*ones(M_mid,1)];
            d_speed_u = v_max*ones(2*M_mid,1);
            d_speed_l = -v_max*ones(2*M_mid,1);
            size(A_curv);
            size(A_acc_xy);
            
            % simple acceleration bounds (from Robin data)
            
            % curvature acceleration  bounds: for all angles
            curv_acc_l = -K_max*min_speed^2*ones(size(A_curv,1),1); %using speed and angle
            curv_acc_u = K_max*min_speed^2*ones(size(A_curv,1),1);
            
            %if curv_acc_l < acc_min % only use standard acc bounds (symmetrical in lower and upper, only need to consider one case):
            d_acc_l = [ones(size(A_acc_xy,1),1)*acc_min];
            d_acc_u = [ones(size(A_acc_xy,1),1)*acc_max];
            %else
            %end
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
            
            A = [A_full_xy ; A_speed_xy ; A_acc_xy ; A_curv];
            l = [d_tot_xy ; d_speed_l ; d_acc_l ; curv_acc_l];
            u = [d_tot_xy ; d_speed_u ; d_acc_u ; curv_acc_u];
            
            if K_max*min_speed^2 < d_acc_u(1) % use lowest acc_bounds only
                A = [A_full_xy ; A_speed_xy ; A_curv];
                l = [d_tot_xy ; d_speed_l ; curv_acc_l];
                u = [d_tot_xy ; d_speed_u ; curv_acc_u];
            else
                A = [A_full_xy ; A_speed_xy ; A_acc_xy];
                l = [d_tot_xy ; d_speed_l ; d_acc_l];
                u = [d_tot_xy ; d_speed_u ; d_acc_u];
            end
                
            
%             % LOW SPEED MECHANISM (w/ eps-constraints):
%             A = [A_full_xy ; A_speed_xy ; A_acc_xy ; A_curv ; A_eps];
%             l = [d_tot_xy ; d_speed_l ; d_acc_l ; curv_acc_l ; l_eps];
%             u = [d_tot_xy ; d_speed_u ; d_acc_u ; curv_acc_u ; u_eps];
            
            
%             A = [A_full_xy ; A_speed_xy ; A_acc_xy]% ; A_curv];
%             l = [d_tot_xy ; d_speed_l ; d_acc_l] % ; curv_acc_l];
%             u = [d_tot_xy ; d_speed_u ; d_acc_u] % ; curv_acc_u];

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
            RHS = [pos_i(1);vel_i(1);acc_i(1);pos_end(1);vel_end(1); 0; 0;
                   pos_i(2);vel_i(2);acc_i(2);pos_end(2);vel_end(2); 0; 0];
            
            p0 = A_matrix\RHS;
            %prob.warm_start('x',p0);
            
            % Solve problem
            res = prob.solve();
            solverStatus = res.info.status; %check solver status
            X = res.x;
            
            %return;
            % The inaccurate statuses define when the optimality, primal infeasibility
            % or dual infeasibility conditions are satisfied 
            % with tolerances 10 times larger than the ones set.
            %% using fmincon:
            % Q_tot_xy = double(Q_tot_xy);
            % FUN = @(p) p'*Q_tot_xy*p;
            % Aeq = A_full_xy;
            % Beq = d_tot_xy;
            % A = []; B = [];
            % p0 = ones(2*(N+1)*M,1);
            % LB = []; UB = []; NONLCON = [];
            % tic;
            % OPTIONS = optimoptions('fmincon','Algorithm','sqp');
            % %X = fmincon(FUN,p0,A,B,Aeq,Beq,LB,UB,NONLCON,OPTIONS);
            % toc
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
            t0 = 0; % (!) remove if using low speed mechanism
            
            Time_list = linspace(t0,T,N_intervals);
            
            if reversing % invert time_list and change sign of speeds!
                Time_list = fliplr(Time_list);
                theta = double(atan2(-y{2,1}(Time_list),-x{2,1}(Time_list)));
                vel_x = double(-x{2,1}(Time_list));
                vel_y = double(-y{2,1}(Time_list));
                acc_x = -x{3,1}(Time_list);
                acc_y = -y{3,1}(Time_list);
            else % do nothing
                theta = double(atan2(y{2,1}(Time_list),x{2,1}(Time_list)));
                vel_x = double(x{2,1}(Time_list));
                vel_y = double(y{2,1}(Time_list));
                acc_x = x{3,1}(Time_list);
                acc_y = y{3,1}(Time_list);
            end
            
            pos_x = x{1,1}(Time_list); 
            pos_y = y{1,1}(Time_list);
            
            % define the signed speed function
            u1 = @(vel_x,vel_y,alpha) ((vel_x ~= 0).*sign(vel_x.*cos(alpha)) + (vel_x == 0).*sign(vel_y.*sin(alpha))).*sqrt(vel_x.^2+vel_y.^2); % signed velocity indicating forward or reverse motion

            vel = u1(vel_x,vel_y,theta);
            

            
            %obtain acceleration at initial point:
            accel_i = [x{3,1}(Time_list(1)),y{3,1}(Time_list(1))];
            
            
            %accel_end = [0,0];
            % store accel. values in list together with state, then find
            % acceleration by searching for the state
            
            
            
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
               
               figure()
               hold on;
               plot(pos_x,pos_y)
               
               %Create a dubinsConnection object.
               
               dubConnObj = dubinsConnection;
               dubConnObj.MinTurningRadius = 1/K_max;
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
               
               
               
               %Show the generated path. Notice the direction of the turns.
               show(pathSegObj{1})
               hold off;
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
               
               
%             % if connected to goal:
%             if state2 == goal
%             
%             % copied from PlannerRRTStar - extend() function:
%             stateNN = state1; stateNew = StateNew;
%              % check motion validity
%             if ~obj.StateValidator.isMotionValid(stateNN, stateNew)
%                 statusCode = obj.MotionInCollision;
%                 return;
%             else % path is coll. free and proceed to interpolation if needed
%                  d = dist; % distance of path, must be the scalar one, before making it a list.
%                        
%                        
%             if d > obj.MaxConnectionDistance
%                 stateNew = obj.StateSpace.interpolate(stateNN, randState, obj.MaxConnectionDistance/d);  % L/d*(randState - nnState) + nnState;
%                 d = obj.MaxConnectionDistance;
%             end
%             costNew = costNN + d;
% 
%                 nearIndices = treeInternal.near(stateNew);
% 
%                 % look around at the near neighbors and insert stateNew
%                 % under the one that has lowest cost
%                 costMin = costNew;
%                 idMin = -1;
%                 
%                 for j = 1:length(nearIndices)
%                     idNear = nearIndices(j);
%                     stateNear = treeInternal.getNodeState(idNear);
%                     costNear = treeInternal.getNodeCostFromRoot(idNear);
% 
%                     costNewTenative = costNear + obj.StateSpace.distance(stateNear, stateNew);
% 
%                     if costMin > costNewTenative ...
%                         && obj.StateValidator.isMotionValid(stateNear, stateNew)
%                         
%                         costMin = costNewTenative;
%                         idMin = idNear;
%                     end
%                 end
%                 
%                 if idMin >= 0
%                     idNew = treeInternal.insertNode(stateNew, idMin);
%                 else
%                     idNew = treeInternal.insertNode(stateNew, idNN);
%                 end
%                 newNodeId = idNew;
% 
%                 
%                 % now see if it's possible to rewire any of the near
%                 % neighbors under the newly added node
%                 if idMin >= 0
%                     nearIndices = nearIndices(nearIndices~=idMin);
%                 end
% 
%                 for k = 1:length(nearIndices)
%                     idNear = nearIndices(k);
%                     stateNear = treeInternal.getNodeState(idNear);
%                     costNear = treeInternal.getNodeCostFromRoot(idNear);
%                     if costNear > costNew + obj.StateSpace.distance(stateNear, stateNew) && ...
%                         obj.StateValidator.isMotionValid(stateNear, stateNew)
%                         rewireStatus = treeInternal.rewire(idNear, idNew); %#ok<NASGU>
%                         %fprintf('attempted to rewire node %d under node %d, result: [%f]\n', int32(idNear), int32(idNew), rewireStatus);
%                     end
%                 end
%                 
%             end
               % added, remove later, for visualization %%%%%%%%%%%%
%                image = imread('parking_harbor.png');
%                Img = im2bw(image);
%                im_matrix = abs(Img-1); % convert 0 to 1 and vice versa
%                % 335 metres top-bottom
%                cell_par = 1.85; % gives 340 m
%                harbor_map = occupancyMap(im_matrix,cell_par);
% 
%                figure()
%                hold on;
%                show(harbor_map)
%                plot(state_matrix(:,1),state_matrix(:,2))
%                plot(states_in_tree(:,1),states_in_tree(:,2),'o')
%                hold off;
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               
               accel_end = [x{3,1}(Time_list(end)),y{3,1}(Time_list(end))];
               accel_map(num2str(state2)) = double(accel_end); %store state2 end acceleration
 
               
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