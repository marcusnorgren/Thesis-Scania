%% test script
%clear all;

clearvars -except T_new % clear all except new time list

set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot, 'DefaultLineLineWidth', 1);

% 
% % plotting heading angle constraint:
% kappa = @(x_vel,y_vel,x_acc,y_acc) (y_acc.*x_acc-y_vel.*x_acc)./(x_vel.^2+y_vel.^2).^(3/2);
% %relaxed input u2 - constraint (denom = u_2_max^2):
% %u2 = @(x_vel,y_vel,x_acc,y_acc) (y_acc.*x_acc-y_vel.*x_acc);
% 
% v_max = 50/3.6;
% x_vel = 1/sqrt(2)*v_max;
% y_vel = x_vel;
% % maxLongitudinalAcceleration: 2.0 # [m/s²]
% acc_max = 2;
% x_acc_list = -acc_max:0.1:acc_max;
% y_acc_list = x_acc_list;
% 
% [X,Y] = meshgrid(x_acc_list,y_acc_list);
% Z = kappa(x_vel,y_vel,X,Y);
% 
% figure();
% surf(X,Y,Z)

%% initial/end conditions:
v_max = (50/3.6); %max speed, m/s
v_i = v_max/2; % initial speed, negative if reversing
v_end = v_max/2; % end speed
%alpha_i = (45-180)*pi/180; % initial heading, reversing
alpha_i = 45*pi/180; 
alpha_end = 0*pi/180; % final heading
beta_i = (alpha_i+pi); % initial trailer heading

% Vehicle params from Scania (Robin data):
yaw_max = 15.708; % 900°
yaw_rate_max = 7.854; % [rad/s] 450 degrees/s
u2_max = yaw_rate_max;
K_max = 0.17; % max. curvature [1/m]


% RRT-map:
load exampleMaps
cell_param = 0.3; % 10 is default, 2.5 meters wide
map = occupancyMap(simpleMap,cell_param);

% harbor scenario:
image = imread('parking_harbor.png');
Img = im2bw(image);
im_matrix = abs(Img-1); % convert 0 to 1 and vice versa
% 335 metres top-bottom
cell_par = 1.85; % gives 340 m
harbor_map = occupancyMap(im_matrix,cell_par);

figure()
show(harbor_map)
start_harbor = [55,290]; % high up

%goal_harbor = [83,157]; % halfway down
%goal_harbor = [83,100]; % bottom left corner
goal_harbor = [214,17]; % bottom corner
goal_harbor = [118,115];
goal_harbor = [147,200];
MaxIter = 100000; % 10000 is default
MaxNodes = 100000; % 10000 is default, 1000000
maxConnDist = 40; % 50 good, 60 good

scenario = 'harbor';
switch(scenario)
    case 'harbor'
        speed_i = 2;
        speed_end = 2;
        alpha_i = 0*pi/180;
        alpha_end = 180*pi/180;
        pos_i = start_harbor;
        %goal_harbor = [175,240];
        goal_harbor = [85,200];
        goal_harbor = [175,200];
        goal_harbor = [75,240];
        pos_end = goal_harbor;
        vel_i = [speed_i*cos(alpha_i),speed_i*sin(alpha_i)];
        vel_end = [speed_end*cos(alpha_end),speed_end*sin(alpha_end)];
        acc_i = [0,0];
        acc_end = [0,0]; % not used, needed just for input
        map = harbor_map;
    case 'simple'
        alpha_i = 45*pi/180;
        alpha_end = 0*pi/180;
        pos_i = [0.125*map.XWorldLimits(2),0.125*map.YWorldLimits(2)]; 
        vel_i = [speed_i*cos(alpha_i),speed_i*sin(alpha_i)]; 
        acc_i = [0,0]; %now accounted for w/ sampled acc.
        % end:
        pos_end = [0.5*map.XWorldLimits(2),0.6*map.YWorldLimits(2)]; 
        %pos_end = [0.3*map.XWorldLimits(2),0.3*map.YWorldLimits(2)]; 
        pos_end = [0.9*map.XWorldLimits(2),0.9*map.YWorldLimits(2)];
        vel_end = [speed_end*cos(alpha_end),speed_end*sin(alpha_end)]; 
        acc_end = [0,0]; 
end

% initial % RRT map is from [0 2.6], so scale w/ some factor  

% RHS in opt.problem. : fixed initial and goal constraints
d_init_x = [pos_i(1) ; vel_i(1)]; 
d_end_x = [pos_end(1) ; vel_end(1)];
d_init_y = [pos_i(2) ; vel_i(2)];
d_end_y = [pos_end(2) ; vel_end(2)];

%dist = norm(pos_i-pos_end);
%T = dist/(v_max/2); %rough approx. time based on euclidian distance
%T = 2; % final time, let be variable later ?

%% RRT-planner: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Dubin = false;
Dubin = true; % true runs w/ Dubin paths, otherwise straight-RRT*
%load exampleMaps
%map = occupancyMap(simpleMap,10);
%map = occupancyMap(simpleMap,cell_param);

% inflate each obstacle cell with a radius of half width of vehicle
vehicle_Width = 2.60; % from Robin, vehicle width [m]
L1 = 3.75;
bumper = 1.41;
rear_to_bumper = L1 + bumper;
old_map = copy(harbor_map);
%inflate(harbor_map,rear_to_bumper);
show(harbor_map)

%% Straight RRT*-planner 
if Dubin == false
ss = stateSpaceSE2; % create state-space
sv = validatorOccupancyMap(ss); % Create an occupanyMap-based state validator using the created state space.
% Create an occupany map from an example map and set map resolution as 10 cells/meter.
load exampleMaps
%map = occupancyMap(simpleMap,cell_param); %decrease 2nd argument to increase map size
%map = occupancyMap(20,20,10);
sv.Map = map;
% Set validation distance for the validator.
sv.ValidationDistance = 0.1;
% Update state space bounds to be the same as map limits.
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
% Create the path planner and increase max connection distance.
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
%planner.MaxConnectionDistance = 0.3; 0.6

planner.MaxConnectionDistance = maxConnDist; % 0.3*(10/cell_param) , 5
planner.MaxIterations = MaxIter;
planner.MaxNumTreeNodes = MaxNodes;

% Set the start and goal states.
start = [pos_i(1),pos_i(2),alpha_i];
goal = [pos_end(1),pos_end(2),alpha_end];
% Plan a path with default settings.
rng(100,'twister'); % for repeatable result
tic
[pthObj,solnInfo] = plan(planner,start,goal);
toc
disp('Elapsed time RRT*-straight:') 

% Visualize the results.
show(map)
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path
%hold off;
d_RRT_x = pthObj.States(2:end,1); %Dubin/RRT* seems to use hard end constraint, can leave out last point (end-1)
d_RRT_y = pthObj.States(2:end,2);
end

%% RRT w/ Dubin/RS paths
if Dubin == true
ss = stateSpaceDubins;
%ss = stateSpaceReedsShepp;

ss.MinTurningRadius = 1/K_max; % 0.2

% Create an occupancyMap-based state validator using the created state space.

sv = validatorOccupancyMap(ss);
% Create an occupancy map from an example map and set map resolution as 10 cells/meter.

sv.Map = map;
% Set validation distance for the validator.
show(map)
sv.ValidationDistance = 0.1;
% Update state space bounds to be the same as map limits.

ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
% Create the path planner and increase max connection distance.

%maxConnDist_Dubin = 50;
planner = plannerRRTStar(ss,sv);
%planner.ContinueAfterGoalReached = true;
planner.MaxConnectionDistance = maxConnDist; 
planner.MaxIterations = MaxIter;
planner.MaxNumTreeNodes = MaxNodes;

% Set the start and goal states.
start = [pos_i(1),pos_i(2),alpha_i];
goal = [pos_end(1),pos_end(2),alpha_end];

% Plan a path with default settings.
rng(100,'twister'); % repeatable result
tic;
[pthObj,solnInfo] = planner.plan(start,goal);
toc
disp('Elapsed time Dubin/RS-RRT*:')

pathStates = pthObj.States;
%%
% tic
% interpolate(pathStates,500); % interpolate to get full path b/w states
% toc
%%
% Visualize the results.
figure()
show(map);
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1), pthObj.States(:,2),'r','LineWidth',1) % draw path
plot(pathStates(:,1),pathStates(:,2),'bo');
hold off;
d_RRT_x = pthObj.States(2:end-1,1); %Dubin seems to use hard end constraint, can leave out last point (end-1)
d_RRT_y = pthObj.States(2:end-1,2);
end
%% add RRT points


%% obtain distance of dubin path:
%space = stateSpaceDubins;
%ss = stateSpaceReedsShepp;
%space.MinTurningRadius = 0.2;
% states1 = start;
% states2 = goal;
% dist = distance(space,states1,states2);

%% obtain coordinates from RRT-planner:
RRT_x = pthObj.States(:,1);
RRT_y = pthObj.States(:,2);

% Here begins the algorithm:
collision_check = true; % set to true to activate coll.checking
collision_free = false; % run until true, set to true to disable
while collision_free == false
if collision_check == false
collision_free = true; % set this to true to disable coll. checking.
end
d_RRT_x = RRT_x(2:end-1); %RRT* ends at goal - leave out last. Dubin seems to use hard end constraint, can leave out last point (end-1)
d_RRT_y = RRT_y(2:end-1);
RRT_states = pthObj.States;
dist_tot = 0;
dist_vec = zeros(length(RRT_x),1);

if Dubin == true
% Dubin distance --> can't be used with collision avoidance 
for i=1:size(RRT_states,1)
    if i < size(RRT_states,1)
        dist = distance(ss,RRT_states(i,:),RRT_states(i+1,:));
    end
    dist_vec(i) = dist; % store distance for each segment
    dist_tot = dist_tot + dist; % store total distance
end
end
if Dubin == false
% Euclidian distance:
for i=1:length(RRT_x)-1
    dist = norm([RRT_x(i),RRT_y(i)]-[RRT_x(i+1),RRT_y(i+1)]);
    dist_vec(i) = dist; % store distance for each segment
    dist_tot = dist_tot + dist; % store total distance
end
end

%% Time allocation:
% perform several initial guesses and then use gradient descent to find
% optimal time allocation based on cost function


M = length(d_RRT_x)+1; % no. of segments = #RRT-points
T_list = zeros(M+1,1);
u2_max_list = zeros(M,1);

%No_runs = 1; %no of runs of optimization 

speed = v_max/sqrt(2)%/1.5 % desired avg. speed, change depending on environment. 1.5 good
speed = v_max/2;
% using length of each segment with constant speed
for i=2:length(T_list)
    T_list(i) = T_list(i-1)+dist_vec(i-1)/speed;
end

T_new = T_list;
no_runs = 1;

for iter=1:no_runs

% update w/ new time:
T_list = T_new;
T = T_list(end); % final time


%% MIT - Polynomial trajectory generation with QP

syms t

%M = 4; % no of path segments
M = length(d_RRT_x)+1; % no. of segments = #RRT-points
N = 5; % order of polynomials --> N+1 coefficients, 
d_k = 3; % order of derivative, jerk (d_k=3) needed for u2 input, (!) not continutiy, only for creating matrices

% k = 
% 0: pos
% 1: vel
% 2: acc
% 3: jerk
% 4: snap
% 5: crackle
% 6: pop

% continuity constraints:
faculty_factor = @(N,k) factorial(N)/factorial(N-k);

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
d_k_fix = 1;
A_init = kron([1,zeros(1,M-1)],A_first(1:d_k_fix+1,:));
A_end = kron([zeros(1,M-1),1],A_last(1:d_k_fix+1,:));

A_full = [A_init; A_end ; A_tot]; % full constraint matrix
d_free = zeros(size(A_tot,1),1); % d_free=d_p in MIT paper (free constraints)
d_fixed_x = [d_init_x ; d_end_x]; % vector of fixed constraints in x
d_tot_x = [d_fixed_x ; d_free];
d_fixed_y = [d_init_y ; d_end_y]; % vector of fixed constraints in y
d_tot_y = [d_fixed_y ; d_free];

size(A_full)

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

%% Add RRT constraints
A_RRT_x = zeros(M-1,(M-1)*(N+1));
for i=1:M-1
    vec = zeros(1,M-1);
    vec(i) = 1;
    diag_m = diag(vec);
    A_RRT_x = A_RRT_x + kron(diag_m,A_seg(1,:,i));
end
A_RRT_x = [A_RRT_x , zeros(size(A_RRT_x,1),N+1)];
A_RRT_y = kron([0 1],A_RRT_x);
A_RRT_x = kron([1 0],A_RRT_x);
A_full_RRT = [A_full_xy ; A_RRT_x ; A_RRT_y];
d_tot_RRT = [d_tot_xy ; d_RRT_x ; d_RRT_y];

%% Add input constraints:
A_speed = zeros(M,M*(N+1)); 
A_acc = zeros(M,M*(N+1)); 
A_curv = zeros(M,2*M*(N+1)); % will contain all waypoints, will be double of columns as A_acc and A_speed

theta_list = linspace(0,pi,5); % used as linear bounds for acceleration. 
%take into account last time point, but not first (given by i.c.)
for i=1:M
    A_curv_add = []; % matrix for one waypoint, to be added to the full A_curv matrix.
    vec = zeros(1,M);
    vec(i) = 1;
    diag_m = diag(vec);
    if i==M % add last time point
        A_speed = A_speed + kron(diag_m,A_last(2,:));
        A_acc = A_acc + kron(vec,A_last(3,:)); 
%         for theta=theta_list
%             A_curv_add = [A_curv_add ; -sin(theta)*A_last(3,:),cos(theta)*A_last(3,:)];
%         end
%         A_curv = A_curv + kron(vec,A_curv_add); 
    else
        A_speed = A_speed + kron(diag_m,A_seg(2,:,i));
        A_acc = A_acc + kron(vec,A_seg(3,:,i)); 
%         for theta=theta_list
%             A_curv_add = [A_curv_add ; -sin(theta)*A_seg(3,:,i),cos(theta)*A_seg(3,:,i)];
%         end
%         A_curv = A_curv + kron(vec,A_curv_add); 
    end
end

A_speed_xy = kron(eye(2),A_speed);
d_speed = v_max/sqrt(2)*ones(2*M,1);
d_speed_l = -d_speed;
d_speed_u = d_speed;
acc_max = 0.7; % 2 m/s^2 in both long. and lateral acc. is default bounds
A_acc_xy = kron(eye(2),A_acc);
d_acc_l = -acc_max*ones(2*M,1);
d_acc_u = acc_max*ones(2*M,1);
% curv. constraints:
min_speed = 2; % walking speed, will be very conservative, as low as possible since want speed to be above this limit...
curv_acc_l = -K_max*min_speed^2*ones(size(A_curv,1),1); %using speed and angle
curv_acc_u = K_max*min_speed^2*ones(size(A_curv,1),1);


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
A = [A_full_RRT ; A_speed_xy ; A_acc_xy];% ; A_curv];
l = [d_tot_RRT ; d_speed_l ; d_acc_l];% ; curv_acc_l];
u = [d_tot_RRT ; d_speed_u ; d_acc_u];% ; curv_acc_u];

% Create an OSQP object
prob = osqp;

% Setup workspace and change alpha parameter
prob.setup(P, q, A, l, u, 'alpha', 1.6);

% warm-start, x = primal variables, y = dual variables = lambdas
%prob.warm_start('x', x0, 'y', y0)

% Solve problem
res = prob.solve();
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

%% Compute gradient (w/ cost on acceleration only):
c2 = 1; % cost on accel.
k_T = 0; % time-penalty, optimum should be indep. of k_T according to MIT, 
%(!) MIT says cost on acc. can be driven to zero by letting T->inf, but this is only true for some scenarios...
grad_J_T = zeros(M,1);
delta_T_list = T_list(2:end)-T_list(1:end-1); % create segment time list for clarity
time_index = 0; 
f_k = @(k,t) x{3,k}(t).^2 + y{3,k}(t).^2; % penalty function for each segment
for n = 1:M
    grad_J_T(n) = c2*f_k(n,T_list(n+1));
%     sum_n = 0;
%     sum_n_plus1 = 0;
%     for k=n:M % first sum
%         sum_n = sum_n + c2*f_k(k,sum(delta_T_list(1:k)));
%     end
%     for k = n+1:M % second sum
%         if k==1
%             % add nothing
%         else
%             sum_n_plus1 = sum_n_plus1 + c2*f_k(k,sum(delta_T_list(1:k-1)));
%         end
%     end  
%     if n == M
%         grad_J_T(n) = c2*f_k(n,sum(delta_T_list(1:M)));
%     else 
%         grad_J_T(n) = sum_n-sum_n_plus1;
%     end
end
grad_J_T = grad_J_T + k_T*ones(M,1);

%%
decrease_factor = 0.1; % factor of smallest segment time to decrease with, to prevent segment time from becoming zero.
% compute descent direction:
p_k = -grad_J_T/norm(grad_J_T)

delta_T_min = min(delta_T_list); % pick out smallest segment time
p_k_min = min(p_k); % pick out smallest descent direction component
alpha = decrease_factor*delta_T_min/abs(p_k_min);
delta_T_update = delta_T_list + alpha*p_k;
T_update = zeros(M,1);
for i=1:M
    T_update(i) = sum(delta_T_update(1:i));
end

% initial time point is always zero
T_old = T_list
T_new = [T_list(1) ; T_update]

%% Collision checking:
tic
dt=1; % time_index stepmping
collision_list = zeros(M,1);
collision_free = true; % assume collision free first
if collision_check == true
for seg=1:M
    first_collision = true;
    for t_i=1:dt:size(t_lists,2)
        x_i = double(x{1,seg}(t_lists(seg,t_i)));
        y_i = double(y{1,seg}(t_lists(seg,t_i)));
        %checks collision avoidance of curve with map
        occupied = checkOccupancy(map,[x_i y_i]); 
        if occupied == 1 %1 is occupied, add to colliding segments
            collision_list(seg) = 1;
            collision_free = false; % re-run required
            if first_collision == true
                first_collision = false;
                disp(strcat('(!) collision detected at segment: ',num2str(seg)))
            end
        end
    end
end

if collision_free == true
    disp('path collision free!');
end
toc
% add collided points as extra RRT-points, in middle of segment:
if Dubin == true
n_interps = 10;
N_interps = n_interps*M; % interpolate b/w the two Dubin states, 10 pts for each segment
interpolate(pthObj,N_interps);
end
for i=1:M
    if collision_list(i) == 1 % one if segment collided
        % straight-line RRT*
        if Dubin == false 
            RRT_x = [RRT_x(1:i) ; 0.5*(RRT_x(i)+RRT_x(i+1)) ; RRT_x(i+1:end)];
            RRT_y = [RRT_y(1:i) ; 0.5*(RRT_y(i)+RRT_y(i+1)) ; RRT_y(i+1:end)];
        else
            % Dubin/RS RRT*
            new_State = pthObj.States(n_interps/2+n_interps*(i-1),:) % add point halfway b/w
            New_x = new_State(1); New_y = new_State(2);
            RRT_x = [RRT_x(1:i) ; New_x ; RRT_x(i+1:end)];
            RRT_y = [RRT_y(1:i) ; New_y ; RRT_y(i+1:end)];
        end
    end
end  
end


% HERE ENDS ALGORITHM
%% plot trajectory
lw = 1.5;
% {derivative+1,segment}
% position:
figure()
show(old_map)
hold on;
for i=1:M
    plot(x{1,i}(t_lists(i,:)),y{1,i}(t_lists(i,:)),'LineWidth',lw);
    plot(RRT_x,RRT_y,'o');
end
hold off;
fs = 16;
%title('Dubin-RRT* + polynomial interpolation','FontSize',fs);
ax = gca;
set(gca,'FontSize',14)
%exportgraphics(ax,'DubinRRT_crop.eps')'
print('DubinRRT_cropcNEW','-depsc2')

% velocity
% figure()
% hold on;
% for i=1:M
%     plot(x{2,i}(t_lists(i,:)),y{2,i}(t_lists(i,:)));
% end
% hold off;
% title('velocity');
% acceleration
% figure()
% hold on;
% for i=1:M
%     plot(x{3,i}(t_lists(i,:)),y{3,i}(t_lists(i,:)));
% end
% hold off;
% title('acceleration');
end % end of time allocation loop (gradient descent)
end % end of collision free loop
%return;
%% obtain inputs:
% dimensions from Rouchon paper:
L1 = 3.75;
L = L1; % length b/w tractor front and rear axis
a = 1.5; % dist. from tractor axis to hitch point
b = 2.5; % dist from hitch point to trailer
input_u1 = cell(M,1);
input_u2 = cell(M,1);
curv_K = cell(M,1);
phi = cell(M,1);

for i=1:M
    input_u1{i} = @(t) sign(v_i)*sqrt(x{2,i}(t).^2+y{2,i}(t).^2);
    input_u2{i} = @(t) L*((y{4,i}(t).*x{2,i}(t)-y{2,i}(t).*x{4,i}(t)).*(x{2,i}(t).^2+y{2,i}(t).^2).^(3/2)-(y{3,i}(t).*x{2,i}(t)-y{2,i}(t).*x{3,i}(t))*(3/2).*(x{2,i}(t).^2+y{2,i}(t).^2).^(1/2).*(2*x{2,i}(t).*x{3,i}(t)+2*y{2,i}(t).*y{3,i}(t)))...
        ./((x{2,i}(t).^2+y{2,i}(t).^2).^3+(L^2)*(y{3,i}(t).*x{2,i}(t)-y{2,i}(t).*x{3,i}(t)).^2);

    curv_K{i} = @(t) (y{3,i}(t).*x{2,i}(t)-y{2,i}(t).*x{3,i}(t))./((x{2,i}(t).^2+y{2,i}(t).^2).^(3/2));
    phi{i} = @(t) atan2(y{2,i}(t),x{2,i}(t));
end

%K_values = zeros(N_steps,1);

figure();
hold on;
for i=1:M
%plot(t_lists(i,:),input_u1{i}(t_lists(i,:)),'r','HandleVisibility','off');
%plot(t_lists(i,:),input_u2{i}(t_lists(i,:)),'r','HandleVisibility','off');
%plot(t_lists(i,:),curv_K{i}(t_lists(i,:)),'b','HandleVisibility','off');
plot(t_lists(i,:),180/pi*phi{i}(t_lists(i,:)),'b','HandleVisibility','off');
end
% plot(linspace(0,T,2),[K_max,K_max],'k','HandleVisibility','off');
% plot(linspace(0,T,2),[-K_max,-K_max],'k','DisplayName','curvature bounds');
% plot(linspace(0,T,2),[u2_max,u2_max],'k-','DisplayName','u2_{max}');
% plot(linspace(0,T,2),[-u2_max,-u2_max],'k-','DisplayName','u2_{max}');
%plot(0,0,'r','DisplayName','velocity, $u_1$ [m/s]');
%plot(0,0,'r','DisplayName','yaw rate, $u_2$ [rad/s]');
%plot(0,0,'b','DisplayName','curvature $\kappa$ [m$^{-1}$]');
plot(0,0,'b','DisplayName','heading angle, $\alpha$ [deg]');
hold off;
fs = 16;
xlabel('time [s]','FontSize',fs)
%ylabel('curvature [m$^{-1}$]','FontSize',fs)
title('Heading angle','FontSize',fs)
legend('show','FontSize',16);

%% Simulate dynamics using inputs:
t_list_sim = reshape(t_lists',size(t_lists,1)*size(t_lists,2),1);
N_steps = size(t_list_sim,1);
x_vec = zeros(N_steps,1);
y_vec = x_vec;
phi_vec = x_vec;
alpha_vec = x_vec;
beta_vec = x_vec;
beta_vec(1)=beta_i; %initial trailer heading (i.e. along x-axis is pi)
u1_vec = x_vec;
u2_vec = x_vec;
x_trailer=x_vec;
y_trailer=x_vec;
% initial conditions:
x_vec(1) = pos_i(1); 
y_vec(1)= pos_i(2);
alpha_vec(1) = alpha_i; 
x_trailer(1)=x_vec(1)-(a*cos(alpha_vec(1))+b*cos(pi-beta_vec(1)));
y_trailer(1)=y_vec(1)-(a*sin(alpha_vec(1))-b*sin(pi-beta_vec(1)));

dt = T/N_steps;
u1_list = zeros(M,size(t_lists,2));
u2_list = zeros(M,size(t_lists,2));
for i=1:M
    u1_list(i,:)=-input_u1{i}(t_lists(i,:));
    u2_list(i,:)=input_u2{i}(t_lists(i,:));
end
u1_vec = reshape(u1_list',size(t_lists,1)*size(t_lists,2),1);
u2_vec = reshape(u2_list',size(t_lists,1)*size(t_lists,2),1);

u2_max_list = max(abs(u2_list),[],2);

%end % end of time allocation iteration
%return;

%% integrate system dynamics using input u1, u2

L_car = 6.0; % length
w_car = 2.60; % width
L_t = 7.82; % from Europe pdf
w_t = w_car; % same as tractor
d_car = sqrt((0.5*L_car)^2+(0.5*w_car)^2); % diagonal length of car
d_t = sqrt((0.5*L_t)^2+(0.5*w_t)^2); % diagonal length of trailer
alpha_car = atan(w_car/L_car); % angle from center to corner
alpha_t = atan(w_t/L_t);
t_list_2 = reshape(t_lists',size(t_lists,1)*size(t_lists,2),1);
for k=1:N_steps-1
    dt = t_list_2(k+1)-t_list_2(k);
    x_vec(k+1)=x_vec(k)+dt*u1_vec(k)*cos(alpha_vec(k)); %prev. alpha
    y_vec(k+1)=y_vec(k)+dt*u1_vec(k)*sin(alpha_vec(k));
    phi_vec(k+1)=phi_vec(k)+dt*u2_vec(k);
    alpha_vec(k+1)=alpha_vec(k)+dt*(1/L)*tan(phi_vec(k))*u1_vec(k);
    %trailer states:
    beta_vec(k+1)=beta_vec(k)+dt*(1/b)*((a/L)*tan(phi_vec(k))*cos(alpha_vec(k)-beta_vec(k))-sin(alpha_vec(k)-beta_vec(k)))*u1_vec(k);
    %using inputs:
    x_trailer(k+1)=x_trailer(k)+dt*u1_vec(k)*((a/L)*tan(phi_vec(k))*sin(alpha_vec(k)-beta_vec(k))+cos(alpha_vec(k)-beta_vec(k)))*cos(beta_vec(k));
    y_trailer(k+1)=y_trailer(k)+dt*u1_vec(k)*((a/L)*tan(phi_vec(k))*sin(alpha_vec(k)-beta_vec(k))+cos(alpha_vec(k)-beta_vec(k)))*sin(beta_vec(k));
    % using geometry (relation to tractor) - same result! 
    %x_trailer(k+1)=x_vec(k+1)-(a*cos(alpha_vec(k+1))+b*cos(pi-beta_vec(k+1)));
    %y_trailer(k+1)=y_vec(k+1)-(a*sin(alpha_vec(k+1))-b*sin(pi-beta_vec(k+1)));
    
    % simulate objects with coordinates defining truck and trailer:
    % tractor:
    A_x(k) = x_vec(k) + d_car*cos(alpha_vec(k)-alpha_car);
    A_y(k) = y_vec(k) + d_car*sin(alpha_vec(k)-alpha_car);
    B_x(k) = A_x(k) - w_car*cos(pi/2-alpha_vec(k));
    B_y(k) = A_y(k) + w_car*sin(pi/2-alpha_vec(k));
    C_x(k) = B_x(k) - L_car*cos(alpha_vec(k));
    C_y(k) = B_y(k) - L_car*sin(alpha_vec(k));
    D_x(k) = A_x(k) - L_car*cos(alpha_vec(k));
    D_y(k) = A_y(k) - L_car*sin(alpha_vec(k));
    % trailer:
    A_x_t(k) = x_trailer(k) + d_t*cos((beta_vec(k)-pi)-alpha_t);
    A_y_t(k) = y_trailer(k) + d_t*sin((beta_vec(k)-pi)-alpha_t);
    B_x_t(k) = A_x_t(k) - w_t*cos(pi/2-(beta_vec(k)-pi));
    B_y_t(k) = A_y_t(k) + w_t*sin(pi/2-(beta_vec(k)-pi));
    C_x_t(k) = B_x_t(k) - L_t*cos((beta_vec(k)-pi));
    C_y_t(k) = B_y_t(k) - L_t*sin((beta_vec(k)-pi));
    D_x_t(k) = A_x_t(k) - L_t*cos((beta_vec(k)-pi));
    D_y_t(k) = A_y_t(k) - L_t*sin((beta_vec(k)-pi));
end 

% added:
%%
figure()
hold on;
plot(A_x(1),A_y(1),'ro')
plot(B_x(1),B_y(1),'bo')
plot(C_x(1),C_y(1),'go')
plot(D_x(1),D_y(1),'yo')
plot(0.5*(A_x(1)+B_x(1)),0.5*(A_y(1)+B_y(1)),'yo')
plot(0.5*(D_x(1)+B_x(1)),0.5*(D_y(1)+B_y(1)),'yo')
plot(linspace(A_x(1),D_x(1),10),linspace(A_y(1),D_y(1),10),'ro')
hold off;

%%
figure()
hold on;
plot(x_vec,y_vec,'--','color','b','LineWidth',lw);
plot(x_trailer,y_trailer,'--','color','r','LineWidth',lw);
for i=1:M
    plot(x{1,i}(t_lists(i,:)),y{1,i}(t_lists(i,:)));
    plot(RRT_x,RRT_y,'-o');
end

% plot(A_x',A_y','o');
% plot(B_x',B_y','*');
%hold off;
title('position');
return;
% plot moving object:
h=[]; 
figure()
for k=1:N_steps 
delete(h) 
h = plot(x_vec(k), y_vec(k),'o');
drawnow; 
end
%%
% figure()
% L = 1. ; B = 2. ;
% C = [0. 0. ; L 0. ; L B ; 0. B; 0 0.] ;
% C1 = [0. 0]+C ;
% C2 = [0. 5.]+C ;
% C3 = [0. 10.]+C ;
%  path = rand(10,2) ;
% for i = 1:10
%     plot(C1(:,1),C1(:,2),'r')
%     hold on
%     plot(C2(:,1)+path(i,1),C2(:,2)+path(i,2),'b')
%     plot(C3(:,1),C3(:,2),'g')
%     hold off
%     axis([-10 10 0 20])
%     drawnow
%     pause(0.5)
% end
%%
% figure()
% plot(t_list_2,(180/pi)*(beta_vec)-180)
% title('trailer heading, beta [deg]');

% Moving simulation:
% create polygon coordinates:
x_car = [A_x',B_x',C_x',D_x']';
y_car = [A_y',B_y',C_y',D_y']';
x_t = [A_x_t',B_x_t',C_x_t',D_x_t']';
y_t = [A_y_t',B_y_t',C_y_t',D_y_t']';
L_car = 2;
w_car = L_car/2;
L_t = 2*L_car;
w_t = L_t/3;
d_car = sqrt((0.5*L_car)^2+(0.5*w_car)^2);

%g = hgtransform;
figure()
%axis equal
%gif('moving_truck.gif')
for i=1:1:length(A_x)
patch('XData',x_car(:,i),'YData',y_car(:,i),'FaceColor','black');%,'Parent',g)
patch('XData',x_t(:,i),'YData',y_t(:,i),'FaceColor','black');%,'Parent',g)
%pause(0.5)
drawnow
%gif
end



%xlim([-10 10])
%ylim([-10 10])
return;


% pt1 = [-3 -4 0];
% pt2 = [5 2 0];
% for t=linspace(0,1,100)
%   g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1));
%   drawnow
% end

s1 = 1/2;
s2 = 2;
r1 = 0;
r2 = 2*pi/3;
for t=linspace(0,1,100)
  g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1), ...
                         'scale',s1 + t*(s2-s1), ...
                         'zrotate',r1 + t*(r2-r1));
  pause(0.1)
  drawnow
end

%% Compare with Matlab smoothing function:

% Interpolate the transition poses of the path. Use these poses as the reference poses for interpolating the smooth path. Also return the motion directions at each pose.
%refPath = pthObj;
%[refPoses,refDirections] = interpolate(refPath,100);
[refPoses,refDirections] = interpolate(refPath)
% Specify the number of poses to return in the smooth path. Return poses spaced about 0.1 meters apart, along the entire length of the path.

approxSeparation = 0.1; % meters
numSmoothPoses = round(refPath.Length / approxSeparation);

% Generate the smooth path by fitting a cubic spline to the reference poses. smoothPathSpline returns the specified number of discretized poses along the smooth path.

[poses,directions] = smoothPathSpline(refPoses,refDirections,numSmoothPoses);

% Plot the smooth path. The more abrupt changes in curvature that were present in the reference path are now smoothed out.
figure()
plot(poses(:,1),poses(:,2),'LineWidth',2,'DisplayName','Smooth path')
hold off
