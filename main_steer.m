
%% initial/end conditions:
clear all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot, 'DefaultLineLineWidth', 1);

global function_counter solverTime_list yaw_end speed_end start goal 
global accel jerk accel_map jerk_map jerk_matrix
global state_matrix state_matrix_final
global distance_list
global free_path_map
global solved_time_list;
free_path_map = containers.Map();
function_counter = 0; % count number of function calls
solverTime_list = []; % count list 
solved_time_list = []; 
global poly_travel dubin_travel % list to store travel distance
poly_travel = []; dubin_travel = [];

% (!) yaw is renamed alpha in stateSpace class to avoid variable error
global v_max
v_max = (50/3.6); %max speed, m/s

global val_distance
val_distance = 1; % validation step along interpolated path

% Vehicle params from Scania (Robin data):
yaw_max = 15.708; % 900° 
yaw_rate_max = 7.854; % [rad/s] 450 degrees/s
u2_max = yaw_rate_max;
global K_max acc_max% also used in state-space
K_max = 0.17; % max. curvature [1/m]
acc_max = 2; % max. long and lateral. acceleration

% l = L1 = length b/w tractor front rear axis
% l = 3.75
% L2 = 7.65 
% M = -0.66 m

% RRT-map:
load exampleMaps
cell_param = 0.3; % 10 is default, 2.5 meters wide
map = occupancyMap(simpleMap,cell_param);

% create custom map from image: 
% parking harbour scenario:
%%
image = imread('parking_harbor.png');
Img = im2bw(image);
im_matrix = abs(Img-1); % convert 0 to 1 and vice versa
% 335 metres top-bottom
cell_par = 1.85; % gives 340 m
harbor_factor = 0.75;
open_map = occupancyMap(0*im_matrix,harbor_factor*cell_par); %w/o obstacles
harbor_map = occupancyMap(im_matrix,harbor_factor*cell_par);
empty = false;
if empty
    harbor_map = open_map; % %w/o obstacles
end
% inflate each obstacle cell with a radius of half width of vehicle
vehicle_Width = 2.60; % from Robin, vehicle width [m]
L1 = 3.75;
bumper = 1.41;
rear_to_bumper = L1 + bumper;
%inflate(harbor_map,rear_to_bumper);

figure()
show(harbor_map)
start_harbor = [55,290];
goal_harbor = [83,157];


%%
% initial % RRT map is from [0 2.6], so scale w/ some factor  
scenario = 'simple';
switch(scenario)
    case 'harbor'
        speed_i = 5;
        speed_end = 0;
        yaw_i = 0*pi/180;
        yaw_end = 180*pi/180;
        start_harbor = [80,385];
        goal_harbor = [40,20];
        
        % reversing test:
        reverse_test = false;
        if reverse_test
            start_harbor = [55,290];
            goal_harbor = [20,290]; % reversing test
            speed_i = 0;
            speed_end = 0;
            yaw_i = 180*pi/180;
            yaw_end = 180*pi/180; % reverse test
        end
        
        pos_i = start_harbor;
        pos_end = goal_harbor; 
        vel_i = [speed_i*cos(yaw_i),speed_i*sin(yaw_i)];
        vel_end = [speed_end*cos(yaw_end),speed_end*sin(yaw_end)];
        acc_i = [0,0];
        acc_end = [0,0]; % not used if not enforcing it, needed just for input
    
    case 'simple'   
        acc_i = [0,0]; %now accounted for w/ sampled acc.
        acc_end = [0,0]; 
        pos_i = [10,15];
        % many tests here:
        pos_end = [110,10]; % [75,15], [110,10], [75,10]
        yaw_i = 90*pi/180;
        yaw_end = -90*pi/180; % final heading
        speed_i = 0;
        speed_end = 0; % inital heading
        vel_i = [speed_i*cos(yaw_i),speed_i*sin(yaw_i)]; 
        vel_end = [speed_end*cos(yaw_end),speed_end*sin(yaw_end)];
end


global acc_end_x acc_end_y
acc_end_x = acc_end(1);
acc_end_y = acc_end(2);
global sample_acc
sample_acc = false; % set false to not sample acc
% (!) note: when sampling acc. problems can occur in motion validator if
% sampled point is very close to the constraint bounds

% Set the start and goal states for RRT-planner.
if sample_acc
    start = [pos_i,yaw_i, speed_i, acc_i];
    %global goal;
    goal = [pos_end,yaw_end, speed_end, acc_end];
else
    %global goal; w/o sampled acc.
    start = [pos_i,yaw_i, speed_i];
    goal = [pos_end,yaw_end, speed_end];
end
%initialize derivatives
%acc:
accel = [acc_i ; acc_end];
accel_map = containers.Map();
accel_map(num2str(start)) = acc_i;

global poly_map_x poly_map_y time_map_T solved_dist_map
poly_map_x = containers.Map();
poly_map_y = containers.Map();
time_map_T = containers.Map();
solved_dist_map = containers.Map();

global states_in_tree
states_in_tree = start;

global state_matrix_map % stores traj. along with state-couple
state_matrix_map = containers.Map();
state_matrix = start;
global distance_list_map
distance_list_map = containers.Map();
global distance_map_Map
distance_map_Map = containers.Map();
global goal
global coll_counter
coll_counter = 0;

%% RRT*-planner 
%ss = Polynomial_StateSpace; % create state-space

if sample_acc
    ss = Polynomial_StateSpace_acc; % samples accel.
else
    ss = Polynomial_StateSpace_NEW; % w/o sampled acc.
end
%ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss); % Create an occupanyMap-based state validator using the created state space.
sv = Polynomial_Validator(ss); %Create own from custom validator
% Create an occupany map from an example map and set map resolution as 10 cells/meter.
load exampleMaps
factor = 0.75;
map = occupancyMap(simpleMap,factor*cell_param); %decrease 2nd argument to increase map size
%map = occupancyMap(20,20,10);
if empty
    map = occupancyMap(0*simpleMap,factor*cell_param);
end

if strcmp(scenario,'harbor')
    map = harbor_map;
end
sv.Map = map;
% Set validation distance for the validator.
sv.ValidationDistance = val_distance; % 0.01 prev. if below 1, fraction list grows too large, why?
% Update state space bounds to be the same as map limits.

if sample_acc
    %ss.StateBounds = [map.XWorldLimits; map.YWorldLimits ; [-pi pi]; [-v_max/sqrt(2) v_max/sqrt(2)]; [-acc_max acc_max]; [-acc_max acc_max]];
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits ; [-pi pi]; [-v_max/sqrt(2) v_max/sqrt(2)]; [-K_max*v_eps^2 K_max*v_eps^2]; [-K_max*v_eps^2 K_max*v_eps^2]];
else
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]; [-v_max/sqrt(2) v_max/sqrt(2)]];
    %ss.StateBounds = [map.XWorldLimits; map.YWorldLimits+[150 0]; [-pi pi]; [-v_max/sqrt(2) v_max/sqrt(2)]];
end
 
% Create the path planner and increase max connection distance.
planner = plannerRRTStar(ss,sv); %
% use Marcus planner:
% planner = plannerRRTStarMarcus(ss,sv); % my edited planner, removes the
% interpolation function, but this can be done by setting maxconndist very
% large in plannerRRTStar - same result.
%planner = plannerRRT(ss,sv);
%planner.ContinueAfterGoalReached = true; 
global maxConnDist
maxConnDist = 1e4; % set to large to disable interpolation (i.e. cutoff)
global dist_limit; 
dist_limit = 50; % limit for trying to connecting states, to prevent unnec. function calls, similar to maxconndist. 

global dubin_time_list; dubin_time_list = [];

planner.MaxConnectionDistance = maxConnDist;  
% Probability of selecting the goal when sampling ([0,1])
planner.GoalBias = 0.05;
planner.MaxIterations = 17500; % Default: 1e4
planner.GoalReachedFcn = @CustomGoalReachedFcn; % use custom goal reached fcn

% Plan a path with default settings.
%rng(100,'twister'); % for repeatable result


[pthObj,solnInfo] = plan(planner,start,goal);
% Visualize the results.
show(map)
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path
%hold off;
RRT_x = pthObj.States(1:end,1); %Dubin/RRT* seems to use hard end constraint, can leave out last point (end-1)
RRT_y = pthObj.States(1:end,2); % 

%% plot 
Tot_solverTime = sum(solverTime_list);
function_counter_2 = function_counter;
No_states = 4; %number of sampled states
fraction = 0.1; %interpolation increment, 1/fraction is the number of points
No_points = 100; %(!) must be the same as N_intervals in state-space
tic
polyStates_list = zeros(length(RRT_x),No_points,No_states);
poly_list_x = cell(length(RRT_x)-1);
poly_list_y = cell(length(RRT_x)-1);
T_list = zeros(length(RRT_x)-1);

% mean solver time:
mean_time = mean(solverTime_list);
total_time = sum(solverTime_list);
%return

% for i=1:length(RRT_x)-1
%     polyStates = interpolate(ss,pthObj.States(i,:),pthObj.States(i+1,:),[0:fraction:1]);
%     polyStates_list(i,:,:) = polyStates;
%     [poly_x,poly_y,t_end] = get_polys(ss,pthObj.States(i,:),pthObj.States(i+1,:),[0:fraction:1]);
%     poly_list_x{i} = poly_x;
%     poly_list_y{i} = poly_y;
%     T_list(i) = t_end;
% end
% continuity of derivatives check: poly{segment}{derivative,1}(time_point)

%vpa(poly_list_x{1}{2,1}(T_list(1)))
% vpa(poly_list_x{1}{3,1}(linspace(0,T_list,10))')
% vpa(poly_list_y{1}{3,1}(linspace(0,T_list,10))')


%% obtain states, inputs and plot 

% define speed, w/ reversing direction as well:
u1 = @(vel_x,vel_y,alpha) ((vel_x ~= 0).*sign(vel_x.*cos(alpha)) + (vel_x == 0).*sign(vel_y.*sin(alpha))).*sqrt(vel_x.^2+vel_y.^2); % signed velocity indicating forward or reverse motion

figure()
show(map)
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
% for i=1:length(RRT_x)-1
%     plot(polyStates_list(i,:,1),polyStates_list(i,:,2),'-o','LineWidth',1.5) % draw path
% end
% curvature function:
kappa = @(x_vel,y_vel,x_acc,y_acc) (y_acc.*x_vel-y_vel.*x_acc)./(x_vel.^2+y_vel.^2).^(3/2);

curv_list =[];

final_tree = pthObj.States;
no_tree_states = size(pthObj.States,1);
x_list = zeros(no_tree_states-1);
y_list = zeros(no_tree_states-1);
T_list = zeros(no_tree_states-1);
% for i = 1:no_tree_states-1
%     state1 = final_tree(i,:);
%     state2 = final_tree(i+1,:);
%     state_couple = [state1 ; state2];
%     x_list(i) = poly_map_x(num2str(state_couple));
%     y_list(i) = poly_map_y(num2str(state_couple));
%     T_list(i) = poly_map_T(num2str(state_couple));
% end
% 

%%
No_states = 4;
figure()
show(map)
hold on;
free_paths = values(free_path_map);

for i = 1:length(free_paths)
    x = free_paths{i}(:,1);
    y = free_paths{i}(:,2);
    theta = free_paths{i}(:,3);
    vel = free_paths{i}(:,4);
    x_vel = free_paths{i}(:,4).*cos(theta);
    y_vel = free_paths{i}(:,4).*sin(theta);
    plot(x,y,'b')
    node = states_in_tree(i,1:2);
    plot(node(1),node(2),'ro')
    %hold off;
end
hold off;
%%
figure()
show(map)
hold on;
% (!) This section plots vehicle
% VEHICLE DIMS:
% dimensions from Scania:
L1 = 3.75;
L = L1; % length b/w tractor front and rear axis
a = -0.66; % dist. from tractor axis to hitch point 
b = 7.65; % dist from hitch point to trailer

L_car = 6.0; % length
w_car = 2.60; % width
L_t = 7.82; % from Europe pdf
w_t = w_car; % same as tractor
d_car = sqrt((0.5*L_car)^2+(0.5*w_car)^2); % diagonal length of car
d_t = sqrt((0.5*L_t)^2+(0.5*w_t)^2); % diagonal length of trailer
alpha_car = atan(w_car/L_car); % angle from center to corner
alpha_t = atan(w_t/L_t);

% plot vehicle: INITIAL
alpha_car_i = yaw_i; alpha_vec_i = yaw_i;
beta_vec_i = pi+alpha_car_i; alpha_t_i = beta_vec_i;
alpha_car_f = 0*pi/180; alpha_vec_f = yaw_end;
beta_vec_f = pi+alpha_car_f; alpha_t_f = beta_vec_f;

x_vec_i = start(1); y_vec_i = start(2); 
x_vec_f = goal(1); y_vec_f = goal(2);
x_trailer_i=x_vec_i-(a*cos(alpha_vec_i)+b*cos(pi-beta_vec_i));
y_trailer_i=y_vec_i-(a*sin(alpha_vec_i)-b*sin(pi-beta_vec_i));
x_trailer_f=x_vec_f-(a*cos(alpha_vec_f)+b*cos(pi-beta_vec_f));
y_trailer_f=y_vec_f-(a*sin(alpha_vec_f)-b*sin(pi-beta_vec_f));

A_x_i = x_vec_i + d_car*cos(alpha_vec_i-alpha_car);
A_y_i = y_vec_i + d_car*sin(alpha_vec_i-alpha_car);
B_x_i = A_x_i - w_car*cos(pi/2-alpha_vec_i);
B_y_i = A_y_i + w_car*sin(pi/2-alpha_vec_i);
C_x_i = B_x_i - L_car*cos(alpha_vec_i);
C_y_i = B_y_i - L_car*sin(alpha_vec_i);
D_x_i = A_x_i - L_car*cos(alpha_vec_i);
D_y_i = A_y_i - L_car*sin(alpha_vec_i);
% trailer:
A_x_t_i = x_trailer_i + d_t*cos((beta_vec_i-pi)-alpha_t);
A_y_t_i = y_trailer_i + d_t*sin((beta_vec_i-pi)-alpha_t);
B_x_t_i = A_x_t_i - w_t*cos(pi/2-(beta_vec_i-pi));
B_y_t_i = A_y_t_i + w_t*sin(pi/2-(beta_vec_i-pi));
C_x_t_i = B_x_t_i - L_t*cos((beta_vec_i-pi));
C_y_t_i = B_y_t_i - L_t*sin((beta_vec_i-pi));
D_x_t_i = A_x_t_i - L_t*cos((beta_vec_i-pi));
D_y_t_i = A_y_t_i - L_t*sin((beta_vec_i-pi));

% plot vehicle: FINAL
A_x_f = x_vec_f + d_car*cos(alpha_vec_f-alpha_car);
A_y_f = y_vec_f + d_car*sin(alpha_vec_f-alpha_car);
B_x_f = A_x_f - w_car*cos(pi/2-alpha_vec_f);
B_y_f = A_y_f + w_car*sin(pi/2-alpha_vec_f);
C_x_f = B_x_f - L_car*cos(alpha_vec_f);
C_y_f = B_y_f - L_car*sin(alpha_vec_f);
D_x_f = A_x_f - L_car*cos(alpha_vec_f);
D_y_f = A_y_f - L_car*sin(alpha_vec_f);
% trailer:
A_x_t_f = x_trailer_f + d_t*cos((beta_vec_f-pi)-alpha_t);
A_y_t_f = y_trailer_f + d_t*sin((beta_vec_f-pi)-alpha_t);
B_x_t_f = A_x_t_f - w_t*cos(pi/2-(beta_vec_f-pi));
B_y_t_f = A_y_t_f + w_t*sin(pi/2-(beta_vec_f-pi));
C_x_t_f = B_x_t_f - L_t*cos((beta_vec_f-pi));
C_y_t_f = B_y_t_f - L_t*sin((beta_vec_f-pi));
D_x_t_f = A_x_t_f - L_t*cos((beta_vec_f-pi));
D_y_t_f = A_y_t_f - L_t*sin((beta_vec_f-pi));

% Moving simulation:
% create polygon coordinates:
x_car_i = [A_x_i',B_x_i',C_x_i',D_x_i']';
y_car_i = [A_y_i',B_y_i',C_y_i',D_y_i']';
x_t_i = [A_x_t_i',B_x_t_i',C_x_t_i',D_x_t_i']';
y_t_i = [A_y_t_i',B_y_t_i',C_y_t_i',D_y_t_i']';

% create polygon coordinates:
x_car_f = [A_x_f',B_x_f',C_x_f',D_x_f']';
y_car_f = [A_y_f',B_y_f',C_y_f',D_y_f']';
x_t_f = [A_x_t_f',B_x_t_f',C_x_t_f',D_x_t_f']';
y_t_f = [A_y_t_f',B_y_t_f',C_y_t_f',D_y_t_f']';


%g = hgtransform;
%figure()
%axis equal
%gif('moving_truck.gif')
for i=1:1
patch('XData',x_car_i(:,i),'YData',y_car_i(:,i),'FaceColor','white');%,'Parent',g)
patch('XData',x_t_i(:,i),'YData',y_t_i(:,i),'FaceColor','white');%,'Parent',g)
patch('XData',x_car_f(:,i),'YData',y_car_f(:,i),'FaceColor','white');%,'Parent',g)
patch('XData',x_t_f(:,i),'YData',y_t_f(:,i),'FaceColor','white');%,'Parent',g)
%pause(0.5)
%drawnow
%gif
end

% PLOT PATHS
% plot tree expansion:
%plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'o'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'bo','LineWidth',1) % draw path

% plot expanded tree as well
for i = 1:length(free_paths)
    x = free_paths{i}(:,1);
    y = free_paths{i}(:,2);
    theta = free_paths{i}(:,3);
    vel = free_paths{i}(:,4);
    x_vel = free_paths{i}(:,4).*cos(theta);
    y_vel = free_paths{i}(:,4).*sin(theta);
    
    node = states_in_tree(i,1:2);
    final_node = false;
    for j = 1:size(final_tree,1)
        node_final = final_tree(j,:);
        if node == node_final(1:2)
            final_node = true;
%             if j==2
%                 plot(x,y,'r','LineWidth',2)
%             end
        end
    end
    if ~final_node
        plot(node(1),node(2),'bo','MarkerFaceColor', 'b')
        plot(x,y,'b')
    else
        plot(node(1),node(2),'ro','MarkerFaceColor', 'r')
        plot(x,y,'b')
    end
    %hold off;
end
plot(final_tree(end,1),final_tree(end,2),'ro','MarkerFaceColor', 'r')
%


dist_list = [];
% plot polynomial path
no_segments = no_tree_states-1;
for i=1:no_segments
    
    % plot solution:
    state1 = final_tree(i,:);
    state2 = final_tree(i+1,:);
    state_couple = [state1 ; state2];
    x = poly_map_x(num2str(state_couple));
    y = poly_map_y(num2str(state_couple));
    T = time_map_T(num2str(state_couple));
    time_span = linspace(0,T,100);
    
    dist = distance_list_map(num2str(state_couple));
    dist_list = [dist_list, dist];
    
    pos_x = x{1,1}(time_span);
    pos_y = y{1,1}(time_span);
    vel_x = x{2,1}(time_span);
    vel_y = y{2,1}(time_span);
    acc_x = x{3,1}(time_span);
    acc_y = y{3,1}(time_span);
    curv = kappa(vel_x,vel_y,acc_x,acc_y);
    % tractor heading angle:
    alpha = atan2(vel_y,vel_x);
    %u1 = sqrt(vel_x.^2 + vel_y.^2); % velocity
    
    vel_x_func = @(t) x{2,1}(t); vel_y_func = @(t) y{2,1}(t);
    vel_x_vec = double(vel_x_func(time_span)); vel_y_vec = double(vel_y_func(time_span)); alpha_vec = alpha;
    %K_values = zeros(N_steps,1);

    u1_vec = (u1(vel_x_vec,vel_y_vec,alpha_vec))'; % from function above
    
    %vel = u1(vel_x,vel_y,alpha);
    %u1_vec = (u1(vel_x_vec,vel_y_vec,alpha_vec))'; % from function above
    t = time_span;
    % yaw rate u2
    u2 = L*((y{4,1}(t).*x{2,1}(t)-y{2,1}(t).*x{4,1}(t)).*(x{2,1}(t).^2+y{2,1}(t).^2).^(3/2)-(y{3,1}(t).*x{2,1}(t)-y{2,1}(t).*x{3,1}(t))*(3/2).*(x{2,1}(t).^2+y{2,1}(t).^2).^(1/2).*(2*x{2,1}(t).*x{3,1}(t)+2*y{2,1}(t).*y{3,1}(t)))...
        ./((x{2,1}(t).^2+y{2,1}(t).^2).^3+(L^2)*(y{3,1}(t).*x{2,1}(t)-y{2,1}(t).*x{3,1}(t)).^2);

    % plot position
    plot(pos_x,pos_y,'r','LineWidth',2);
%     if i==1
%         plot([start(1),pos_x(1)],[start(2),pos_y(1)],'r','LineWidth',2);
%     end
    % plot u1:
%      plot(t,vel)
%     title('input $u_1$')
%     xlabel('time [s]')
%     ylabel('velocity [m/s]')
    % plot u2
%     plot(t,u2)
%     title('input $u_2$')
%     xlabel('time [s]')
%     ylabel('steering angle rate [rad/s]')
    % store values in list:
    if i==1
        time_span_list = time_span;
    end
    time_span_list = [time_span_list ; time_span+time_span_list(end)];
    pos_x_list(i,:) = pos_x;
    pos_y_list(i,:) = pos_y;
    vel_x_list(i,:) = vel_x_vec;
    vel_y_list(i,:) = vel_y_vec;
    acc_x_list(i,:) = acc_x;
    acc_y_list(i,:) = acc_y;
    %u1_list(i,:) = vel;
    u1_list(i,:) = u1_vec;
    u2_list(i,:) = u2;
    curv_list(i,:) = curv;
    alpha_list(i,:) = alpha;
end


%hold off;
%axis equal;
ax = gca;
ax.FontSize = 20;
hold off;
axis equal
tot_sol_time = Tot_solverTime+sum(dubin_time_list)
function_counter
size(states_in_tree,1)
title(strcat('Solution time: ',num2str(tot_sol_time),' s.',' Function calls: ',num2str(function_counter)));

%% 
figure()
hold on;
for i=1:no_segments
    plot(time_span_list(i,:),double(acc_x_list(i,:)),'r','DisplayName','$\ddot{x}$')
    plot(time_span_list(i,:),double(acc_y_list(i,:)),'b','DisplayName','$\ddot{y}$')
    plot(time_span_list(i,end),acc_x_list(i,end),'ro','MarkerFaceColor','r','HandleVisibility','off')
end
hold off;
legendFS = 22; axFS = 24;
legend('$\ddot{x}$','$\ddot{y}$','FontSize',legendFS)
ax = gca;
ax.FontSize = axFS;
xlabel('time [s]','FontSize',24)
ylabel('acceleration [m/s$^2$]','FontSize',24)

%%
% plot constraints and curvature

figure()
hold on;
for i=1:no_segments
    
plot(time_span_list(i,:),-linspace(K_max,K_max,length(time_span_list(i,:))),'k','HandleVisibility','off')
if i==1
    plot(time_span_list(i,:),linspace(K_max,K_max,length(time_span_list(i,:))),'k','DisplayName','$\pm \kappa_{max}$')
    plot(time_span_list(i,:),curv_list(i,:),'r-','DisplayName','$\kappa(t)$');
else
    plot(time_span_list(i,:),curv_list(i,:),'r-','HandleVisibility','off');
    plot(time_span_list(i,:),linspace(K_max,K_max,length(time_span_list(i,:))),'k','HandleVisibility','off')

end    
plot(time_span_list(i,end),curv_list(i,end),'ro','MarkerFaceColor','r','HandleVisibility','off')
% 
% plot(time_span_list(i,:),linspace(u2_max,u2_max,length(time_span_list(i,:))))
% plot(time_span_list(i,:),-linspace(u2_max,u2_max,length(time_span_list(i,:))))
% if i==1
%     plot(time_span_list(i,:),u1_list(i,:),'r-','DisplayName','$u_1$ [m/s]')
%     plot(time_span_list(i,:),u2_list(i,:),'r--','DisplayName','$u_2$ [rad/s]')
% else
%     plot(time_span_list(i,:),u1_list(i,:),'r-','HandleVisibility','off')
%     plot(time_span_list(i,:),u2_list(i,:),'r--','HandleVisibility','off')
% end

% plot(time_span_list(i,end),u1_list(i,end),'ro','MarkerFaceColor','r','HandleVisibility','off')
% plot(time_span_list(i,end),u2_list(i,end),'ro','MarkerFaceColor','r','HandleVisibility','off')

xlabel('time [s]')
ylabel('curvature [m$^{-1}$]','FontSize',24)

end
hold off;


%title('Curvature','FontSize',24);
% xlabel('x [m]','FontSize',24);
% ylabel('y [m]','FontSize',24);
legend('FontSize',legendFS);
ax = gca;
ax.FontSize = axFS; 
%axis equal;
%return;
%plot curves

%%
% for i=1:size(state_matrix_final,2)/No_states
%     x = state_matrix_final(:,1+(i-1)*No_states);
%     y = state_matrix_final(:,2+(i-1)*No_states);
%     theta = state_matrix_final(:,3+(i-1)*No_states);
%     x_vel = state_matrix_final(:,4+(i-1)*No_states).*cos(theta);
%     y_vel = state_matrix_final(:,4+(i-1)*No_states).*sin(theta);
% %     x_acc = state_matrix_final(:,5+(i-1)*No_states);
% %     y_acc = state_matrix_final(:,6+(i-1)*No_states);
%     %x{3,1}(
%     %plot(fig1,x,y)
%     plot(x,y)
%     
%     %curv_list = [curv_list,curv];
% end
%plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
%plot(state_matrix_final(:,1),state_matrix_final(:,2))
%hold off;

% check curvature 
figure()
hold on;
for i=1:size(state_matrix_final,2)/No_states
    x = state_matrix_final(:,1+(i-1)*No_states);
    y = state_matrix_final(:,2+(i-1)*No_states);
    theta = state_matrix_final(:,3+(i-1)*No_states);
    x_vel = state_matrix_final(:,4+(i-1)*No_states).*cos(theta);
    y_vel = state_matrix_final(:,4+(i-1)*No_states).*sin(theta);
%     x_acc = state_matrix_final(:,5+(i-1)*No_states);
%     y_acc = state_matrix_final(:,6+(i-1)*No_states);
    curv = kappa(x_vel,y_vel,x_acc,y_acc);
    plot(curv)
    %curv_list = [curv_list,curv];
end
hold off;

%% Plot solver time vs. function calls

figure()
plot(solverTime_list,'-o')
title('Solution time [s]');
xlabel('Function call');
ylabel('Solver time');

return;


%% END OF METHOD - BELOW IS JUST COMPARISONS


%% Matlab RRT w/ smoothing cubic splines:
%ss = stateSpaceDubins; % create state-space
ss = stateSpaceReedsShepp;
%ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss); % Create an occupanyMap-based state validator using the created state space.

% Create an occupany map from an example map and set map resolution as 10 cells/meter.
load exampleMaps
%map = occupancyMap(simpleMap,0.6*cell_param); %decrease 2nd argument to increase map size

%map = occupancyMap(20,20,10);
sv.Map = map;
% Set validation distance for the validator.
sv.ValidationDistance = 1;
% Update state space bounds to be the same as map limits.
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
% Create the path planner and increase max connection distance.
planner = plannerRRTStar(ss,sv);
planner.MaxIterations = 50000;
planner.MaxNumTreeNodes = 3000;
planner.ContinueAfterGoalReached = true;
%planner.MaxConnectionDistance = 0.3; 0.6
planner.MaxConnectionDistance = 100; % 0.3*(10/cell_param) , 5
ss.MinTurningRadius = (1/K_max)*1.2;
ss.ReverseCost = 10;
% Set the start and goal states.
start = [pos_i(1),pos_i(2),yaw_i];
%global goal;
goal = [pos_end(1),pos_end(2),yaw_end];
% Plan a path with default settings.
[pthObj,solnInfo] = plan(planner,start,goal);

reedsConnObj = reedsSheppConnection; RS_costs = []; pathSegObj_list =[];
reedsConnObj.MinTurningRadius = (1/K_max)*1.2;

for i=1:size(pthObj.States,1)-1
    state1 = pthObj.States(i,:)
    state2 = pthObj.States(i+1,:)
    [pathSegObj,pathCosts] = connect(reedsConnObj,state1,state2);
    RS_cost = [RS_costs,pathCosts];
    pathSegObj_list = [pathSegObj_list,pathSegObj{1}]
end

rng(100,'twister'); % for repeatable result
% Visualize the results.
figure()
show(map)
hold on;
for i=1:size(pthObj.States,1)-1
    show(pathSegObj_list(i))
end
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path
hold off;

%%
RRT_x = pthObj.States(:,1); %Dubin/RRT* seems to use hard end constraint, can leave out last point (end-1)
RRT_y = pthObj.States(:,2);

waypoints = [RRT_x, RRT_y];

refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);

initState = [0 0 0 0 0 0];  % [S ds ddS L dL ddL]
termState = [10 0 0 0 0 0]; % [S ds ddS L dL ddL]
[~,trajGlobal] = connect(connector,initState,termState,5);

show(refPath);
hold on
axis equal
plot(trajGlobal.Trajectory(:,1),trajGlobal.Trajectory(:,2),'b')
legend(["Waypoints","Reference Path","Trajectory to 30m"])


%%

dubConnObj = dubinsConnection;
%Define start and goal poses as [x y theta] vectors.

startPose = [0 0 0];
goalPose = [1 1 pi];

%Calculate a valid path segment to connect the poses.
tic
pathSegObj = connect(dubConnObj,startPose,goalPose);
toc


state_i = [0,0,60*pi/180,8];
state_f = [50,10,60*pi/180,8];

obj = 0; % obj not used
N = 7;
global c0 c1 c2 c3 c4
figure()
hold on;
c0 = 0; c1 = 0; c2 = 1; c3 = 1; c4 = 0;
%c0 = 0; c1 = 0; c2 = 1; c3 = 0; c4 = 0;
solved_dist_map = containers.Map();
state_f(end) = 0;
global speed
speed = 5;
[dist1,x1,y1,T1,solverStatus] = Steer_Dubin_costs(obj,state_i,state_f,N)
time_span1 = linspace(0,T1,100);
plot(double(x1{1,1}(time_span1)),double(y1{1,1}(time_span1)),'r','DisplayName',strcat('$v_d =$ ',num2str(speed)))
%c0 = 0; c1 = 0; c2 = 0; c3 = 1; c4 = 0;
solved_dist_map = containers.Map();
%state_f(end) = 5;
speed = 7;
[dist2,x2,y2,T2,solverStatus] = Steer_Dubin_costs(obj,state_i,state_f,N)
time_span2 = linspace(0,T2,100);
plot(double(x2{1,1}(time_span2)),double(y2{1,1}(time_span2)),'m','DisplayName',strcat('$v_d =$ ',num2str(speed)))
%c0 = 0; c1 = 0; c2 = 1; c3 = 1; c4 = 0;
solved_dist_map = containers.Map();
%state_f(end) = 8;
speed = 9;
[dist3,x3,y3,T3,solverStatus] = Steer_Dubin_costs(obj,state_i,state_f,N)
time_span3 = linspace(0,T3,100);
plot(double(x3{1,1}(time_span3)),double(y3{1,1}(time_span3)),'b','DisplayName',strcat('$v_d =$ ',num2str(speed)))
hold off;
ax = gca;
legendFS = 20; axFS = 24;
legend('FontSize',legendFS);
ax.FontSize = axFS; 
xlabel('$x$ [m]')
ylabel('$y$ [m]')
%axis equal


