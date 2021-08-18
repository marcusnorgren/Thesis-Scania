function [u1_vec,u2_vec] = get_Inputs_and_Plot(poly_x,poly_y,time_list,alpha_i,beta_i)

%% obtain inputs:
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

M = 1; % single segment

input_u1 = cell(M,1);
input_u2 = cell(M,1);
curv_K = cell(M,1);
phi = cell(M,1);


T = time_list(end);
time_list = linspace(0,T,3000); % take finer than input, for simulation.
x = poly_x; y = poly_y; t_lists = time_list;

u1 = @(vel_x,vel_y,alpha) ((vel_x ~= 0).*sign(vel_x.*cos(alpha)) + (vel_x == 0).*sign(vel_y.*sin(alpha))).*sqrt(vel_x.^2+vel_y.^2); % signed velocity indicating forward or reverse motion

for i=1:M
    
    
    input_u2{i} = @(t) L*((y{4,i}(t).*x{2,i}(t)-y{2,i}(t).*x{4,i}(t)).*(x{2,i}(t).^2+y{2,i}(t).^2).^(3/2)-(y{3,i}(t).*x{2,i}(t)-y{2,i}(t).*x{3,i}(t))*(3/2).*(x{2,i}(t).^2+y{2,i}(t).^2).^(1/2).*(2*x{2,i}(t).*x{3,i}(t)+2*y{2,i}(t).*y{3,i}(t)))...
        ./((x{2,i}(t).^2+y{2,i}(t).^2).^3+(L^2)*(y{3,i}(t).*x{2,i}(t)-y{2,i}(t).*x{3,i}(t)).^2);

    curv_K{i} = @(t) (y{3,i}(t).*x{2,i}(t)-y{2,i}(t).*x{3,i}(t))./((x{2,i}(t).^2+y{2,i}(t).^2).^(3/2));
    phi{i} = @(t) atan2(y{2,i}(t),x{2,i}(t)); % heading angle
    
    
    %input_u1{i} = @(t) u1(vel_x(t),vel_y(t),phi{i}(t));
end
vel_x = @(t) x{2,1}(t); vel_y = @(t) y{2,1}(t);
vel_x_vec = double(vel_x(t_lists(1,:))); vel_y_vec = double(vel_y(t_lists(1,:))); alpha_vec = double(phi{1}(t_lists(1,:)));
%K_values = zeros(N_steps,1);

u1_vec = (u1(vel_x_vec,vel_y_vec,alpha_vec))'; % from function above

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
fs = 24;
xlabel('time [s]','FontSize',fs)
%ylabel('curvature [m$^{-1}$]','FontSize',fs)
title('Heading angle','FontSize',fs)
legend('show','FontSize',22);

%% Simulate dynamics using inputs:
t_list_sim = reshape(t_lists',size(t_lists,1)*size(t_lists,2),1);
N_steps = size(t_list_sim,1);
x_vec = zeros(N_steps,1);
y_vec = x_vec;
phi_vec = x_vec;
alpha_vec = x_vec;
beta_vec = x_vec;
beta_vec(1)=beta_i; %initial trailer heading (i.e. along x-axis is pi)
%test:
beta_vec(1) = 1.8892;

u2_vec = x_vec;
x_trailer=x_vec;
y_trailer=x_vec;
% initial conditions:
pos_i = double([x{1,1}(0),y{1,1}(0)]);
x_vec(1) = pos_i(1); 
y_vec(1)= pos_i(2);
alpha_vec(1) = alpha_i; 
x_trailer(1)=x_vec(1)-(a*cos(alpha_vec(1))+b*cos(pi-beta_vec(1)));
y_trailer(1)=y_vec(1)-(a*sin(alpha_vec(1))-b*sin(pi-beta_vec(1)));

T = time_list(end);
dt = T/N_steps;
u1_list = zeros(M,size(t_lists,2));
u2_list = zeros(M,size(t_lists,2));
for i=1:M
    %u1_list(i,:)=input_u1{i}(t_lists(i,:));
    u2_list(i,:)=input_u2{i}(t_lists(i,:));
end
%u1_vec = reshape(u1_list',size(t_lists,1)*size(t_lists,2),1);
u2_vec = reshape(u2_list',size(t_lists,1)*size(t_lists,2),1);

u2_max_list = max(abs(u2_list),[],2);

%end % end of time allocation iteration
%return;

%% integrate system dynamics using input u1, u2


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

%% Solve for trailer heading in backward recursion
beta_vec_sim = zeros(N_steps,1);
beta_0 = pi+0; % actual initial trailer heading (0 deg)
beta_vec_sim(end) = beta_0; 
for k = N_steps-1:-1:1
    % solve for the beta heading angle:
    beta_k_plus1 = beta_vec_sim(k+1);
    dt = abs(t_list_2(k+1)-t_list_2(k));
    FUN = @(beta_k) -beta_k_plus1+beta_k+dt*(u1_vec(k))*(1/b)*((a/L)*tan(phi_vec(k))*cos(alpha_vec(k)-beta_k)-sin(alpha_vec(k)-beta_k));

    X0 = pi+alpha_vec(k); % initial guess same as tractor
    X_beta_k = fsolve(FUN,X0); 
    beta_vec_sim(k) = X_beta_k; % add beta to list for next iteration
end
x_trailer2 = zeros(N_steps,1); y_trailer2=x_trailer2;
x_trailer2(1)=x_vec(end)-(a*cos(0)+b*cos(pi-pi));
y_trailer2(1)=y_vec(end)-(a*sin(0)-b*sin(pi-pi));
for k = N_steps:-1:1
    x_trailer2(N_steps-k+2)=x_trailer2(N_steps-k+1)+dt*(-u1_vec(k))*((a/L)*tan(phi_vec(k))*sin(alpha_vec(k)-beta_vec_sim(k))+cos(alpha_vec(k)-beta_vec_sim(k)))*cos(beta_vec_sim(k));
    y_trailer2(N_steps-k+2)=y_trailer2(N_steps-k+1)+dt*(-u1_vec(k))*((a/L)*tan(phi_vec(k))*sin(alpha_vec(k)-beta_vec_sim(k))+cos(alpha_vec(k)-beta_vec_sim(k)))*sin(beta_vec_sim(k));
end

x_trailer3=x_vec-(a*cos(alpha_vec)+b*cos(pi-beta_vec_sim));
y_trailer3=y_vec-(a*sin(alpha_vec)-b*sin(pi-beta_vec_sim));


figure()
hold on;
% plot(t_list_2,(180/pi)*fliplr(beta_vec_sim),'r')
% plot(t_list_2,(180/pi)*beta_vec,'b')
plot(x_trailer2,y_trailer2,'r-')
plot(x_trailer3,y_trailer3,'g-')
plot(x_trailer,y_trailer,'b-')
plot(x_vec,y_vec,'k-')
%plot(t_list_2,(180/pi)*beta_vec,'b')
hold off;


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
lw = 1.5;
figure()
hold on;
plot(x_vec,y_vec,'--','color','b','LineWidth',lw);
plot(x_trailer,y_trailer,'--','color','r','LineWidth',lw);
for i=1:M
    plot(x{1,i}(t_lists(i,:)),y{1,i}(t_lists(i,:)),'LineWidth',lw,'DisplayName','reference');
    %plot(RRT_x,RRT_y,'-o');
end

% plot(A_x',A_y','o');
% plot(B_x',B_y','*');
%hold off;
legendFS = 22; axFS = 24;
title('Position','FontSize',24);
xlabel('x [m]','FontSize',24);
ylabel('y [m]','FontSize',24);
legend('FontSize',legendFS);
ax = gca;
ax.FontSize = axFS; 
legend('show')
%return;
% plot moving object:

% Moving simulation:
% create polygon coordinates:
x_car = [A_x',B_x',C_x',D_x']';
y_car = [A_y',B_y',C_y',D_y']';
x_t = [A_x_t',B_x_t',C_x_t',D_x_t']';
y_t = [A_y_t',B_y_t',C_y_t',D_y_t']';

%g = hgtransform;
%figure()
%axis equal
%gif('moving_truck.gif')
for i=1:50:length(A_x)
patch('XData',x_car(:,i),'YData',y_car(:,i),'FaceColor','white');%,'Parent',g)
patch('XData',x_t(:,i),'YData',y_t(:,i),'FaceColor','white');%,'Parent',g)
pause(0.5)
drawnow
%gif
end
hold off;
axis equal;
end