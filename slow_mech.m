
function [state1_new,state2_new,t_i_eps,t_f_eps,x_start,y_start,x_end,y_end,v_start,v_end] = slow_mech(state1,state2)

%% (!) SLOW MECHANISM SECTION:
            % set false to disable low-start mechanism
            % a_start, a_end is the constant accel.
            global a0
            
            a_i = a0; % constant accel.
            a_f = -a0;
            global v_eps
            % constant accel: other forms also possible
            
            x_i = state1(1); y_i = state1(2); theta_i = state1(3); v_i = state1(4);
            x_f = state2(1); y_f = state2(2); theta_f = state2(3); v_f = state2(4);
            
            x_start = @(t) 0.5*a_i*cos(theta_i)*t.^2+v_i*cos(theta_i)*t+x_i;
            y_start = @(t) 0.5*a_i*sin(theta_i)*t.^2+v_i*sin(theta_i)*t+y_i;
            x_end = @(t) 0.5*a_f*cos(theta_f)*t.^2+v_f*cos(theta_f)*t+x_f;
            y_end = @(t) 0.5*a_f*sin(theta_f)*t.^2+v_f*sin(theta_f)*t+y_f;
            v_x_start = @(t) a_i*cos(theta_i)*t+v_i*cos(theta_i);
            v_y_start = @(t) a_i*sin(theta_i)*t+v_i*sin(theta_i);
            v_x_end = @(t) a_f*cos(theta_f)*t+v_f*cos(theta_f);
            v_y_end = @(t) a_f*sin(theta_f)*t+v_f*sin(theta_f);
            v_start = @(t) sqrt(v_x_start(t).^2+v_y_start(t).^2);
            v_end = @(t) sqrt(v_x_end(t).^2+v_y_end(t).^2);
            
            t_i_eps = (v_eps-v_i)/a0; % this assumes both speeds are lower than v_eps
            t_f_eps = (v_eps-v_f)/a0;
              
            
            if v_i < v_eps
                state1_new = [x_start(t_i_eps),y_start(t_i_eps),theta_i,v_eps,a0*cos(theta_i),a0*sin(theta_i)];
            else
                state1_new = state1;
            end
            if v_f < v_eps
                state2_new = [x_end(-t_f_eps),y_end(-t_f_eps),theta_f,v_eps,-a0*cos(theta_f),-a0*sin(theta_f)];
            else
                state2_new = state2;
            end
            
end