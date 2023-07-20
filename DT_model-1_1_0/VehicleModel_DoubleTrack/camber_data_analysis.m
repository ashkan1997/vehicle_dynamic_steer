function camber_data_analysis(model_sim_camber,vehicle_data,Ts , camber_ang)

    % ----------------------------------------------------------------
    %% Post-Processing and Data Analysis
    % ----------------------------------------------------------------
    
    % ---------------------------------
    %% Load vehicle data
    % ---------------------------------
    Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
    Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
    L  = vehicle_data.vehicle.L;   % [m] Vehicle length
    Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
    Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
    m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
    g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
    tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)
    h_rr = vehicle_data.front_suspension.h_rc_f;
    h_rf = vehicle_data.rear_suspension.h_rc_r;
    h_r = h_rr + (h_rf - h_rr)*Lr/(L);
    h_s = vehicle_data.vehicle.hGs;
    h_G = h_r + h_s;
    Ks_f = vehicle_data.front_suspension.Ks_f;
    Ks_r = vehicle_data.rear_suspension.Ks_r;
    eps_roll = (Ks_f)/(Ks_f + Ks_r);
    
    % ---------------------------------
    %% Extract data from simulink model
    % ---------------------------------
    time_sim = model_sim_camber.v0.states.u.time;
    dt = time_sim(2)-time_sim(1);
    time_cut = 20;
    
    % -----------------
    % Inputs
    % -----------------
    ped_0      = model_sim_camber.v0.inputs.ped_0.data;
    delta_D    = model_sim_camber.v0.inputs.delta_D.data;
    
    % -----------------
    % States
    % -----------------
   
    % x_CoM      = model_sim_camber.states.x.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        x_CoM.(my_field) = model_sim_camber.(my_field).states.x.data;
    end

    % y_CoM      = model_sim_camber.states.y.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        y_CoM.(my_field) = model_sim_camber.(my_field).states.y.data;
    end

    % psi        = model_sim_camber.states.psi.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        psi.(my_field) = model_sim_camber.(my_field).states.psi.data;
    end

    % u          = model_sim_camber.states.u.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        u.(my_field) = model_sim_camber.(my_field).states.u.data;
    end

    % v          = model_sim_camber.states.v.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        v.(my_field) = model_sim_camber.(my_field).states.v.data;
    end

    % Omega      = model_sim_camber.states.Omega.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Omega.(my_field) = model_sim_camber.(my_field).states.Omega.data;
    end

    % Fz_rr      = model_sim_camber.states.Fz_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fz_rr.(my_field) = model_sim_camber.(my_field).states.Fz_rr.data;
    end

    % Fz_rl      = model_sim_camber.states.Fz_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fz_rl.(my_field) = model_sim_camber.(my_field).states.Fz_rl.data;
    end

    % Fz_fr      = model_sim_camber.states.Fz_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fz_fr.(my_field) = model_sim_camber.(my_field).states.Fz_fr.data;
    end

    % Fz_fl      = model_sim_camber.states.Fz_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fz_fl.(my_field) = model_sim_camber.(my_field).states.Fz_fl.data;
    end

    % delta      = model_sim_camber.states.delta.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        delta.(my_field) = model_sim_camber.(my_field).states.delta.data;
    end

    % omega_rr   = model_sim_camber.states.omega_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        omega_rr.(my_field) = model_sim_camber.(my_field).states.omega_rr.data;
    end

    % omega_rl   = model_sim_camber.states.omega_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        omega_rl.(my_field) = model_sim_camber.(my_field).states.omega_rl.data;
    end

    % omega_fr   = model_sim_camber.states.omega_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        omega_fr.(my_field) = model_sim_camber.(my_field).states.omega_fr.data;
    end

    % omega_fl   = model_sim_camber.states.omega_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        omega_fl.(my_field) = model_sim_camber.(my_field).states.omega_fl.data;
    end

    % alpha_rr   = model_sim_camber.states.alpha_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        alpha_rr.(my_field) = model_sim_camber.(my_field).states.alpha_rr.data;
    end

    % alpha_rl   = model_sim_camber.states.alpha_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        alpha_rl.(my_field) = model_sim_camber.(my_field).states.alpha_rl.data;
    end

    % alpha_fr   = model_sim_camber.states.alpha_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        alpha_fr.(my_field) = model_sim_camber.(my_field).states.alpha_fr.data;
    end

    % alpha_fl   = model_sim_camber.states.alpha_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        alpha_fl.(my_field) = model_sim_camber.(my_field).states.alpha_fl.data;
    end

    % kappa_rr   = model_sim_camber.states.kappa_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        kappa_rr.(my_field) = model_sim_camber.(my_field).states.kappa_rr.data;
    end

    % kappa_rl   = model_sim_camber.states.kappa_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        kappa_rl.(my_field) = model_sim_camber.(my_field).states.kappa_rl.data;
    end

    % kappa_fr   = model_sim_camber.states.kappa_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        kappa_fr.(my_field) = model_sim_camber.(my_field).states.kappa_fr.data;
    end

    % kappa_fl   = model_sim_camber.states.kappa_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        kappa_fl.(my_field) = model_sim_camber.(my_field).states.kappa_fl.data;
    end


    % -----------------
    % Extra Parameters
    % -----------------
    % Tw_rr      = model_sim_camber.extra_params.Tw_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Tw_rr.(my_field) = model_sim_camber.(my_field).extra_params.Tw_rr.data;
    end

    % Tw_rl      = model_sim_camber.(my_field).extra_params.Tw_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Tw_rl.(my_field) = model_sim_camber.(my_field).extra_params.Tw_rl.data;
    end

    % Tw_fr      = model_sim_camber.(my_field).extra_params.Tw_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Tw_fr.(my_field) = model_sim_camber.(my_field).extra_params.Tw_fr.data;
    end

    % Tw_fl      = model_sim_camber.(my_field).extra_params.Tw_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Tw_fl.(my_field) = model_sim_camber.(my_field).extra_params.Tw_fl.data;
    end

    % Fx_rr      = model_sim_camber.(my_field).extra_params.Fx_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fx_rr.(my_field) = model_sim_camber.(my_field).extra_params.Fx_rr.data;
    end

    % Fx_rl      = model_sim_camber.(my_field).extra_params.Fx_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fx_rl.(my_field) = model_sim_camber.(my_field).extra_params.Fx_rl.data;
    end

    % Fx_fr      = model_sim_camber.(my_field).extra_params.Fx_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fx_fr.(my_field) = model_sim_camber.(my_field).extra_params.Fx_fr.data;
    end

    % Fx_fl      = model_sim_camber.(my_field).extra_params.Fx_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fx_fl.(my_field) = model_sim_camber.(my_field).extra_params.Fx_fl.data;
    end

    % Fy_rr      = model_sim_camber.(my_field).extra_params.Fy_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fy_rr.(my_field) = model_sim_camber.(my_field).extra_params.Fy_rr.data;
    end

    % Fy_rl      = model_sim_camber.(my_field).extra_params.Fy_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fy_rl.(my_field) = model_sim_camber.(my_field).extra_params.Fy_rl.data;
    end

    % Fy_fr      = model_sim_camber.(my_field).extra_params.Fy_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fy_fr.(my_field) = model_sim_camber.(my_field).extra_params.Fy_fr.data;
    end

    % Fy_fl      = model_sim_camber.(my_field).extra_params.Fy_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Fy_rr.(my_field) = model_sim_camber.(my_field).extra_params.Fy_rr.data;
    end

    % Mz_rr      = model_sim_camber.(my_field).extra_params.Mz_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Mz_rr.(my_field) = model_sim_camber.(my_field).extra_params.Mz_rr.data;
    end

    % Mz_rl      = model_sim_camber.(my_field).extra_params.Mz_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Mz_rl.(my_field) = model_sim_camber.(my_field).extra_params.Mz_rl.data;
    end

    % Mz_fr      = model_sim_camber.(my_field).extra_params.Mz_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Mz_fr.(my_field) = model_sim_camber.(my_field).extra_params.Mz_fr.data;
    end

    % Mz_fl      = model_sim_camber.(my_field).extra_params.Mz_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Mz_fl.(my_field) = model_sim_camber.(my_field).extra_params.Mz_fl.data;
    end

    % gamma_rr   = model_sim_camber.(my_field).extra_params.gamma_rr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        gamma_rr.(my_field) = model_sim_camber.(my_field).extra_params.gamma_rr.data;
    end

    % gamma_rl   = model_sim_camber.(my_field).extra_params.gamma_rl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        gamma_rl.(my_field) = model_sim_camber.(my_field).extra_params.gamma_rl.data;
    end

    % gamma_fr   = model_sim_camber.(my_field).extra_params.gamma_fr.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        gamma_fr.(my_field) = model_sim_camber.(my_field).extra_params.gamma_fr.data;
    end

    % gamma_fl   = model_sim_camber.(my_field).extra_params.gamma_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        gamma_fl.(my_field) = model_sim_camber.(my_field).extra_params.gamma_fl.data;
    end

    % delta_fr   = model_sim_camber.(my_field).extra_params.delta_fr.data; 
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        delta_fr.(my_field) = model_sim_camber.(my_field).extra_params.delta_fr.data;
    end

    % delta_fl   = model_sim_camber.(my_field).extra_params.delta_fl.data;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        delta_fl.(my_field) = model_sim_camber.(my_field).extra_params.delta_fl.data;
    end


    % Chassis side slip angle beta [rad]
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        beta.(my_field) = atan(v.(my_field)./u.(my_field));
    end

    

    % -----------------
    % Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    % dot_u = diff(u)/Ts;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        dot_u.(my_field) = diff(u.(my_field))/Ts;
    end

    % dot_v = diff(v)/Ts;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        dot_v.(my_field) = diff(v.(my_field))/Ts;
    end

    % Total longitudinal and lateral accelerations
    % Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Ax.(my_field) = dot_u.(my_field)(1:end) - Omega.(my_field)(2:end).*v.(my_field)(2:end);
    end

    % Ay = dot_v(1:end) + Omega(2:end).*u(2:end);
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Ax.(my_field) = dot_v.(my_field)(1:end) - Omega.(my_field)(2:end).*u.(my_field)(2:end);
    end

    % Ax low-pass filtered signal (zero-phase digital low-pass filtering)
    Wn_filter = 0.01;
    [b_butt,a_butt] = butter(4,Wn_filter,'low');
    % Ax_filt = filtfilt(b_butt,a_butt,Ax);  
    % dot_u_filt = filtfilt(b_butt,a_butt,dot_u);  
    % Steady state lateral acceleration
    % Ay_ss = Omega.*u;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        Ay_ss.(my_field) = Omega.(my_field).*u.(my_field);
    end

    % Ay_filt = filtfilt(b_butt,a_butt,Ay_ss); 
    % Longitudinal jerk [m/s^3]
    
    % jerk_x = diff(dot_u)/Ts;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        jerk_x.(my_field) = diff(dot_u.(my_field))/Ts;
    end



    % -----------------
    % Other parameters
    % -----------------
    % Total CoM speed [m/s]

    % vG = sqrt(u.^2 + v.^2);
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        vG.(my_field) = sqrt(u.(my_field).^2 + v.(my_field).^2);
    end

    % Steady state and transient curvature [m]

    % rho_ss   = Omega./vG;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        rho_ss.(my_field) = Omega.(my_field)./vG.(my_field);
    end

    % rho_tran = ((dot_v.*u(1:end-1) - dot_u.*v(1:end-1)) ./ ((vG(1:end-1)).^3)) + rho_ss(1:end-1);
    % Desired sinusoidal steering angle for the equivalent single track front wheel
    desired_steer_atWheel = delta_D/tau_D;
    

    %% alpha

    % alpha_r = 0.5*(alpha_rl+alpha_rr);
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        alpha_r.(my_field) = 0.5*(alpha_rl.(my_field)+alpha_rr.(my_field));
    end

    % alpha_f = 0.5*(alpha_fl+alpha_fr);
    
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        alpha_f.(my_field) = 0.5*(alpha_fl.(my_field)+alpha_fr.(my_field));
    end


    % dalpha = alpha_r - alpha_f;
    for i = -camber_ang:camber_ang
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        dalpha.(my_field) =alpha_r.(my_field) - alpha_f.(my_field);
    end


    

    %% Plot

    %% Plot handling diagram
    figure('Name','Camber angle Handling Diagram')
    cmap = jet(camber_ang*2 + 1);
    j = 1;
    for i = -camber_ang:camber_ang
        
        if i<0
            my_field = strcat('v_min',num2str(abs(i)));
        else
            my_field = strcat('v',num2str(i));
        end
        
        plot(Ay_ss.(my_field)/g , dalpha.(my_field) , 'LineWidth', 2 , 'DisplayName', ['$\gamma =$ ' ,num2str(i)] , 'Color',cmap(j , :));
        hold on
        j = j+1;
    end
   
   
    ylabel('$\rho$L - $\delta$ = $\Delta \alpha$')
    xlabel('$\frac{A_{y}}{g}$')
    legend 

end