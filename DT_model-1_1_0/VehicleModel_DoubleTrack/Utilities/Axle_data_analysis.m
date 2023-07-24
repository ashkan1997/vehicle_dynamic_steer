function Axle_data_analysis(model_sim_Ks,vehicle_data,Ts , eps_Ks_init)

    % ----------------------------------------------------------------
    %% Post-Processing and Data Analysis
    % ----------------------------------------------------------------
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

    % ---------------------------------
    %% Extract data from simulink model
    % ---------------------------------
    time_sim = model_sim_Ks.eps_Ks_1.states.u.time;
    dt = time_sim(2)-time_sim(1);
    time_cut = 20;

    % -----------------
    % Inputs
    % -----------------
    ped_0      = model_sim_Ks.eps_Ks_1.inputs.ped_0.data;
    delta_D    = model_sim_Ks.eps_Ks_1.inputs.delta_D.data;

    % -----------------
    % States
    % -----------------

    % x_CoM      = model_sim.states.x.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        x_CoM.(my_field) = model_sim_Ks.(my_field).states.x.data;
    end

    % y_CoM      = model_sim.states.y.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        y_CoM.(my_field) = model_sim_Ks.(my_field).states.y.data;
    end

    % psi        = model_sim.states.psi.data;

    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        psi.(my_field) = model_sim_Ks.(my_field).states.psi.data;
    end

    % u          = model_sim.states.u.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        u.(my_field) = model_sim_Ks.(my_field).states.u.data;
    end

    % v          = model_sim.states.v.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        v.(my_field) = model_sim_Ks.(my_field).states.v.data;
    end

    % Omega      = model_sim.states.Omega.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Omega.(my_field) = model_sim_Ks.(my_field).states.Omega.data;
    end

    % Fz_rr      = model_sim.states.Fz_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fz_rr.(my_field) = model_sim_Ks.(my_field).states.Fz_rr.data;
    end

    % Fz_rl      = model_sim.states.Fz_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fz_rl.(my_field) = model_sim_Ks.(my_field).states.Fz_rl.data;
    end

    % Fz_fr      = model_sim.states.Fz_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fz_fr.(my_field) = model_sim_Ks.(my_field).states.Fz_fr.data;
    end

    % Fz_fl      = model_sim.states.Fz_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fz_fl.(my_field) = model_sim_Ks.(my_field).states.Fz_fl.data;
    end

    % delta      = model_sim.states.delta.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        delta.(my_field) = model_sim_Ks.(my_field).states.delta.data;
    end

    % omega_rr   = model_sim.states.omega_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        omega_rr.(my_field) = model_sim_Ks.(my_field).states.omega_rr.data;
    end

    % omega_rl   = model_sim.states.omega_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        omega_rl.(my_field) = model_sim_Ks.(my_field).states.omega_rl.data;
    end

    % omega_fr   = model_sim.states.omega_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        omega_fr.(my_field) = model_sim_Ks.(my_field).states.omega_fr.data;
    end

    % omega_fl   = model_sim.states.omega_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        omega_fl.(my_field) = model_sim_Ks.(my_field).states.omega_fl.data;
    end

    % alpha_rr   = model_sim.states.alpha_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        alpha_rr.(my_field) = model_sim_Ks.(my_field).states.alpha_rr.data;
    end

    % alpha_rl   = model_sim.states.alpha_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        alpha_rl.(my_field) = model_sim_Ks.(my_field).states.alpha_rl.data;
    end

    % alpha_fr   = model_sim.states.alpha_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        alpha_fr.(my_field) = model_sim_Ks.(my_field).states.alpha_fr.data;
    end

    % alpha_fl   = model_sim.states.alpha_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        alpha_fl.(my_field) = model_sim_Ks.(my_field).states.alpha_fl.data;
    end

    % kappa_rr   = model_sim.states.kappa_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        kappa_rr.(my_field) = model_sim_Ks.(my_field).states.kappa_rr.data;
    end

    % kappa_rl   = model_sim.states.kappa_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        kappa_rl.(my_field) = model_sim_Ks.(my_field).states.kappa_rl.data;
    end

    % kappa_fr   = model_sim.states.kappa_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        kappa_fr.(my_field) = model_sim_Ks.(my_field).states.kappa_fr.data;
    end

    % kappa_fl   = model_sim.states.kappa_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        kappa_fl.(my_field) = model_sim_Ks.(my_field).states.kappa_fl.data;
    end

    % -----------------
    % Extra Parameters
    % -----------------
    % Tw_rr      = model_sim.extra_params.Tw_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Tw_rr.(my_field) = model_sim_Ks.(my_field).extra_params.Tw_rr.data;
    end

    % Tw_rl      = model_sim.extra_params.Tw_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Tw_rl.(my_field) = model_sim_Ks.(my_field).extra_params.Tw_rl.data;
    end

    % Tw_fr      = model_sim.extra_params.Tw_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Tw_fr.(my_field) = model_sim_Ks.(my_field).extra_params.Tw_fr.data;
    end

    % Tw_fl      = model_sim.extra_params.Tw_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Tw_fl.(my_field) = model_sim_Ks.(my_field).extra_params.Tw_fl.data;
    end

    % Fx_rr      = model_sim.extra_params.Fx_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fx_rr.(my_field) = model_sim_Ks.(my_field).extra_params.Fx_rr.data;
    end

    % Fx_rl      = model_sim.extra_params.Fx_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fx_rl.(my_field) = model_sim_Ks.(my_field).extra_params.Fx_rl.data;
    end

    % Fx_fr      = model_sim.extra_params.Fx_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fx_fr.(my_field) = model_sim_Ks.(my_field).extra_params.Fx_fr.data;
    end

    % Fx_fl      = model_sim.extra_params.Fx_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fx_fl.(my_field) = model_sim_Ks.(my_field).extra_params.Fx_fl.data;
    end

    % Fy_rr      = model_sim.extra_params.Fy_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fy_rr.(my_field) = model_sim_Ks.(my_field).extra_params.Fy_rr.data;
    end

    % Fy_rl      = model_sim.extra_params.Fy_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fy_rl.(my_field) = model_sim_Ks.(my_field).extra_params.Fy_rl.data;
    end

    % Fy_fr      = model_sim.extra_params.Fy_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fy_fr.(my_field) = model_sim_Ks.(my_field).extra_params.Fy_fr.data;
    end

    % Fy_fl      = model_sim.extra_params.Fy_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Fy_fl.(my_field) = model_sim_Ks.(my_field).extra_params.Fy_fl.data;
    end

    % Mz_rr      = model_sim.extra_params.Mz_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Mz_rr.(my_field) = model_sim_Ks.(my_field).extra_params.Mz_rr.data;
    end

    % Mz_rl      = model_sim.extra_params.Mz_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Mz_rl.(my_field) = model_sim_Ks.(my_field).extra_params.Mz_rl.data;
    end

    % Mz_fr      = model_sim.extra_params.Mz_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Mz_fr.(my_field) = model_sim_Ks.(my_field).extra_params.Mz_fr.data;
    end

    % Mz_fl      = model_sim.extra_params.Mz_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Mz_fl.(my_field) = model_sim_Ks.(my_field).extra_params.Mz_fl.data;
    end

    % gamma_rr   = model_sim.extra_params.gamma_rr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        gamma_rr.(my_field) = model_sim_Ks.(my_field).extra_params.gamma_rr.data;
    end

    % gamma_rl   = model_sim.extra_params.gamma_rl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        gamma_rl.(my_field) = model_sim_Ks.(my_field).extra_params.gamma_rl.data;
    end

    % gamma_fr   = model_sim.extra_params.gamma_fr.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        gamma_fr.(my_field) = model_sim_Ks.(my_field).extra_params.gamma_fr.data;
    end

    % gamma_fl   = model_sim.extra_params.gamma_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        gamma_fl.(my_field) = model_sim_Ks.(my_field).extra_params.gamma_fl.data;
    end

    % delta_fr   = model_sim.extra_params.delta_fr.data; 
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        delta_fr.(my_field) = model_sim_Ks.(my_field).extra_params.delta_fr.data;
    end

    % delta_fl   = model_sim.extra_params.delta_fl.data;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        delta_fl.(my_field) = model_sim_Ks.(my_field).extra_params.delta_fl.data;
    end

    % Chassis side slip angle beta [rad]
    % beta = atan(v./u);
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        beta.(my_field) = atan(v.(my_field)./u.(my_field));
    end

    % -----------------
    % Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    % dot_u = diff(u)/Ts;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        dot_u.(my_field) = diff(u.(my_field))/Ts;
    end

    % dot_v = diff(v)/Ts;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        dot_v.(my_field) = diff(v.(my_field))/Ts;
    end

    % % Total longitudinal and lateral accelerations
    % Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
    % Ay = dot_v(1:end) + Omega(2:end).*u(2:end);

    % % Ax low-pass filtered signal (zero-phase digital low-pass filtering)
    % Wn_filter = 0.01;
    % [b_butt,a_butt] = butter(4,Wn_filter,'low');
    % Ax_filt = filtfilt(b_butt,a_butt,Ax);  
    % dot_u_filt = filtfilt(b_butt,a_butt,dot_u); 

    % Steady state lateral acceleration
    % Ay_ss = Omega.*u;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        Ay_ss.(my_field) = Omega.(my_field).*u.(my_field);
    end

    % Ay_filt = filtfilt(b_butt,a_butt,Ay_ss); 
    % Longitudinal jerk [m/s^3]
    % jerk_x = diff(dot_u)/Ts;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        jerk_x.(my_field) = diff(dot_u.(my_field))/Ts;
    end

    
    % -----------------
    % Other parameters
    % -----------------
    % % Total CoM speed [m/s]
    % vG = sqrt(u.^2 + v.^2);
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        vG.(my_field) = sqrt(u.(my_field).^2 + v.(my_field).^2);
    end

    % Steady state and transient curvature [m]
    % rho_ss   = Omega./vG;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        rho_ss.(my_field) = Omega.(my_field)./vG.(my_field);
    end

    % rho_tran = ((dot_v.*u(1:end-1) - dot_u.*v(1:end-1)) ./ ((vG(1:end-1)).^3)) + rho_ss(1:end-1);
    % Desired sinusoidal steering angle for the equivalent single track front wheel
    desired_steer_atWheel = delta_D/tau_D;

    %% alpha

    % alpha_r = 0.5*(alpha_rl+alpha_rr);
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        alpha_r.(my_field) = 0.5*(alpha_rl.(my_field)+alpha_rr.(my_field));
    end
    
    % alpha_f = 0.5*(alpha_fl+alpha_fr);
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        alpha_f.(my_field) = 0.5*(alpha_fl.(my_field)+alpha_fr.(my_field));
    end

    % dalpha = alpha_r - alpha_f;
    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        dalpha.(my_field) = alpha_r.(my_field) - alpha_f.(my_field);
    end

    
     %% Plot

    %% Plot handling diagram
    
    figure('Name','Ks Handling Diagram')
    cmap = jet(length(eps_Ks_init) + 1);

    for i=1:length(eps_Ks_init)
        my_field = strcat('eps_Ks_',num2str(i));
        plot(Ay_ss.(my_field)/g , -dalpha.(my_field) , 'LineWidth', 2 , 'DisplayName', ['$\epsilon =$ ' ,num2str(0.1*i)] , 'Color',cmap(i,:));
        hold on
        
    end
   
   
    ylabel('$\delta$ - $\rho$L = $ - \Delta \alpha$')
    xlabel('$\frac{A_{y}}{g}$')
    xlim([0,0.7])
    legend(Location='best')
    title('Handling diagrams for different front suspension stiffnesses ')
    


end