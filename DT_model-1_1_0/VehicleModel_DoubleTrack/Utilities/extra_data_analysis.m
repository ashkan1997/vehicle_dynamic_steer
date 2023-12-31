function extra_data_analysis(model_sim,vehicle_data,Ts)

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
    time_sim = model_sim.states.u.time;
    dt = time_sim(2)-time_sim(1);
    time_cut = 20;
    
    % -----------------
    % Inputs
    % -----------------
    ped_0      = model_sim.inputs.ped_0.data;
    delta_D    = model_sim.inputs.delta_D.data;
    
    % -----------------
    % States
    % -----------------
    x_CoM      = model_sim.states.x.data;
    y_CoM      = model_sim.states.y.data;
    psi        = model_sim.states.psi.data;
    u          = model_sim.states.u.data;
    v          = model_sim.states.v.data;
    Omega      = model_sim.states.Omega.data;
    Fz_rr      = model_sim.states.Fz_rr.data;
    Fz_rl      = model_sim.states.Fz_rl.data;
    Fz_fr      = model_sim.states.Fz_fr.data;
    Fz_fl      = model_sim.states.Fz_fl.data;
    delta      = model_sim.states.delta.data;
    omega_rr   = model_sim.states.omega_rr.data;
    omega_rl   = model_sim.states.omega_rl.data;
    omega_fr   = model_sim.states.omega_fr.data;
    omega_fl   = model_sim.states.omega_fl.data;
    alpha_rr   = model_sim.states.alpha_rr.data;
    alpha_rl   = model_sim.states.alpha_rl.data;
    alpha_fr   = model_sim.states.alpha_fr.data;
    alpha_fl   = model_sim.states.alpha_fl.data;
    kappa_rr   = model_sim.states.kappa_rr.data;
    kappa_rl   = model_sim.states.kappa_rl.data;
    kappa_fr   = model_sim.states.kappa_fr.data;
    kappa_fl   = model_sim.states.kappa_fl.data;
    
    % -----------------
    % Extra Parameters
    % -----------------
    Tw_rr      = model_sim.extra_params.Tw_rr.data;
    Tw_rl      = model_sim.extra_params.Tw_rl.data;
    Tw_fr      = model_sim.extra_params.Tw_fr.data;
    Tw_fl      = model_sim.extra_params.Tw_fl.data;
    Fx_rr      = model_sim.extra_params.Fx_rr.data;
    Fx_rl      = model_sim.extra_params.Fx_rl.data;
    Fx_fr      = model_sim.extra_params.Fx_fr.data;
    Fx_fl      = model_sim.extra_params.Fx_fl.data;
    Fy_rr      = model_sim.extra_params.Fy_rr.data;
    Fy_rl      = model_sim.extra_params.Fy_rl.data;
    Fy_fr      = model_sim.extra_params.Fy_fr.data;
    Fy_fl      = model_sim.extra_params.Fy_fl.data;
    Mz_rr      = model_sim.extra_params.Mz_rr.data;
    Mz_rl      = model_sim.extra_params.Mz_rl.data;
    Mz_fr      = model_sim.extra_params.Mz_fr.data;
    Mz_fl      = model_sim.extra_params.Mz_fl.data;
    gamma_rr   = model_sim.extra_params.gamma_rr.data;
    gamma_rl   = model_sim.extra_params.gamma_rl.data;
    gamma_fr   = model_sim.extra_params.gamma_fr.data;
    gamma_fl   = model_sim.extra_params.gamma_fl.data;
    delta_fr   = model_sim.extra_params.delta_fr.data; 
    delta_fl   = model_sim.extra_params.delta_fl.data;
    
    % Chassis side slip angle beta [rad]
    beta = atan(v./u);
    
    % -----------------
    % Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    dot_u = diff(u)/Ts;
    dot_v = diff(v)/Ts;
    % Total longitudinal and lateral accelerations
    Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
    Ay = dot_v(1:end) + Omega(2:end).*u(2:end);
    % Ax low-pass filtered signal (zero-phase digital low-pass filtering)
    Wn_filter = 0.01;
    [b_butt,a_butt] = butter(4,Wn_filter,'low');
    Ax_filt = filtfilt(b_butt,a_butt,Ax);  
    dot_u_filt = filtfilt(b_butt,a_butt,dot_u);  
    % Steady state lateral acceleration
    Ay_ss = Omega.*u;
    Ay_filt = filtfilt(b_butt,a_butt,Ay_ss); 
    % Longitudinal jerk [m/s^3]
    jerk_x = diff(dot_u)/Ts;
    
    % -----------------
    % Other parameters
    % -----------------
    % Total CoM speed [m/s]
    vG = sqrt(u.^2 + v.^2);
    % Steady state and transient curvature [m]
    rho_ss   = Omega./vG;
    rho_tran = ((dot_v.*u(1:end-1) - dot_u.*v(1:end-1)) ./ ((vG(1:end-1)).^3)) + rho_ss(1:end-1);
    % Desired sinusoidal steering angle for the equivalent single track front wheel
    desired_steer_atWheel = delta_D/tau_D;
    
    %% ADDED PART
    %----------------------------
    %% Axle Characteristics
    %----------------------------

    % Experimental axle lateral force:
    Fy_r_data = Fy_rr + Fy_rl;
    Fy_f_data = Fy_fl + Fy_fr;
    
    % There is one to one relationship between lateral forces on each axle and the 
    % lateral acceleration via the lateral force balance equations:
    Fy_r_theor = m*Ay_ss*(Lf/L);
    Fy_f_theor = m*Ay_ss*(Lr/L);
    
    % Axle vertical force:
    Fz_f = Fz_fl + Fz_fr;
    Fz_r = Fz_rl + Fz_rr;
    
    % Theoretical adhesion coefficient:
    % mu_r_theor = Ay_ss/g;
    % mu_f_theor = Ay_ss/g;             % --> which one are correct?
    mu_r_theor = Fy_r_theor./Fz_r;      % --> which one are correct?
    mu_f_theor = Fy_f_theor./Fz_f;
    
    % Experimental adhesion coefficient:
    mu_r_data = Fy_r_data./Fz_r;
    mu_f_data = Fy_f_data./Fz_f;
    
    % Ay_norm = Ay_ss/g;
    Ay_norm = Ay_filt/g;
    Ay_norm_lin = linspace(0 , max(Ay_ss/g) , length(mu_f_data));
    
    % In SS we can consider the alpha of the axle to be the average between
    % the two tyre slips:
    alpha_r = 0.5*(alpha_rl+alpha_rr);
    alpha_f = 0.5*(alpha_fl+alpha_fr);
    
    alpha_r_lin = linspace(0,max(alpha_r) , length(mu_f_data));
    alpha_f_lin = linspace(0,max(alpha_f) , length(mu_f_data));

    %----------------------------
    %% Lateral Load transfer
    %----------------------------

    % Experimental:
    dFz_f = Fz_fr - Fz_fl;
    dFz_r = Fz_rr - Fz_rl;
    
    % Theoretical:
    dFz_f_theor = m*Ay_ss *( (Lr*h_rf/L/Wf) + eps_roll*h_s/Wf );
    dFz_r_theor = m*Ay_ss *( (Lf*h_rr/L/Wr) + h_s*(1-eps_roll)/Wr);
    
    %----------------------------
    %% Handling diagram
    %----------------------------

    delta_AK = rho_ss*L/tau_D;
    
    %----------------------------
    %% Understeering Gradient
    %---------------------------- 

    Kus_theor = (-m/ L /tau_D)*(Lf/Ks_r - Lr/Ks_f);
    oper_cond = rho_ss*L - deg2rad(desired_steer_atWheel);      % Will use dalpha instead of -dalpha so not  rho*L - delta = dAlpha but delta-rho*L = -dAlpha
    dalpha = alpha_r - alpha_f;    
    
    %% Extra calculations
    
    % Time indexing
    idx.time = time_sim>time_cut;
    time_sim_lin = time_sim(idx.time , :);
    
    % Ay_ss lin
    idx_ay = Ay_ss > 3.3;
    ay_ss_idx = Ay_ss(idx_ay,:);
    
    dalpha = deg2rad(dalpha);
   
    % Slope --> m = dy/dx
    Kus_fit = diff(dalpha(ceil(end*0.1):ceil(end*0.25)))./diff(Ay_ss(ceil(end*0.1):ceil(end*0.25)));
    Kus_fit(isinf(Kus_fit)|isnan(Kus_fit)) = 0;
    Kus_data = mean(Kus_fit);

    % Interpolation
    p2 = polyfit(Ay_ss(ceil(end*0.1):ceil(end*0.25)),dalpha(ceil(end*0.1):ceil(end*0.25)),1);
    disp(['Interpolated','Kus = ',num2str(-p2(1))])

    
    %Kus_theor = -(m/L/tau_D) * ((Lf/Ks_r)-(Lr/Ks_f));
    Kus_theor = -m/(L^2)*(Lf/Ks_r - Lr/Ks_f);

    fprintf('Fitted data Kus = %f \n' , -Kus_data);
    fprintf('Theoretical Kus = %f \n' , Kus_theor);

    
    
    %% Yaw rate gain
    
    % First Def:
    yaw_rate_gain_def1 = Omega./delta;

    % Second Def:
    yaw_rate_gain_def2 = u./(L*(1 + (-Kus_data) * u.^2));


   % In NS --> Kus = 0 --> Yaw_gain = u/L
    Omega_NS = u./L ;
    
    %% Acceleration gain

    Acc_gain = Ay_ss./delta;

    Acc_gain_NS = u.^2 ./L;

    %% Body slip gain

    beta_gain = beta./delta;
    
    % B/d = Lr/L - (ar*Lf + af*Lr)/L*d
    % But for NS => Dalpha = Da = 0 so subs Da = ar - af => ar = Da + af
    % Solving we get B/d = Lr/L - af/d

    beta_gain_neurtal = Lr / L - deg2rad(alpha_f)./delta;
    
    % -------------------------------
    %% Plot Lateral load transfer (t)
    % -------------------------------
    % figure('Name','Lateral Load Transfer [t]')
    % plot(time_sim , dFz_f , 'LineWidth',2 , 'DisplayName','$\Delta F_{zf} (a_y)$')
    % hold on
    % plot(time_sim , dFz_r , 'LineWidth',2 , 'DisplayName','$\Delta F_{zr} (a_y)$')
    % hold on
    % legend
    % xlabel('t [s]')
    % ylabel('$\Delta F_{z} [N]$')


    % -------------------------------
    %% Plot Lateral load transfer (Ay)
    % -------------------------------
    figure('Name','Lateral Load Transfer [Ay]')
    plot(Ay_ss , dFz_f , 'LineWidth',2 , 'DisplayName','$\Delta F_{zf} (a_y)$')
    hold on
    plot(Ay_ss , dFz_r , 'LineWidth',2 , 'DisplayName','$\Delta F_{zr} (a_y)$')
    hold on
    plot(Ay_ss , dFz_f_theor , 'LineWidth',2 , 'DisplayName','$\Delta F_{zf} Theory$' , 'LineStyle','--')
    hold on
    plot(Ay_ss , dFz_r_theor , 'LineWidth',2 , 'DisplayName','$\Delta F_{zr} Theory$' , 'LineStyle','--' )
    hold on
    legend(Location="best");
    title("Lateral Load Transfer for Speed Ramp Test in Steady State")
    xlabel('$A_y$')
    ylabel('$\Delta F_{z}$')


    % -------------------------------
    %% Plot Axle Characteristics
    % -------------------------------
    figure('Name','Axle Characteristics')
    plot(alpha_r_lin , mu_r_data , 'DisplayName' , 'Rear data' , 'LineWidth', 2 , 'Color', 'blue' , 'LineStyle','-')
    hold on
    plot(alpha_r_lin , mu_r_theor , 'DisplayName' , 'Rear theory' , 'LineWidth', 2 , 'Color', 'blue' , 'LineStyle','-.')
    hold on
    plot(alpha_r_lin , mu_f_data , 'DisplayName' , 'Front data' , 'LineWidth', 2 , 'Color', 'red' , 'LineStyle','-')
    hold on
    plot(alpha_r_lin , mu_f_theor , 'DisplayName' , 'Front theory' , 'LineWidth', 2 , 'Color', 'red' , 'LineStyle','--')
    hold on
    xlabel('$\alpha_{r},\alpha_{f}$')
    ylabel('$\mu_{f},\mu_{r}$')
    legend(Location = 'best')
    title('Normalised Axle Characteristics')
    
    % -------------------------------
    %% Plot Handling dialgram
    % -------------------------------

    % rho*L - delta = dAlpha --> delta --> vehicle steering angle [NOT DRIVER STEER ANGLE] 
    figure('Name','EXTRA Handling Diagram')
    
    plot(Ay_ss/g , -oper_cond , 'LineWidth', 2 , 'DisplayName', 'Operating condition');
    hold on
    plot(Ay_ss/g , -dalpha , 'LineWidth', 2, 'LineStyle','--' , 'DisplayName','$ - \Delta \alpha$')
    hold on
    ylabel('$\delta$ - $\rho$L')
    xlabel('$\frac{A_{y}}{g}$')
    xlim([0.01,0.6])
    legend(Location = 'best');
    title('Handling Diagram')

    % -------------------------------
    %% Plot dAlpha - Time
    % -------------------------------
    figure('Name', 'dAlpha - Time')
 
    plot(alpha_r , 'DisplayName' , '$\alpha_{r}$' , 'LineWidth', 2 )
    hold on
    plot(alpha_f , 'DisplayName' , '$\alpha_{f}$', 'LineWidth', 2 )
    hold on
    plot(alpha_r-alpha_f , 'DisplayName' , '$\Delta\alpha$' , 'LineWidth', 2 )
    ylabel('$\alpha$ [rad]' )
    xlabel('t [s]')
    legend
    % -------------------------------
    %% Plot Critical speed
    % -------------------------------
    figure('Name' , 'Critical speed')
    plot(time_sim_lin , sqrt(Ay_ss(idx.time)./rho_ss(idx.time)) , 'DisplayName' , 'Critical speed' , 'Linewidth' , 2 , 'LineStyle' , '--')
    hold on
    plot(time_sim_lin , u(idx.time) , 'DisplayName' , 'speed' , 'LineWidth', 2 )
    hold on 
    ylabel('u [m/s]' )
    xlabel('t [s]')
    legend
    % -------------------------------
    %% Plot Yaw rate Gain
    % -------------------------------
    
    figure('Name' , 'Yaw rate Gain')
    plot(u , yaw_rate_gain_def1 , 'LineWidth', 2 , 'DisplayName','$\frac{\Omega}{\delta}$')
    hold on
    plot(u , yaw_rate_gain_def2 ,'LineStyle', '--' , 'LineWidth', 2 , 'DisplayName','$\frac{u}{L (1 + K_{us} u^2)}$')
    plot(u , Omega_NS  , 'Color','green' , 'LineWidth',2 , 'DisplayName', 'NS')
    xlabel('u [m/s]')
    ylabel('Yaw Gain')
    legend(Location='best')
    title('Yaw Rate Gain')


    % -------------------------------
    %% Plot Acceleration gain
    % -------------------------------

    figure('Name','Acceleration gain')
    plot(u , Acc_gain , 'LineWidth',2 , 'DisplayName','Data')
    hold on
    plot(u , Acc_gain_NS , 'LineWidth',2 , 'DisplayName','NS' , 'Color','green')
    hold on
    xlabel('u [m/s]')
    ylabel('$\frac{A_y}{\delta}$')
    legend

    % -------------------------------
    %% Plot Body slip gain
    % -------------------------------
    
    figure('Name' , 'Body slip gain')
    plot(u , beta_gain , 'LineWidth',2 , 'DisplayName','Data')
    hold on
    plot(u , beta_gain_neurtal , 'LineWidth',2 , 'DisplayName','NS')
    xlabel('u')
    ylabel('$\frac{\beta}{\delta}$')
    grid on
    legend(Location = 'northwest')
    title('Body Slip Gain')

    







end