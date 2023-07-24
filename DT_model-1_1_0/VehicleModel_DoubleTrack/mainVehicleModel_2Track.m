% ----------------------------------------------------------------
%% Main script for a basic simulation framework with a double track vehcile model
%  authors: 
%  rev. 1.0 Mattia Piccinini & Gastone Pietro Papini Rosati
%  rev. 2.0 Edoardo Pagot
%  date:
%  rev 1.0:    13/10/2020
%  rev 2.0:    16/05/2022
%  rev 2.1:    08/07/2022 (Biral)
%       - added Fz saturation. Correceted error in Fx
%       - initial condition is now parametric in initial speed
%       - changed the braking torque parameters to adapt to a GP2 model
% ----------------------------------------------------------------

% ----------------------------
%% Initialization
% ----------------------------
initialize_environment;

% ----------------------------
%% TEST SELECTION
% ----------------------------
flag.test = 1;      % 1 --> STEER ramp test
                    % 0 --> SPEED ramp test
% ----------------------------
%% Load vehicle data
% ----------------------------

% test_tyre_model; % some plot to visualize the curvers resulting from the
% loaded data

vehicle_data = getVehicleDataStruct();
% pacejkaParam = loadPacejkaParam();

% ----------------------------
%% Define initial conditions for the simulation
% ----------------------------
V0 = 0.01/3.6; % Initial speed
X0 = loadInitialConditions(V0);

% ----------------------------
%% Define the desired speed
% ----------------------------
V_des = 50/3.6; % Desired speed

% ----------------------------
%% Simulation parameters
% ----------------------------
simulationPars = getSimulationParams(); 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

% ----------------------------
%% Start Simulation
% ----------------------------
fprintf('Starting Simulation\n')
tic;
model_sim = sim('Vehicle_Model_2Track');
elapsed_time_simulation = toc;
fprintf('Simulation completed\n')
fprintf('The total simulation time was %.2f seconds\n',elapsed_time_simulation)

%% camber
camber_ang = 6;

% For every camber angle run a simulation. 
for i = -camber_ang:camber_ang
    
    vehicle_data.rear_wheel.static_camber = i;
    vehicle_data.front_wheel.static_camber = i;
    if i<0
        % v_min = version negative velocity
        my_field = strcat('v_min',num2str(abs(i)));
    else
        % v = version
        my_field = strcat('v',num2str(i));
    end
    fprintf('camber angle = %f\n' , i);
    model_sim_camber.(my_field) = sim('Vehicle_Model_2Track');
    
end

fprintf('Camber simulation Completed \n')

%% Axle Stiffness

eps_Ks_init = [9 4 2.33333333333 1.5000000 1 0.666666 0.428571428571429 0.25 0.11111];
vehicle_data = getVehicleDataStruct();
Ks_r = vehicle_data.rear_suspension.Ks_r;
Ks_f = vehicle_data.front_suspension.Ks_f;

for i=1:length(eps_Ks_init)
    Ks_f = eps_Ks_init(i)*Ks_r;
    vehicle_data.front_suspension.Ks_f = Ks_f;
    eps_Ks = Ks_f/(Ks_f+Ks_r);
    fprintf('epsilon_{Ks} = %f\n' , eps_Ks);
    my_field = strcat('eps_Ks_',num2str(i));
    model_sim_Ks.(my_field) = sim('Vehicle_Model_2Track');
    
end

fprintf('Axle stiffness simulation Completed \n')

%% Toe angle

toe_ang = 6;
for i = -toe_ang:toe_ang
    
    vehicle_data.front_wheel.delta_f0 = i;
    if i<0
        my_field = strcat('v_min',num2str(abs(i)));
    else
        my_field = strcat('v',num2str(i));
    end
    fprintf('Toe angle = %f\n' , i);
    model_sim_toe.(my_field) = sim('Vehicle_Model_2Track');
    
end

fprintf('Toe angle simulation Completed \n')

% ----------------------------
%% Post-Processing
% ----------------------------
vehicle_data = getVehicleDataStruct();
% clc
% close all
dataAnalysis(model_sim,vehicle_data,Ts);

%% Extra Data Analysis
extra_data_analysis(model_sim,vehicle_data,Ts);

%% Camber Data Analysis

camber_data_analysis(model_sim_camber , vehicle_data , Ts , camber_ang);

%% Axle Stiffness Data Analysis
Axle_data_analysis(model_sim_Ks,vehicle_data,Ts,eps_Ks_init);

%% Toe angle Data Analysis

toe_data_analysis(model_sim_toe,vehicle_data,Ts , toe_ang)

%% Save Figures
save_fig(flag.test);






