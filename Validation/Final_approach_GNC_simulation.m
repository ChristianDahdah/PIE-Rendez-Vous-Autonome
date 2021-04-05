% POSSIBLE FURTHER IMPROVEMENTS TO THIS TOOL:
%   * add realistic environmental disturbances to the nonlinear model (atmospheric drag, etc)
%   * add uncertainties and model errors to the nonlinear model (error on the position of the center of mass,..) to run robustness campains (monte carlo)
%   * define metrics to quantify the quality of the trajectory provided by the MPC law (energy consumed, time, distance to approach corridor, etc) and of the state estimation
%   * add an output matrix C to simulate the fact that we don't measure directly the entire state X but a limited set of information based on our set of sensors (gyroscope,..)

function [Xproper,U,Xest] = Final_approach_GNC_simulation(nonlinear,initial,precision,maxtime,dt,Radius,q,r,DT,horizon,estimation,W,V,dT1,dT2,SNR)
% INPUTS:
% (the first 6: general simulation parameters)
% - nonlinear: boolean, true to use the realistic nonlinear equations to simulate the real dynamics and false to use the linearised equations instead (for instance for debugging or unitary validation purposes)
% - initial: initial state, format x0 = [alpha;beta;gamma;wx;wy;wz;sx;sy;sz;dsx;dsy;dsz] (vertical vector) with (alpha, beta, gamma) the initial euler angles (rad), w the initial relative angular velocity of the chaser in target frame (rad/s), s the initial relative distance of the chaser in target frame (m), and ds the initial relative speed of the chaser in target frame (m/s)
% - precision: precision threshold for the norm of the state, upon which we consider to have reached the objective
% - maxtime: maximum simulated time (in seconds) upon which the simulation will stop if it hasn't already stopped so far, to avoid infinitely long running time in case of non-convergence
% - dt: time step (in seconds) to be used for the numerical integration of the differential equations that simulate the real dynamics (the smaller the more realistic the simulation but the more computationaly heavy it gets)
% - Radius: radius of the approach cylinder (in meters)
% (the next 4: MPC controler parameters)
% - q and r: respectively 12x12 and 6x6 matrices, classical tuning parameters for the optimisation cost (similar to a LQ controler)
% - DT: MPC controler time step in seconds (every DT seconds, the MPC controler is called to compute the next comands to be held for the next DT seconds until the next computation). Has to be greater than dt
% - horizon: prediction horizon of the MPC controler, in number of time steps DT (not in seconds). Refer to the report for details on this parameter
% (the last 6: noise and estimation parameters)
% - estimation: boolean, true if the estimation of the state with noise should be taken into account in the simulation, false if the entire simulation step should be bypassed and the controler has access to perfect state feedback (for instance for debugging or unitary controler validation purposes). If set to false, the next 5 inputs will not have any influence
% - w and v: 12x12 matrices, respectively state noise and measurement noise, classical tuning parameters for the Kalman estimator (the bigger w is relative to v, the less we trust the model compared to the measurements when making estimates of the state ; and vice versa). If previous parameter 'estimation' is set to False, the values of these parameters don't have any influence
% - dT1: time step (in seconds) for the prediction step of the estimator (frequency at which the state estimate is propagated with the linearised model), basically corresponds to the onboard calculator time step. If previous parameter 'estimation' is set to False, the value of this parameter doesn't have any influence
% - dT2: time step (in seconds) for the upsate step of the estimateur (frequency at which the state estimate is updated with the new measurements), basically corresponds to the measurement sampling time. If previous parameter 'estimation' is set to False, the value of this parameter doesn't have any influence
% - SNR: signal to noise ratio for simulating measurement noises (the smaller the SNR, the noisier the measurements). If previous parameter 'estimation' is set to False, the value of this parameter doesn't have any influence
%
% OUTPUTS
% - X: history of the real states x throughout the simulations every dt step, format X = [x(0) x(dt) x(2*dt) x(3*dt)..]
% - U: history of the commands u applied to the chaser throughout the simulation every dt step, format U = [u(0) u(dt) u(2*dt) u(3*dt)..]
% - Xest: history of the estimated states x throughout the simulation every dt steps, same format as X if 'estimation' input parameter is set to True, empty otherwise

close all, clc

addpath('../Codes-MPC');
addpath('../models');
addpath('../subfunctions');
model = load('../initialization/linear_model.mat');
param = load('../initialization/parameters.mat');


initial = [param.eulerDCDT_i; param.omegaDCDT_i; param.sDT_i; param.dsDT_i];
%% Overall simulation setup
quat = euler2quat(initial(1:3));
x = [quat; initial(4:12)];

s = size(x); order = s(1);
X = [x]; % History of states
U = []; % History of control steps
t = 0; % reference simulation current time in seconds

simiter = 0; % simulation steps counter

global radius % radius of the approach cylinder
radius = Radius;

%% Linear model to be used by the GNC scheme being simulated (MPC and Kalman)
% (not the more realistic nonlinear model that simulate the environment)

A = model.A;
B = model.B;
n = length(A); % Order of linear model
sb = size(B);
nb_controls = sb(2);

%% MPC setup

global T N Q R
% declared as global variables here and in fcn_MPC.m to be shared between
% the 2 codes

T = DT; % MPC control computation sampling time [s]
N = horizon; % prediction horizon

% Q and R are the tuning parameter of the control law
Q = q; % weighing matrice (precision and duration)
R = r; % weighing matrice (controls and energy)

mpciter = 0; % mpc computation steps counter

%% Kalman Filter setup

euler = quat2euler(x(1:4)); % Extraction of euler-angles information from initial state x (expressed in quaternions form)
Xest = [euler';x(5:13)]; % The estimator starts at the correct initial state, but in euler form
XXest = [Xest];

P = 0.1*eye(n); % Initial estimate covariance

% State and input matrices Ad and Bd for the state equation of the
% discretized linear model with sample time dt
L = 6*eye(n) + dt*A*(3*eye(n)+(dt/2)*A*(2*eye(n)+(dt/2)*A));
Ad = eye(n)+(dt/6)*A*L; Bd = (dt/6)*L*B;

K = 0; % Estimation gain 
% If we use an asymtptotic Kalman filter (offline preliminary computation
% of constant filter gain):
% K=lqr(A',C',W,V);
% K=K';

kalmaniter = 0;

%% Main simulation loop - updates the simulation current time t by dt at every loop run
% Note : the simulation internaly relies on quaternions for the relative
% attitude, which are converted to euler angle only when calling the MPC
% function

main_loop = tic;
while(norm(x,2)>precision && t<=maxtime) % 2 stop conditions: either the precision threshold has been reached, or the simulation has reached its maximum duration
    
    if(estimation && (t-mpciter*dT2)>=0)% measurements are available (dT2-periodic), so we can update state estimation with this information
        
        % We pass the real state x to euler form in order to simulate real measurements from it
        euler = quat2euler(x(1:4)); % Extraction of euler-angles information from simulation state x (expressed in quaternions form)
        x_proper = [euler';x(5:13)];
        
        measurements = awgn(x_proper,SNR,'measured'); % Generates the simulated measurements to be used by the GNC, by adding noise to the nonlinearly simulated state X
        
        innovation = measurements - Xest;

        K = P*inv(P + V); % updating estimator gain
        Xest = Xest + K*innovation; % taking the innovation into account to update the state estimation
        P = (eye(n)-K)*P; % updating the state estimation covariance based on this update
        
        kalmaniter = kalmaniter + 1;
    end
    
    if((t-mpciter*T)>=0) % Determine next control step with MPC (T-periodic)
        
        u = fcn_MPC(Xest); % Next input to be used by the simulation until the next call to MPC (period T)
        u = u';
        
        mpciter
        mpciter = mpciter + 1;
    end

    U = [U u];
    
    % Environment propagation with RK4
    
    if(nonlinear)
        k1 = P2P_DynCin(x,u);
        k2 = P2P_DynCin(x+k1*dt/2,u);
        k3 = P2P_DynCin(x+k2*dt/2,u);
        k4 = P2P_DynCin(x+k3*dt,u);
        
        x = x + (dt/6)*(k1+2*k2+2*k3+k4);
    else % we convert to euler form and use the linear model
        euler = quat2euler(x(1:4));
        xproper = [euler';x(5:13)];
        
        k1 = A*xproper + B*u;
        k2 = A*(xproper+k1*dt/2) + B*u;
        k3 = A*(xproper+k2*dt/2) + B*u;
        k4 = A*(xproper+k3*dt) + B*u;
        
        xproper = xproper + (dt/6)*(k1+2*k2+2*k3+k4);
        
        quat = euler2quat(xproper(1:3));
        x = [quat;xproper(4:12)];
    end

    t = t + dt;
    X = [X x];
    
    if(estimation)
        if((t-mpciter*dT1)>=0)
            % "A priori" state prediction with Kalman Filter (this purely model-based
            % prediction, updated at each dT1 step, is to be refined every time
            % measurement informations are available).
            % Just like the MPC controler, this estimator relies on the linear model to
            % make its model based prediction (the whole GNC uses this model).
            % RK4 method is used to optain the discrete state update used in this
            % prediction step, out of this continuous linear model.
            % It may not look like RK4, but in this particular case when the
            % continuous model to discretise is on the linear form AX+Bu, we can
            % derive a general analytic formula for the equivalent discrete state
            % and input matrices. Since they are always the same, they are
            % pre-calculated outside the loop.

            Xest = Ad*Xest + Bd*u; % "a-priori" model-based prediction relying on RK4
            P = Ad*P*Ad' + W;
        end
    else % If we don't take estimation into account, the estimate Xest is just the real state x put into euler form
        euler = quat2euler(x(1:4));
        Xest = [euler';x(5:13)];
    end
    
    XXest = [XXest Xest];
    
    % Note : contrary to the real state x, the state estimate Xest is always in the euler form, not
    % quaternion form, since it relies on the matrices of the linearised
    % version of the dynamics (the same used by the MPC), which is in this
    % form. This is because the purpose of Xest is to be used by MPC, so it is easier
    % to have it always in the same form than the MPC. Additionally, it simplifies 
    % the complexity. Indeed, the only needed conversion is in the update step: when we simulate measurements from
    % the real state x to update Xest, we have to first turn x into an euler form (same as Xest) before adding the noise.

    simiter = simiter + 1;
end
main_loop_time = toc(main_loop)

simiter = round(simiter)+1;

U = [U zeros(nb_controls,1)];

%% Results and plots
'Simulation iterations:'
simiter
'Simulated time:'
t
'Control steps taken:'
mpciter

Xproper = []; % Converting history of simulated states X from quaternions form to euler form
for i=1:simiter
    x = X(:,i);
    euler = quat2euler(x(1:4)); 
    Xproper = [Xproper [euler';x(5:13)]];
end

time = [1:simiter];
time = time*dt;

% State
figure
subplot(211)
plot(time,Xproper(7,:),time,Xproper(8,:),time,Xproper(9,:)), grid on
title('Evolution of relative position','FontSize',18)
xlabel('Time [s]','FontSize',15)
ylabel('x,y,z','FontSize',15)
legend('x','y','z')

subplot(212)
plot(time,Xproper(1,:),time,Xproper(2,:),time,Xproper(3,:)), grid on
title('Evolution of relative attitude','FontSize',18)
xlabel('Time [s]','FontSize',15)
ylabel('alpha,beta,gamma','FontSize',15)
legend('alpha','beta','gamma')


% Estimation
figure
subplot(211)
plot(time,Xproper(7,:),':b',time,XXest(7,:),'-b',time,Xproper(8,:),':r',time,XXest(8,:),'-r',time,Xproper(9,:),':g',time,XXest(9,:),'-g'), grid on
title('Evolution of estimated relative position','FontSize',18)
xlabel('Time [s]','FontSize',15)
ylabel('x,y,z','FontSize',15)
legend('x','x est','y','y est','z','z est')

subplot(212)
plot(time,Xproper(1,:),':b',time,XXest(1,:),'-b',time,Xproper(2,:),':r',time,XXest(2,:),'-r',time,Xproper(3,:),':g',time,XXest(3,:),'-g'), grid on
title('Evolution of estimated relative attitude','FontSize',18)
xlabel('Time [s]','FontSize',15)
ylabel('alpha,beta,gamma','FontSize',15)
legend('alpha','alpha est','beta','beta est','gamma','gamma est')


% Controls
figure
subplot(211)
plot(time,U(4,:),time,U(5,:),time,U(6,:)), grid on
title('Control signals : Forces','FontSize',18)
xlabel('Time [s]','FontSize',15)
ylabel('Control signals','FontSize',15)
legend('Ux','Uy','Uz')

subplot(212)
plot(time,U(1,:),time,U(2,:),time,U(3,:)), grid on
title('Control signals : Torques','FontSize',18)
xlabel('Time [s]','FontSize',15)
ylabel('Control signals','FontSize',15)
legend('Tx','Ty','Tz')
end