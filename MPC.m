% Parameters
a = 0.1; % constant for angular velocity
b = 0.5; % constant for linear velocity

% Define prediction and control horizons
PredictionHorizon = 30;
ControlHorizon = 4;

% Number of states, inputs, and outputs6
nlobj = nlmpc(3, 3, 2);

% Specify sample time
Ts = 0.05; % Sample time

% Set state function
nlobj.Model.StateFcn = @(x, u) vehicleModelDiscrete(x, u, Ts);

% Set state and output functions
nlobj.Model.IsContinuousTime = false; % Discrete-time system
nlobj.Ts = Ts;

% Set constraints on inputs (optional)
nlobj.MV(1).Min = -3*pi; % Min value for u1
nlobj.MV(1).Max = 3*pi;  % Max value for u1
nlobj.MV(2).Min = -3*pi; % Min value for u2
nlobj.MV(2).Max = 3*pi;  % Max value for u2

% Set the prediction and control horizons
nlobj.PredictionHorizon = PredictionHorizon;
nlobj.ControlHorizon = ControlHorizon;

% Set cost function (optional, could also use default)
%nlobj.Optimization.CustomCostFcn = @(X, U, e, data) myCostFunction(X, U, e, data);
%nlobj.Optimization.ReplaceStandardCost = true;

% Set weightings for state deviations and control effort
nlobj.Weights.OutputVariables = [0 1 2];  % Penalty on (alpha, x, y)
nlobj.Weights.ManipulatedVariablesRate = [0.01 0.01]; % Penalize large input changes


% Initial conditions
x0 = [pi/4; 0; 0];  % Initial state: [alpha, x, y]

% Simulation horizon
T = 500;  % Number of time steps

time = 0:Ts:T;

% Preallocate
xHistory = zeros(T, 3);
uHistory = zeros(T, 2);
xHistory(1, :) = x0;

xHistory2 = zeros(T, 3);

% Initial control input
u0 = [0; 0]; % initial input guess

robot = differentialDriveKinematics("WheelRadius",0.02,"TrackWidth",0.18);

errorHistory = zeros(T);

for k = 1:T
    % Define references (e.g., moving to a target location)
 

    yPojazdu = cel_dla_pojazdu(xHistory(k,:));

    errorHistory(k)=yPojazdu;

    goal = [0, 0.24, yPojazdu];  % Target state
    
    % Compute the optimal control action
    [u, info] = nlmpcmove(nlobj, [0;0;0], u0, goal);
    
    % Apply the control input to the system
    %x_next = vehicleModelDiscrete(xHistory(k, :)', u, Ts);

    %xHistory2(k+1,:) = model_zdyskretyzowany(xHistory2(k,:), Ts, u(1), u(2),0.02,0.18);
    tspan = [time(k), time(k+1)];
    [t,y] = ode45(@(t,y) test_modelu(t,y,u(1),u(2),0.02,0.18),tspan, xHistory(k,:));

    %x2_next(1)=y(end,3);
    %x2_next(2)=y(end,1);
    %x2_next(3)=y(end,2);

    xHistory(k+1,:) = y(end,:);
    
    % Update the state with the final value from the ODE solver
    %x = y_cont(end, :)';  % Update to the final state
    
    % Store updated state in history
    %xHistory(k+1, :) = x_next';
    
    % Update the initial control input for the next step
    uHistory(k,:)=u;
    u0 = u;


end
figure;
plot(uHistory(:,1))
hold on
plot(uHistory(:,2))
title("u1, u2")

% Plot results
figure;
subplot(3,1,1); plot(xHistory(:,2)); title('X position');
subplot(3,1,2); plot(xHistory(:,3)); title('Y position');
subplot(3,1,3); plot(xHistory(:,1)); title('Orientation');

figure
plot(xHistory(:,2), xHistory(:,3))
hold on
plot(linspace(0,4,400), trasa(linspace(0,4,400)));
xlabel("X")
ylabel("Y")
title("Position on map")

figure
plot(errorHistory)
xlabel("t")
ylabel("Error")
title("wykryty błąd")



