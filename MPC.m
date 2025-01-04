% Parameters
a = 0.1; % constant for angular velocity
b = 0.5; % constant for linear velocity

% Define prediction and control horizons
PredictionHorizon = 15;
ControlHorizon = 3;

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
nlobj.MV(1).Min = -6*pi; % Min value for u1
nlobj.MV(1).Max = 6*pi;  % Max value for u1
nlobj.MV(2).Min = -6*pi; % Min value for u2
nlobj.MV(2).Max = 6*pi;  % Max value for u2

% Set the prediction and control horizons
nlobj.PredictionHorizon = PredictionHorizon;
nlobj.ControlHorizon = ControlHorizon;


% Set weightings for state deviations and control effort
nlobj.Weights.OutputVariables = [0 1 6];  % Penalty on (alpha, x, y)
nlobj.Weights.ManipulatedVariablesRate = [0.01 0.01]; % Penalize large input changes


% Initial conditions
x0 = [pi/100; 0; 0];  % Initial state: [alpha, x, y]

% Simulation horizon
T = 500;  % Number of time steps

time = 0:Ts:T;

% Preallocate
xHistory = zeros(T, 3);
uHistory = zeros(T, 2);
xHistory(1, :) = x0;
dxHistory = zeros(T, 2);

xHistory2 = zeros(T, 3);

% Initial control input
u0 = [0; 0]; % initial input guess

robot = differentialDriveKinematics("WheelRadius",0.02,"TrackWidth",0.18);

errorHistory = zeros(T,1);

u1=zeros(T,2);

for k = 1:T
    % Define references (e.g., moving to a target location)
 

    yPojazdu = cel_dla_pojazdu(xHistory(k,:));
    dxHistory(k,:) = yPojazdu(2,:);

    errorHistory(k)=yPojazdu(1,1);

    goal = [0.24, yPojazdu(1,1)];  % Target state
    
    u = MPCfmincon([0;0;0],u0,goal,Ts);

    % Compute the optimal control action

    %u = MPCsl([0;0;0],u0,goal,Ts);

    %if k>1
    %    u = MPCsl([0;0;0],u0,goal,Ts,xHistory(k,:)',xHistory(k-1,:)');
    %else
    %    u = MPCsl([0;0;0],u0,goal,Ts,[0;0;0],[0;0;0]);
    %end
    % Apply the control input to the system

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
title("Sterowanie")
legend("U1", "U2")

% Plot results
figure;
subplot(3,1,1); plot(xHistory(:,2)); title('X position');
subplot(3,1,2); plot(xHistory(:,3)); title('Y position');
subplot(3,1,3); plot(xHistory(:,1)); title('Orientation');

syms x 
eq2 = 0.5*sin(x*pi*0.5)^5;
%eq2 = 0.2;

figure
plot(dxHistory(:,1), dxHistory(:,2))
hold on
fplot(eq2, [0,max(dxHistory(:,1))]);
hold on
legend("Środek czujnika","Trasa")
xlabel("X")
ylabel("Y")
title("Pozycja na mapie")


 %for k = 1:10:T
 %quiver(xHistory(k,2),xHistory(k,3),dxHistory(k,1)-xHistory(k,2),dxHistory(k,2)-xHistory(k,3),0, "Color","red","LineWidth",1);
 %end    


figure
plot(errorHistory)
xlabel("t")
ylabel("Error")
title("wykryty błąd, E = " + string(sum(errorHistory.^2)))



