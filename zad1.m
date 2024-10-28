kinematicModel = differentialDriveKinematics;
initialState = [0 0 0];

tspan = 0:0.05:2;
inputs = [30 40]; %Left wheel is spinning faster
[t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,initialState);

figure
plot(y(:,1),y(:,2))
xlabel('x')
ylabel('y')
hold on

[t,y] = ode45(@(t,y) test_modelu(t,y,40,30,0.05,0.2),tspan,initialState);
plot(y(:,1),y(:,2))

Y=zeros(3,length(tspan));

for i=1:length(tspan)*5-1
    Y(i+1,:) = model_zdyskretyzowany(Y(i,:),0.01,40,30,0.05,0.2);
end
plot(Y(:,1),Y(:,2))