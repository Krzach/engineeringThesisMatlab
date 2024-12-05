 kinematicModel = differentialDriveKinematics;
 initialState = [0 0 0];
 
 tspan = 0:0.01:2;
 inputs = [30 40]; %Left wheel is spinning faster
 [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,initialState);
 
 %figure
 %plot(y(:,1),y(:,2))
 %xlabel('x')
 %ylabel('y')
 %hold on
 
 [t,y] = ode45(@(t,y) test_modelu(t,y,40,30,0.05,0.2),tspan,initialState);
 plot(y(:,2),y(:,3))
 hold on
 legenda = ["Model ciągły"];
 
 Y=zeros(3,length(tspan));

 for k=1:5

   if(k ==1)
       for i=1:length(tspan)-1
            Y(i+1,:) = model_zdyskretyzowany(Y(i,:),0.01,40,30,0.05,0.2);
       end
       legenda(k+1) = "Model dyskretny, t = "+string(0.1);
   else 
      
     for i=1:length(tspan)*0.01/(0.05*(k-1))-1
         Y(i+1,:) = model_zdyskretyzowany(Y(i,:),0.05*(k-1),40,30,0.05,0.2);
     end
     
     legenda(k+1) = "Model dyskretny, t = "+string(0.05*(k-1));
     
      

   end
   plot(Y(:,1),Y(:,2))
   Y=zeros(3,length(tspan));
 end
 legend(legenda);