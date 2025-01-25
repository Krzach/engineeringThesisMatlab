function J = mpcno_car(deltau, prevU, X0, goal, T)

    N = 10;
    Nu = 3;
    
    nx = 3;
    nu = 2;
    ny = 2;

   psi = [1; 20]; 

   lambda = 0.001;

    J = eye(nu*Nu);
    for i=1:Nu-1
        for j =  1:i
            J(2+2*i-1 : 2+2*i, 2*j-1 : 2*j)=eye(2);
        end
    end
    J2 = zeros(Nu*nu,nu);
    for i = 1:Nu
        J2(2*i-1:2*i,:) = eye(nu);
    end

    u=zeros(N*nu,1);
    
    u(1:Nu*nu) = J*deltau' + J2*prevU;


    for i = Nu:N
      u(2*i-1:2*i) = u(Nu*nu-1:Nu*nu);
    end    

% Model
    xHistory = zeros(N,nx);


    for i =1:N
        if i == 1
            xHistory(i,:) = vehicleModelDiscrete(X0, u(2*i-1:2*i), T);
        else
            xHistory(i,:) = vehicleModelDiscrete(xHistory(i-1,:), u(2*i-1:2*i), T);
        end    
    end
    

    tempPsi = zeros(N*nu,1);
    for i =1:N
        tempPsi(2*i-1:2*i) = psi;
    end
    
    PSI = diag(tempPsi);
    


J_du=lambda*norm(deltau(1:Nu*nu))^2;


tempDiff = ones(N,1)*goal-xHistory(:,2:3);

newDiff = zeros(N*ny,1);

for i =1:N
    newDiff(2*i-1:2*i) = tempDiff(i,:)';
end

J = newDiff'*PSI*newDiff + J_du;

