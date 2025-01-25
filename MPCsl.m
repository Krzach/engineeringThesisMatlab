function nextU = MPCsl(X,prevU,goalY,T)
    N = 8;
    Nu = 2;
    nx = 3;
    nu = 2;
    ny = 2;

    umax = ones(nu*Nu,1).*6*pi; 
    umin = ones(nu*Nu,1).*-6*pi;
%     dumax = [pi; pi;
%              pi; pi;
%              pi; pi];
%     dumin = [-pi; -pi;
%              -pi; -pi;
%              -pi; -pi];

    C = [0 1 0;
       0 0 1];
    psi = [1 0;
           0 23];
    lambda = [0.0007 0;
              0   0.0007];

    [A,B] = linearizeModel(X,prevU,T);

    Afalka=zeros(nx*N,nx);
    V=zeros(nx*N,nx);

    for i=1:N
        Afalka((i-1)*nx+1 : i*nx, :)=A^i;
    end    
    
    for i=1:N
        if(i>1)
            sumA=zeros(nx,nx);
            for j=1:i-1
                sumA = sumA + A^j;
            end
            V((i-1)*nx+1 : i*nx, :) = eye(nx,nx) + sumA; 
        else    
            V((i-1)*nx+1 : i*nx, :) = eye(nx,nx); 
        end
    end

   % vk = vehicleModelDiscrete(X,prevU,T) - A*X - B*prevU;

    Mx = zeros(nx*N,nu*Nu);

    for k = 1:Nu
        for i = 1:N
            p1min = (i-1)*nx+1;
            p1max = i*nx;

            p2min = (k-1)*nu+1;
            p2max = k*nu;

            if(i-k>1)
                sumA=zeros(nx,nx);
                for j=1:i-k
                    sumA = sumA + A^j;
                end
                Mx(p1min:p1max, p2min:p2max) = (eye(nx,nx) + sumA)*B; 
            elseif(i-k == 0)    
                Mx(p1min:p1max, p2min:p2max) = B; 
            end
        end
    end

    Cfalka = zeros(ny*N,nx*N);

    for i = 1:N
       Cfalka((i-1)*ny+1:i*ny, (i-1)*nx+1:i*nx) = C;
    end 

    duzeLambda = zeros(nu*Nu,nu*Nu);

    for i = 1:Nu
       duzeLambda((i-1)*nu+1:i*nu, (i-1)*nu+1:i*nu) = lambda;
    end 

    duzePsi = zeros(ny*N,ny*N);

    for i = 1:N
        duzePsi((i-1)*ny+1:ny*i, (i-1)*ny+1:ny*i) = psi;
    end

    %N1=zeros(nu*Nu, nu*Nu + nu +1);
    %N1(:, 1:nu*Nu)=eye(nu*Nu);
    N1=eye(nu*Nu);

    M = Cfalka*Mx;

    Hqp = (N1'*M'*M*N1 + N1'*duzeLambda'*N1).*2;

    %teraz liczymy fqp

    Ifalka = zeros(ny*N, ny);
    for i =1:N
        Ifalka((i-1)*ny+1 : i*ny, :) = eye(ny);
    end
%    ddx = XabsPrevK-XabsK;
    %zaniechuje błąd modelu
%     vk = X - A*(X+ddx) - B*prevU;
%  
 %     dk = C*vk;
% 
%     y0 = Cfalka*(Afalka*X + V*(B*prevU)) + Ifalka*dk;
  y0 = Cfalka*(Afalka*X + V*(B*prevU)); %tutaj zmiana

    ysp = zeros(N*ny,1);
    for i =1:N
        ysp(i*ny-ny+1:i*ny) = goalY;
    end
    fqp = (N1'*M'*duzePsi*(ysp-y0)).*(-2);

    J = eye(nu*Nu);
    for i=1:Nu-1
        for j =  1:i
            J(2+2*i-1:2+2*i,2*j-1:2*j)=eye(2);
        end
    end
    

    Aqp = [-J*N1;
            J*N1];
           % -N1;
           % N1];
    bqp = [-umin + [prevU;prevU];
            umax - [prevU;prevU]];
           % -dumin;
           % dumax];


    tempU = quadprog(Hqp,fqp,Aqp,bqp);
    nextU = tempU(1:2)+prevU;

end    