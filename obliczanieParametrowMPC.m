    
  syms a31 goalY1 goalY2 u1 u2 real


  T = 0.05;
  a=0.02/0.18*T;
  b=0.01*T;

    A = [1 0 0;
        0 1 0;
        a31 0 1];

    B = [a -a;
        b  b;
        0 0];  

    goalY = [goalY1; goalY2];

    X=[0; 0; 0];
    prevU = [u1; u2];

    N = 10;
    Nu = 3;
    nx = 3;
    nu = 2;
    ny = 2;

    umax = ones(nu*Nu,1).*6*pi; 
    umin = ones(nu*Nu,1).*-6*pi;

    C = [0 1 0;
       0 0 1];

    psi = [1 0;
           0 20];
    lambda = [0.0005 0;
              0   0.0005];

%     [A,B] = linearizeModel(X,prevU,T);

    Afalka=sym(zeros(nx*N,nx));
    V=sym(zeros(nx*N,nx));

    for i=1:N
        Afalka((i-1)*nx+1 : i*nx, :)=A^i;
    end    
    
    for i=1:N
        if(i>1)
            sumA=sym(zeros(nx,nx));
            for j=1:i-1
                sumA = sumA + A^j;
            end
            V((i-1)*nx+1 : i*nx, :) = eye(nx,nx) + sumA; 
        else    
            V((i-1)*nx+1 : i*nx, :) = eye(nx,nx); 
        end
    end

    Mx = sym(zeros(nx*N,nu*Nu));

    for k = 1:Nu
        for i = 1:N
            p1min = (i-1)*nx+1;
            p1max = i*nx;

            p2min = (k-1)*nu+1;
            p2max = k*nu;

            if(i-k>1)
                sumA=sym(zeros(nx,nx));
                for j=1:i-k
                    sumA = sumA + A^j;
                end
                Mx(p1min:p1max, p2min:p2max) = (eye(nx,nx) + sumA)*B; 
            elseif(i-k == 0)    
                Mx(p1min:p1max, p2min:p2max) = B; 
            end
        end
    end

    Cfalka = sym(zeros(ny*N,nx*N));

    for i = 1:N
       Cfalka((i-1)*ny+1:i*ny, (i-1)*nx+1:i*nx) = C;
    end 

    duzeLambda = sym(zeros(nu*Nu,nu*Nu));

    for i = 1:Nu
       duzeLambda((i-1)*nu+1:i*nu, (i-1)*nu+1:i*nu) = lambda;
    end 

    duzePsi = sym(zeros(ny*N,ny*N));

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
    
    %zaniechuje błąd modelu
%      vk = vehicleModelDiscrete(X,prevU,T) - A*X - B*prevU;
%  
%      dk = C*vk;
% 
%      y0 = Cfalka*(Afalka*X + V*(B*prevU+vk)) + Ifalka*dk;
   y0 = Cfalka*(Afalka*X + V*(B*prevU));

    ysp = sym(zeros(N*ny,1));
    for i =1:N
        ysp(i*ny-ny+1:i*ny) = goalY;
    end
    fqp = (N1'*M'*duzePsi*(ysp-y0)).*(-2);

    Hqpexpanded = expand(Hqp);

    Hqpcollected = collect(Hqpexpanded, [a31 goalY1 goalY2 u1 u2]);

    fqpexpanded = expand(fqp);