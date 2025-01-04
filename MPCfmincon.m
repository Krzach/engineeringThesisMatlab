function nextU = MPCfmincon(X,prevU,goalY,T)

    options_fmincon = optimoptions(@fmincon,'Algorithm','sqp','Display','none');


    Nu = 3;
    nu = 2;



    umax = ones(nu*Nu,1).*6*pi; 
    umin = ones(nu*Nu,1).*-6*pi;

    N1=eye(nu*Nu);

    J = eye(nu*Nu);
    for i=1:Nu-1
        for j =  1:i
            J(2+2*i-1:2+2*i,2*j-1:2*j)=eye(2);
        end
    end

    A_opt = [-J*N1;
          J*N1];

    b_opt = [-umin + [prevU;prevU;prevU];
            umax - [prevU;prevU;prevU]];

    [du_opt,fval,exitflag] = fmincon(@(du) mpcno_car(du,prevU,X,goalY,T),zeros(1,Nu*nu),A_opt,b_opt,[],[],[],[],[],options_fmincon);

    nextU = du_opt(1:2)'+prevU;