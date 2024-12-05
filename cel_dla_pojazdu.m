function to_send = cel_dla_pojazdu(X)

    alpha = X(1);
    x0=X(2);
    y0=X(3);

    dx = 0.24/sqrt(1+tan(alpha)^2);
    dy = tan(alpha)*dx;

    x1 = x0+dx;
    y1 = y0+dy;
    to_send = zeros(2,2);
    to_send(2,1) = x1;
    to_send(2,2) = y1;
    
    syms x y

    if alpha == 0
        
        eq1 = x == x1;
        %eq2 = 0.5*sin(x*pi*0.5)^5 - y == 0;
        eq2 = y == 0.2;

        solution = solve([eq1, eq2], [x, y]);
    else
        a1 = - 1/tan(alpha);
    
        b = y1 - a1*x1;
    
        eq1 = -a1*x+y == b;
        %eq2 = 0.5*sin(x*pi*0.5)^5 - y == 0;
        eq2 = y == 0.2;
    
        solution = solve([eq1, eq2], [x, y]);
    end

    c2 = (solution.x-x0)^2+(solution.y-y0)^2; 

    y_for_car = double(sqrt(c2-0.24^2));

    if ((solution.y-y0)/(solution.x-x0)) < tan(alpha)
        y_for_car = -y_for_car;
    end
    to_send(1,1) = y_for_car;

end