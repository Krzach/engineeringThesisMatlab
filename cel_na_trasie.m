function Y = cel_na_trasie(X)

    alpha = X(1);
    x0=X(2);
    y0=X(3);

    dx = 0.14/sqrt(1+tan(alpha)^2);
    dy = tan(alpha)*dx;

    x1 = x0+dx;
    y1 = y0+dy;
    
    syms x y

    if alpha == 0
        
        eq1 = x == x1;
        eq2 = 0.5*sin(x*pi*0.5) - y == 0;

        solution = solve([eq1, eq2], [x, y]);
    else
        a1 = - 1/tan(alpha);
    
        b = y1 - a1*x1;
    
        eq1 = -a1*x+y == b;
        eq2 = 0.5*sin(x*pi*0.5) - y == 0;
    
        solution = solve([eq1, eq2], [x, y]);
    end
    Y = [solution.x, solution.y];

end