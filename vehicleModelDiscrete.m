function dxdt = vehicleModelDiscrete(x, u, T)

    a=0.02/0.18*T;
    b=0.01*T;

    alpha = x(1);
    x_pos = x(2);
    y_pos = x(3);

    u1 = u(1);
    u2 = u(2);

    % Update equations
    alpha_next = a * (u1 - u2) + alpha;
    x_next = b * (u1 + u2) * cos(alpha) + x_pos;
    y_next = b * (u1 + u2) * sin(alpha) + y_pos;

    dxdt = [alpha_next; x_next; y_next];
end
