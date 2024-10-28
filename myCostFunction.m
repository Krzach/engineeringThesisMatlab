function J = myCostFunction(x, u, e, data)
    % Target state [alpha, x, y]
    x_target = [0, 1, 1];  % Desired final state

    % State cost (quadratic cost over prediction horizon)
    for k = 1:size(x, 1)
        x_temp(k,:) = x(k,2:end)-x_target(2:end);
    end 
    state_cost = sum(sum((x_temp).^2));

    % Control effort cost (penalize large control inputs)
    %u_prev = data.MV0;

    control_effort = 0.1 * sum(sum((u).^2));

    % Total cost
    J = state_cost + control_effort;
end
