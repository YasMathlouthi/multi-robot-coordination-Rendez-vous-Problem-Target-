function simulate_robots2(n, time, dt, x_h, y_h, x_f, y_f, g1, K_d, K_ij)
    % Initialize positions
    x = zeros(n, length(time));
    y = zeros(n, length(time));
    distances = zeros(n, length(time));
    has_met = false;  % Indicator if all robots met at the meeting point

    % Set initial positions
    x(:, 1) = x_h';
    y(:, 1) = y_h';

    % Initialize control variables for the curve phase
    curve_start_time = 0;  % Time when curve following starts
    curve_speed = 0.5;  % Speed along x-axis for curve following

    % Run the simulation (Euler method for simplicity)
    for t = 2:length(time)
        for i = 1:n
            if ~has_met
                % Control law for moving towards the meeting point
                U_xi = -K_d * (x(i, t-1) - x_f(i));
                U_yi = -K_d * (y(i, t-1) - y_f(i));
            else
                % Control law for moving along the curve y = 2x^3
                if t == curve_start_time
                    x(i, t) = x_f(i);  % Synchronize x positions
                    y(i, t) = y_f(i);  % Synchronize y positions
                end
                U_xi = curve_speed;
                U_yi = -K_d * (y(i, t-1) - 2 * x(i, t-1)^3);
            end
            
            % Sum of the influence of other robots (if necessary)
            for j = 1:n
                if i ~= j
                    U_xi = U_xi - g1(i, j) * K_ij * (x(i, t-1) - x(j, t-1));
                    U_yi = U_yi - g1(i, j) * K_ij * (y(i, t-1) - y(j, t-1));
                end
            end

            % Update positions (simple Euler integration)
            x(i, t) = x(i, t-1) + U_xi * dt;
            y(i, t) = y(i, t-1) + U_yi * dt;
        end
        
        % Check if all robots have met at the meeting point
        if ~has_met && all(sqrt((x(:, t) - x_f(i)).^2 + (y(:, t) - y_f(i)).^2) < 0.0001)
            has_met = true;  % All robots have met
            curve_start_time = t;  % Set the time when curve following starts
        end
    end

    % Plotting results
    figure;
    hold on;
    axis equal;
    grid on;
    for i = 1:n
        plot(x(i, :), y(i, :), 'LineWidth', 2); % Trajectory
        plot(x(i, 1), y(i, 1), 'ro'); % Initial position
        text(x(i, 1) + 0.5, y(i, 1), sprintf('Robot %d', i), 'FontSize', 12);
        plot(x_f(i), y_f(i), 'gx', 'LineWidth', 2); % Meeting point
        
    end
    title('Robot trajectories and meeting point');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Trajectories', 'Start Positions','Meeting point', 'Location', 'best');
    hold off;
end
