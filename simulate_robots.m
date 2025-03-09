function simulate_robots(n, time, dt, x_h, y_h, x_f, y_f, g1, K_d, K_ij)
    % Initialize positions
    x = zeros(n, length(time));
    y = zeros(n, length(time));
    distances = zeros(n, length(time));

    % Set initial positions
    x(:, 1) = x_h';
    y(:, 1) = y_h';
    A = zeros(2*n, 2*n);
    for i = 1:n
        for j = 1:n
            if i ~= j
                A(2*i-1, 2*j-1) = g1(i, j) * K_ij;  % X interactions
                A(2*i, 2*j) = g1(i, j) * K_ij;      % Y interactions
            end
        end
        A(2*i-1, 2*i-1) = -K_d-5*K_ij;  % Self-damping X
        A(2*i, 2*i) = -K_d-5*K_ij;      % Self-damping Y
    end
    disp(A)
    % Run the simulation (Euler method for simplicity)
    for t = 2:length(time)
        for i = 1:n
            % Control law
            U_xi = -K_d * (x(i, t-1) - x_f(i)); % traction to the target  
            U_yi = -K_d * (y(i, t-1) - y_f(i));
            
            % the influence of other robots
            for j = 1:n
                if i ~= j
                    U_xi = U_xi - g1(i, j) * K_ij * ((x(i, t-1) - x(j, t-1)) - (x_f(i) - x_f(j)));
                    U_yi = U_yi  - g1(i, j) * K_ij * ((y(i, t-1) - y(j, t-1)) - (y_f(i) - y_f(j)));
                    
                        
                end
            end
            disp(U_xi)
            disp(U_yi)
            
            % Update positions (simple Euler integration)
            x(i, t) = x(i, t-1) + U_xi * dt;
            y(i, t) = y(i, t-1) + U_yi * dt;
            
            % Calculate distance from the target
            distances(i, t) = sqrt((x(i, t)^2 + y(i, t)^2) - ((x_f(i)^2 + y_f(i)^2) ));
        end
    end
disp(A)
eig(A)
    % Plot the trajectories of the robots
    figure;
    hold on;
    axis([-50 50 -50 50]);
    for i = 1:n
        plot(x(i, 1), y(i, 1), 'ro'); % Initial position
        text(x(i, 1) + 0.5, y(i, 1), sprintf('Robot %d', i), 'FontSize', 12);
        plot(x_f(i), y_f(i), 'gx', 'LineWidth', 2); 
        text(x_f(i) - 4, y_f(i), sprintf('Target'), 'FontSize', 12);
        plot(x(i, :), y(i, :), 'LineWidth', 2); % Trajectory
    end
    
    xlabel('X Position');
    ylabel('Y Position');
    title(sprintf('Trajectories of Robots for K_d= %d and K_ij= %d', K_d, K_ij));
    grid on;
    hold off;

    % Plot the distances from the origin vs time
    figure;
    hold on;
    for i = 1:n
        plot(time, distances(i, :), 'LineWidth', 1);
    end
    xlabel('Time (s)');
    ylabel('Distance to Target');
    title('Distances from Robots to the target vs. Time');
    grid on;
    hold off;
end