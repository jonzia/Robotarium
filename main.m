%% Herding Behavior using Networked Control
% Jacob Kimball, Jonathan Zia
% 11/2019

load('net.mat')

% Define number of sheep and dogs
sheep = 5; dogs = 1; N = sheep + dogs;

% Define Robotarium object
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Set path to save video
vidPath = 'robotarium_simulation.avi';

%% Define experimental parameters

% Define waypoints for the dogs
waypoints_dogs = [1.2; 0];
% Define waypoints for the sheep
waypoints_sheep = [-0.133 0.535;0.743 -0.207;0.234 0.329;0.219 -0.373;-0.287 -0.024]';

% Combine sheep and dog waypoints
waypoints = [waypoints_dogs waypoints_sheep];

% Set error tolerance for waypoint convergence
close_enough = 0.05;

% Set the maximum number of iterations
iterations = 1000;

% set skip parameter to help it run faster
skip = 10; % needs to be > 1 for this to work

% Specify delta disk
delta = sqrt(1.5^2 + 1);

% Set possible angles for dog trajectory
angles = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2, 7*pi/4];

% Set the dog and (maximum) sheep velocity
% dog_velocity = 0.5*r.max_linear_velocity;
% sheep_velocity = 0.25*r.max_linear_velocity;
dog_velocity = 0.75*r.max_linear_velocity;
sheep_velocity = 0.375*r.max_linear_velocity;

%% Initialize video recording

% Create video writer and define video quality and frame rate
vid = VideoWriter(vidPath); vid.Quality = 100; vid.FrameRate = 10; open(vid);

%% Initialize conversion tools from single-integrator to unicycle dynamics

% Create mapping functions
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

% Create barrier certificates
% uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary('SafetyRadius', 0.06);

% Single-integrator position controller
controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.1);

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);

% Initialize flag for stopping loop and counter
FLAG = true; counter = 1;

%% Initialize data to be plotted

% Plotting Initialization
% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = rand(N,3);%{'.-k','.-b','.-r','.-g','.-m','.-y','.-c'};

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 5;

start_time = tic; %The start time to compute time elapsed.

for i = 1:N
    % Initialize additional information plot here. Note the order of
    % plotting matters, objects that are plotted later will appear over
    % object plotted previously.
    
    % Text for robot identification
    robot_caption = sprintf('Robot %d', i);
    % Text with goal identification
    goal_caption = sprintf('G%d', i);
    % Plot colored square for goal location.
    d(i) = plot(waypoints(1, i), waypoints(2, i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i,:));
    % Plot the goal identification text inside the goal location
    goal_labels{i} = text(waypoints(1, i) - 0.05, waypoints(2, i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    % Plot the robot label text 
    robot_labels{i} = text(500, 500, robot_caption, 'FontSize', font_size, 'FontWeight', 'bold');
end

% Plot the iteration and time in the lower left. Note when run on your 
% computer, this time is based on your computers simulation time. For a
% better approximation of experiment time on the Robotarium when running
% this simulation on your computer, multiply iteration by 0.033. 
iteration_caption = sprintf('Iteration %d', 0);
time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));

iteration_label = text(-1.5, -0.8, iteration_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');

% We can change the order of plotting priority, we will plot goals on the 
% bottom and have the iteration/time caption on top.
uistack([goal_labels{:}], 'bottom'); % Goal labels are at the very bottom, arrows are now only above the goal labels.
uistack(d, 'bottom');% Goal squares are at the very bottom, goal labels are above the squares and goal arrows are above those.
uistack([iteration_label], 'top'); % Iteration label is on top.
uistack([time_label], 'top'); % Time label is above iteration label.

x = r.get_poses(); r.step();

%% Send robots to waypoints

while FLAG
    
    % Retrieve the most recent poses and convert to SI states
    x = r.get_poses(); xi = uni_to_si_states(x);
    
    % Trip the flag if it all agents are already in correct positions
    distances = zeros(N, 1);
    for i = 1:N; distances(i) = norm(x(1:2, i) - waypoints(:, i)); end
    FLAG = false; for i = 1:N; if distances(i) > close_enough; FLAG = true; end; end
    
    % Update plotting information and locations
    for q = 1:N; robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15]; end
    
    % Define SI commands based on waypoints
    for i = 1:N; dxi(:, i) = controller(x(1:2, i), waypoints(:, i)); end
    
    % Avoid actuator error by thresholding output velocity
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    % Map SI to Uni dynamics and utilize barrier certificates
    dxu = si_to_uni_dyn(dxi, x); % dxu = uni_barrier_cert_boundary(dxu, x);
    
    % Set velocities and send velocities to agents
    r.set_velocities(1:N, dxu); r.step();
    
    % Increment the counter and determine whether to stop execution
    counter = counter + 1; if counter > iterations; FLAG = false ;end
    
    % Update Iteration and Time marker
    iteration_caption = sprintf('Iteration %d', counter);
    time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
    iteration_label.String = iteration_caption;
    time_label.String = time_caption;
    
    % Add frame to video
    if(mod(counter,skip) == 1); writeVideo(vid, getframe(gcf)); end
    
end

%%

% Reset counter and remove goal labels and boxes
counter = 1; d.delete; for i = 1:N; goal_labels{i}.delete; end

% Reset flag and reset number of iterations
FLAG = true; iterations = 2000;

% Plot initial lines on the figure
% Compute adjacency matrix
[~, A] = computeLaplacian(N, x, delta); c = 1;
% Plot lines
for i = 1:N
    if A(1, i) == 1
        lf{c} = line([x(1, 1), x(1, i)], [x(2, 1), x(2, i)], 'LineWidth', 1, 'Color', 'k');
        c = c + 1;
    end
end

%Iterate for the previously specified number of iterations
while FLAG
    
    % Compute the delta disk graph laplacian
    [L, A] = computeLaplacian(N, x, delta);
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    
    %% Determine dog heading
        
    % Get sheep/dog positions
    sheep_positions = xi(:, dogs+1:end);
    dog_position = xi(:, 1);

    % For each dog...
    for i = 1:dogs
    
        if(mod(counter,skip) == 1)
            % Choose an angle for the dog that maximizes the expected return (ER)
            ER = zeros(size(angles));           % Initialize ER placeholder
            % For each angle...
            for angle = 1:length(angles)
                % Move the dog by that angle
                new_state = updateState(sheep_positions, dog_position, angles(angle), ...
                    dog_velocity, sheep_velocity, delta);
                ER(angle) = net(new_state');    % Estimate ER for the angle
            end; [B, I] = max(ER); dog_angle = angles(I);
        end
        % Return dog's heading
        dxi(1, i) = dog_velocity*cos(dog_angle);
        dxi(2, i) = dog_velocity*sin(dog_angle);

        % Enforce boundaries
        if abs(xi(1, i) + dxi(1, i)) > 1.5 && abs(xi(2, i) + dxi(2, i)) < 1.0; dxi(1, i) = 0; end
        if abs(xi(2, i) + dxi(2, i)) > 1.0 && abs(xi(1, i) + dxi(1, i)) < 1.5; dxi(2, i) = 0; end
    
    end
    
    %% Update sheep headings
    
    for i = 2:N  
        if(mod(counter,skip) == 1)
            % Initialize the SI command for the agent
            dxi(:, i) = [0; 0];

            % Determine the topological neighbors of the agent
            neighbors = topological_neighbors(L, i);

            % For each neighbor...
            for j = 1:length(neighbors)

                % If the neighbor is a dog...
                if neighbors(j) <= dogs
                    % Get the distance from the dog
                    dist = norm(xi(:, i) - xi(:, neighbors(j)));
                    % Compute the weight based on the distance
                    w = 1/dist^2;
                    % Compute update
                    dxi(:, i) = dxi(:, i) + w*(xi(:, i) - xi(:, neighbors(j)));
                end

            end

            % Limit the sheep velocity
            if norm(dxi(:, i)) > sheep_velocity
                dxi(:, i) = sheep_velocity*dxi(:, i)/norm(dxi(:, i));
            end
        end
        
        % Enforce boundaries
        if abs(xi(1, i) + dxi(1, i)) > 1.5 && abs(xi(2, i) + dxi(2, i)) < 1.0; dxi(1, i) = 0; end
        if abs(xi(2, i) + dxi(2, i)) > 1.0 && abs(xi(1, i) + dxi(1, i)) < 1.5; dxi(2, i) = 0; end
        if abs(xi(2, i) + dxi(2, i)) > 1.0 && abs(xi(1, i) + dxi(1, i)) > 1.5
            dxi(2, i) = 0; dxi(1, i) = 0;
        end
        
    end
    
    % Threshold dxi to avoid errors
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    % Transform the single-integrator to unicycle dynamics and utilize
    % barrier certificates
    dxu = si_to_uni_dyn(dxi, x); % dxu = uni_barrier_cert_boundary(dxu, x);
    
    % Set velocities and send to agents
    r.set_velocities(1:N, dxu); r.step();
    
    % Update plotting information and locations
    for q = 1:N; robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15]; end
    
    % Increment the counter and determine whether to stop execution
    counter = counter + 1; if counter > iterations; FLAG = false ;end
    
    % Update Iteration and Time marker
    iteration_caption = sprintf('Iteration %d', counter);
    time_caption = sprintf('Total Time Elapsed %0.2f', toc(start_time));
    iteration_label.String = iteration_caption;
    time_label.String = time_caption;
    
    % Remove lines
    if exist('lf', 'var'); for i = 1:length(lf); lf{i}.delete; end; clear lf; end
    
    % Plot new lines
    c = 1;
    for i = 1:N
        if A(1, i) == 1
            lf{c} = line([x(1, 1), x(1, i)], [x(2, 1), x(2, i)], 'LineWidth', 1, 'Color', 'k');
            c = c + 1;
        end
    end
    
    % Add frame to video
    if(mod(counter,skip) == 1); writeVideo(vid, getframe(gcf)); end
    
end

% Close video
close(vid);

% We can call this function to debug our experiment!
r.debug();

%% Helper Functions

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end

% Compute the graph laplacian and return laplacian (L) and adjacency (A)
function [L, A] = computeLaplacian(N, states, delta)

A = zeros(N, N); D = zeros(N, N);
for i = 1:N
    neighbors = delta_disk_neighbors(states, i, delta);
    if ~isempty(neighbors)
        for j = 1:length(neighbors)
            A(i, neighbors(j)) = 1; A(neighbors(j), i) = 1;
        end
        D(i, i) = length(neighbors);
    end
end; L = D - A;

end