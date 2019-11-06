%% Herding Behavior using Networked Control
% Jacob Kimball, Jonathan Zia
% 11/2019

% Define number of sheep and dogs
sheep = 5; dogs = 4; N = sheep + dogs;

% Define Robotarium object
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% Define initial positions

% Define waypoints for the dogs (defined)
waypoints_dogs = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';

% Define waypoints for the sheep (random in [-1, 1])
waypoints_sheep = 1.4*rand(2, sheep) - 0.7;

% Combine sheep and dog waypoints
waypoints = [waypoints_dogs waypoints_sheep];

% Set error tolerance for waypoint convergence
close_enough = 0.05;

% Set the maximum number of iterations
iterations = 3000;

%% Initialize conversion tools from single-integrator to unicycle dynamics

% Create mapping functions
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

% Create barrier certificates
uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();

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
    distances = zeros(dogs, 1);
    for i = 1:dogs; distances(i) = norm(x(1:2, i) - waypoints_dogs(:, i)); end
    FLAG = false; for i = 1:dogs; if distances(i) > close_enough; FLAG = true; end; end
    
    % Update plotting information and locations
        for q = 1:N
            robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15];
            robot_details = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f', x(1,q), x(2,q));
        end
    
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
    
end

dxi = zeros(2, N);

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    A = zeros(N, N); D = zeros(N, N);
    for i = 1:N
        if ismember(i, 5:9)
            neighbors = delta_disk_neighbors(x, i, 1);
            neighbors(~ismember(neighbors, 1:4)) = [];
            if ~isempty(neighbors)
                for j = 1:length(neighbors)
                    A(i, neighbors(j)) = 1; A(neighbors(j), i) = 1;
                end
                D(i, i) = length(neighbors);
            end
        else
            neighbors = delta_disk_neighbors(x, i, 1);
            neighbors(~ismember(neighbors, 5:9)) = [];
            if ~isempty(neighbors)
                for j = 1:length(neighbors)
                    A(i, neighbors(j)) = 1; A(neighbors(j), i) = 1;
                end
                D(i, i) = length(neighbors);
            end
        end
    end; L = D - A;
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    
    %% Algorithm
    
    for i = 1:N  
        
        dxi(:, i) = [0; 0];
        
        neighbors = topological_neighbors(L, i);
        
        if ismember(i, 5:9)
            for j = 1:length(neighbors)
                dxi(:, i) = dxi(:, i) - (xi(:, neighbors(j)) - xi(:, i));
            end
        else
            for j = 1:length(neighbors)
                dxi(:, i) = dxi(:, i) + (xi(:, neighbors(j)) - xi(:, i));
            end
        end
        
    end
    
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Map SI to Uni dynamics and utilize barrier certificates
    
    % Transform the single-integrator to unicycle dynamics using the the
    % transformation we created earlier
    dxu = si_to_uni_dyn(dxi, x);
    
    dxu = uni_barrier_cert_boundary(dxu, x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
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