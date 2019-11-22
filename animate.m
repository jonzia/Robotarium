function animate(validation_data, reward, lambda, varargin)

% Optional arguments:
% - 'start'         Iteration at which to start animation   (default 1)
% - 'end'           Iteration at which to stop animation
% - 'sheep'         Number of sheep
% - 'pause'         Duration to pause between steps         (default 0.25)
% - 'angles'        Angles of possible directions (default 0:pi/4:7*pi/4)
% - 'video'         Filepath to video file for saving       (default none)

% Parse optional input arguments
if ~isempty(varargin)
    for arg = 1:length(varargin)
        if strcmp(varargin{arg}, 'start'); start_iteration = varargin{arg + 1};
        elseif strcmp(varargin{arg}, 'end'); end_iteration = varargin{arg + 1};
        elseif strcmp(varargin{arg}, 'sheep'); numSheep = varargin{arg + 1};
        elseif strcmp(varargin{arg}, 'pause'); pauseTime = varargin{arg + 1};
        elseif strcmp(varargin{arg}, 'angles'); angles = varargin{arg + 1};
        elseif strcmp(varargin{arg}, 'video'); vidFLAG = true; vidPath = varargin{arg + 1};
        end
    end
end

% Set defaults for optional input arguments
if ~exist('start_iteration', 'var'); start_iteration = 1; end
if ~exist('end_iteration', 'var'); end_iteration = size(validation_data, 1)/lambda; end
if ~exist('numSheep', 'var'); numSheep = (size(validation_data, 2) - 2)/2; end
if ~exist('pauseTime', 'var'); pauseTime = 0.25; end
if ~exist('angles', 'var'); angles = 0:pi/4:7*pi/4; end
if ~exist('vidFLAG', 'var'); vidFLAG = false; end

% Set placeholder for sheep indicators and graph lines
s = cell(numSheep, 1); l = cell(length(angles), 1);

% Initialize the sheep indicators
for i = 1:numSheep
    s{i} = scatter(0, 0, 'or', 'MarkerFaceColor', 'r');
end

% Initialize dog indicator
d = scatter(1, -1, 'ok', 'MarkerFaceColor', 'k'); M = 0.25;

% Initialize lines for possible directions
for i = 1:length(angles)
    l{i} = line([d.XData d.XData + M*cos(angles(i))], [d.YData d.YData + M*sin(angles(i))], ...
        'Color', [0, 0, 0], 'LineWidth', 2);
end

% Initialize boundary lines
line([-1.5 1.5], [1, -1], 'Color', 'k', 'LineStyle', '--')
line([-1.5 1.5], [1, 1], 'Color', 'k', 'LineWidth', 2)
line([-1.5 1.5], [-1, -1], 'Color', 'k', 'LineWidth', 2)
line([-1.5 -1.5], [-1, 1], 'Color', 'k', 'LineWidth', 2)
line([1.5 1.5], [-1, 1], 'Color', 'k', 'LineWidth', 2)

% Set limits on axes
xlim([-1.5, 1.5]); ylim([-1.5, 1.5])

% Initialize video
if vidFLAG
    vid = VideoWriter(vidPath);
    vid.Quality = 100; vid.FrameRate = 10;
    open(vid);
end

% For each iteration...
for i = start_iteration:end_iteration
    
    % Set placeholders for iteration
    startIdx = lambda*(i-1) + 1; endIdx = startIdx + (lambda - 1);
    data = validation_data(startIdx:endIdx, :);
    weights = reward(startIdx:endIdx, :);
    if ~isempty(find(isnan(weights(:)), 1)); continue; end
    
    % For each step in the iteration
    for j = 1:lambda
        
        % Set figure title
        title("Iteration: " + i + ", Step: " + j)
        
        % Initialize counter for parsing validation data
        counter = 1;
        
        % Update the x and y data for each sheep
        for k = 1:numSheep
            s{k}.XData = data(j, counter); counter = counter + 1;
        end
        for k = 1:numSheep
            s{k}.YData = data(j, counter); counter = counter + 1;
        end
        
        % Update the x and y data for the dog
        d.XData = data(j, counter); counter = counter + 1;
        d.YData = data(j, counter);
        
        % Update the movement lines
        for k = 1:length(angles)
            l{k}.XData = [d.XData d.XData + M*cos(angles(k))];
            l{k}.YData = [d.YData d.YData + M*sin(angles(k))];
        end
        
        % Update the weights such that the minimum value is 0
        col = weights(j, :) - min(weights(j, :));
        col = col./max(col);    % Normalize to range [0, 1]
        
        % Update the colors of each line based on the weights
        try
            for k = 1:length(angles)
                l{k}.Color = 1 - [col(k) col(k) col(k)];
            end
        end
            
        
        % Record video frame
        if vidFLAG; writeVideo(vid, getframe(gcf)); end
        
        % Pause animation
        pause(pauseTime)
        
    end
    
end

% End video, if necessary
if vidFLAG; close(vid); end

end

