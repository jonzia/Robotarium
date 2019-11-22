function [new_state, varargout] = updateState(sheep_positions, dog_position, dog_angle, velocity, sheep_velocity, delta)

% Move the dog
dog_position(1) = dog_position(1) + velocity*cos(dog_angle);
dog_position(2) = dog_position(2) + velocity*sin(dog_angle);
dog_position(1) = min(dog_position(1), 1.5);
dog_position(1) = max(dog_position(1), -1.5);
dog_position(2) = min(dog_position(2), 1);
dog_position(2) = max(dog_position(2), -1);

% For each sheep, compute the updated position
for k = 1:size(sheep_positions, 2)

    % Get the distance between sheep and dog
    dist = norm(sheep_positions(:, k) - dog_position');

    % Compute weight based on delta disk
    if dist < delta; w = 1/dist^2; else; w = 0; end

    % Compute step
    dx = w*(sheep_positions(:, k) - dog_position);

    % Threshold step
    if norm(dx) > sheep_velocity; dx = sheep_velocity*dx/norm(dx); end
    
    % Update sheep position
    sheep_positions(:, k) = sheep_positions(:, k) + dx; 

    % Enforce boundaries
    sheep_positions(1, k) = min(sheep_positions(1, k), 1.5);
    sheep_positions(1, k) = max(sheep_positions(1, k), -1.5);
    sheep_positions(2, k) = min(sheep_positions(2, k), 1);
    sheep_positions(2, k) = max(sheep_positions(2, k), -1);

end

new_state = getState(sheep_positions, dog_position);

if nargout > 1; varargout{1} = sheep_positions; end
if nargout > 2; varargout{2} = dog_position; end

end

