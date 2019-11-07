function cost = cost(sheep_positions, from)

% Get the number of sheep
sheep = size(sheep_positions, 2);

% Initialize return value
cost = 0;

% For each sheep, compute the cost
for i = 1:sheep
    cost = cost + norm(sheep_positions(:, i) - from)^2;
end

% Get the cost per sheep
cost = cost/sheep;

end

