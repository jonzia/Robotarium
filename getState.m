function state = getState(sheep_positions, dog_position)
state = [sheep_positions(1, :) sheep_positions(2, :) dog_position'];
end

