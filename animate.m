function animate(validation_data, reward, lambda, start_iteration, end_iteration, numSheep)

% figure; hold on; grid on;
if numSheep == 2
    s1 = scatter(0, 0, 'or', 'MarkerFaceColor', 'r');
    s2 = scatter(0, 0, 'or', 'MarkerFaceColor', 'r');
else
    s = scatter(0, 0, 'or', 'MarkerFaceColor', 'r');
end
d = scatter(1, -1, 'ok', 'MarkerFaceColor', 'k'); M = 0.25;
l1 = line([d.XData d.XData + M*cos(0)], [d.YData d.YData + M*sin(0)], 'Color', [0, 0, 0], 'LineWidth', 2);
l2 = line([d.XData d.XData + M*cos(pi/4)], [d.YData d.YData + M*sin(pi/4)], 'Color', [0, 0, 0], 'LineWidth', 2);
l3 = line([d.XData d.XData + M*cos(pi/2)], [d.YData d.YData + M*sin(pi/2)], 'Color', [0, 0, 0], 'LineWidth', 2);
l4 = line([d.XData d.XData + M*cos(3*pi/4)], [d.YData d.YData + M*sin(3*pi/4)], 'Color', [0, 0, 0], 'LineWidth', 2);
l5 = line([d.XData d.XData + M*cos(pi)], [d.YData d.YData + M*sin(pi)], 'Color', [0, 0, 0], 'LineWidth', 2);
l6 = line([d.XData d.XData + M*cos(5*pi/4)], [d.YData d.YData + M*sin(5*pi/4)], 'Color', [0, 0, 0], 'LineWidth', 2);
l7 = line([d.XData d.XData + M*cos(3*pi/2)], [d.YData d.YData + M*sin(3*pi/2)], 'Color', [0, 0, 0], 'LineWidth', 2);
l8 = line([d.XData d.XData + M*cos(7*pi/4)], [d.YData d.YData + M*sin(7*pi/4)], 'Color', [0, 0, 0], 'LineWidth', 2);

line([-1.5 1.5], [1, -1], 'Color', 'k', 'LineStyle', '--')
line([-1.5 1.5], [1, 1], 'Color', 'k', 'LineWidth', 2)
line([-1.5 1.5], [-1, -1], 'Color', 'k', 'LineWidth', 2)
line([-1.5 -1.5], [-1, 1], 'Color', 'k', 'LineWidth', 2)
line([1.5 1.5], [-1, 1], 'Color', 'k', 'LineWidth', 2)

xlim([-1.5, 1.5]); ylim([-1.5, 1.5])

for i = start_iteration:end_iteration
    
    startIdx = lambda*(i-1) + 1; endIdx = startIdx + (lambda - 1);
    data = validation_data(startIdx:endIdx, :);
    weights = reward(startIdx:endIdx, :);
    if ~isempty(find(isnan(weights(:)), 1)); continue; end
    
    for j = 1:lambda
        
        title("Iteration: " + i + ", Step: " + j)
        
        if numSheep == 2
            s1.XData = data(j, 1); s1.YData = data(j, 3);
            s2.XData = data(j, 2); s2.YData = data(j, 4);
            d.XData = data(j, 5); d.YData = data(j, 6);
        else
            s.XData = data(j, 1); s.YData = data(j, 2);
            d.XData = data(j, 3); d.YData = data(j, 4);
        end
        l1.XData = [d.XData d.XData + M*cos(0)];
        l1.YData = [d.YData d.YData + M*sin(0)];
        l2.XData = [d.XData d.XData + M*cos(pi/4)];
        l2.YData = [d.YData d.YData + M*sin(pi/4)];
        l3.XData = [d.XData d.XData + M*cos(pi/2)];
        l3.YData = [d.YData d.YData + M*sin(pi/2)];
        l4.XData = [d.XData d.XData + M*cos(3*pi/4)];
        l4.YData = [d.YData d.YData + M*sin(3*pi/4)];
        l5.XData = [d.XData d.XData + M*cos(pi)];
        l5.YData = [d.YData d.YData + M*sin(pi)];
        l6.XData = [d.XData d.XData + M*cos(5*pi/4)];
        l6.YData = [d.YData d.YData + M*sin(5*pi/4)];
        l7.XData = [d.XData d.XData + M*cos(3*pi/2)];
        l7.YData = [d.YData d.YData + M*sin(3*pi/2)];
        l8.XData = [d.XData d.XData + M*cos(7*pi/4)];
        l8.YData = [d.YData d.YData + M*sin(7*pi/4)];
        col = weights(j, :) - min(weights(j, :));
        col = col./max(col);
        l1.Color = 1 - [col(1), col(1), col(1)];
        l2.Color = 1 - [col(2), col(2), col(2)];
        l3.Color = 1 - [col(3), col(3), col(3)];
        l4.Color = 1 - [col(4), col(4), col(4)];
        l5.Color = 1 - [col(1), col(1), col(1)];
        l6.Color = 1 - [col(2), col(2), col(2)];
        l7.Color = 1 - [col(3), col(3), col(3)];
        l8.Color = 1 - [col(4), col(4), col(4)];
        
        pause(0.5)
        
    end
    
end

end

