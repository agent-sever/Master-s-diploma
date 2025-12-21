% --- Завантаження карти ---
load exampleMaps.mat simpleMap
assignin('base', 'simpleMap', simpleMap);

% --- Спільна ціль ---
goalLoc = [5 5];
assignin('base', 'goalLoc', goalLoc);

% --- Агент 1 ---
startLoc = [9 9];  % старт
assignin('base', 'startLoc', startLoc);
simData1 = sim('pathPlanning.slx');

% --- Агент 2 ---
startLoc = [20 20];
assignin('base', 'startLoc', startLoc);
simData2 = sim('pathPlanning.slx');

% --- Початковий заряд батареї ---
battery1 = 100;  
battery2 = 100;  
assignin('base', 'battery1', battery1);
assignin('base', 'battery2', battery2);

% --- Візуалізація карти ---
map = binaryOccupancyMap(simpleMap);
figure;
show(map)
hold on

plot(goalLoc(1), goalLoc(2), 'rx', 'MarkerSize', 8, 'LineWidth', 2)
text(goalLoc(1)+0.5, goalLoc(2), 'Dock')

plot(9, 9, 'go', 'MarkerSize', 8, 'LineWidth', 2)
text(9+0.5, 9, 'Start 1')
plot(20, 20, 'go', 'MarkerSize', 8, 'LineWidth', 2)
text(20+0.5, 20, 'Start 2')

pose1 = simData1.Pose;
plot(pose1(:,1), pose1(:,2), '-b', 'LineWidth', 1.5)
pose2 = simData2.Pose;
plot(pose2(:,1), pose2(:,2), '-r', 'LineWidth', 1.5)

lgd = legend('Dock', 'Start 1', 'Start 2', 'Agent 1', 'Agent 2', 'Map');
title('')
lgd.Position = [0.84 0.77 0.15 0.1];

grid on;

% --- Отримуємо межі графіка ---
ax = gca;
fig = gcf;
xLimits = xlim;
yLimits = ylim;

% Приховуємо звичайні осі
% set(ax, 'XColor', 'none', 'YColor', 'none');
% Залишаємо осі видимими
set(ax, 'XColor', 'k', 'YColor', 'k'); % чорні осі

% Прибираємо стандартні підписи осей
xlabel(''); 
ylabel('');

% --- Відступ для осей від країв графіка ---
xOffset = -2.5;  % для Y-осі
yOffset = 0.0;  % для X-осі

% --- Стрілка осі X ---
[x1_fig, y1_fig] = ds2nfu(ax, xLimits(1)+2.5, yLimits(1)-yOffset);
[x2_fig, y2_fig] = ds2nfu(ax, xLimits(2)+2.5, yLimits(1)-yOffset);
annotation(fig,'arrow', [x1_fig x2_fig], [y1_fig y2_fig], 'LineWidth', 1.5);

% --- Стрілка осі Y ---
[x3_fig, y3_fig] = ds2nfu(ax, xLimits(1)-xOffset, yLimits(1));
[x4_fig, y4_fig] = ds2nfu(ax, xLimits(1)-xOffset, yLimits(2)+2);
annotation(fig,'arrow', [x3_fig x4_fig], [y3_fig y4_fig], 'LineWidth', 1.5);

% --- Підпис осі X (трохи правіше) ---
text(mean([xLimits(1) xLimits(2)]) + 17, yLimits(1)-yOffset-1, 'X, м', ...
    'FontSize', 14, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

% --- Підпис осі Y (вище) ---
text(xLimits(1)-xOffset-3, yLimits(2) + 2, 'Y, м', ...
    'FontSize', 14, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'Rotation', 0);

hold off
