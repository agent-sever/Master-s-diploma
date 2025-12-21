% Центр: скрипт з реалістичним рухом і довгим часом очікування

clearvars -except simData1 simData2 simpleMap goalLoc
close all

%% --- Параметри витрати енергії (після clearvars!) ---
a = 0.5;      % постійна частина %/с (електроніка)
b = 0.8;      % лінійний коефіцієнт від v
c = 3;      % квадратичний коефіцієнт (аеродинаміка)

idle_rate = 0.05;  % %/с під час очікування у зоні
stop_rate = 0.01; % %/с при повній стоянці/вимкненні

dt = 1; % одна ітерація циклу = 1 сек
batteryDrainPerSec_move = @(v) max(0, a + b.*v + c.*v.^2); % функція від v
k=0.6;
% Масиви для збереження історії (для графіків)
maxSimTime = 10000;
battery1Array = nan(maxSimTime,1);
battery2Array = nan(maxSimTime,1);
timeArray     = nan(maxSimTime,1);
speed1Array   = nan(maxSimTime,1);
speed2Array   = nan(maxSimTime,1);

%% --- Ініціалізація фігури і карти ---
figure('Name','Координація групи дронів');
map = binaryOccupancyMap(simpleMap);
show(map)
title('Motion modeling')
hold on
%% view([-35 25]);
 view(2);
axis vis3d; axis equal; grid on;
camlight('headlight'); lighting gouraud; material shiny;

start1 = [9,9]; start2 = [20,20];
plot(start1(1), start1(2), 'go', 'MarkerSize',8,'LineWidth',2)
plot(start2(1), start2(2), 'go', 'MarkerSize',8,'LineWidth',2)
plot(goalLoc(1), goalLoc(2), 'rx', 'MarkerSize',8,'LineWidth',2)
text(start1(1)+0.5,start1(2),'Start 1')
text(start2(1)+0.5,start2(2),'Start 2')
text(goalLoc(1)+0.5,goalLoc(2),'Dock')

zoneHalf = 2.5;
rectangle('Position',[goalLoc(1)-zoneHalf, goalLoc(2)-zoneHalf, 2*zoneHalf, 2*zoneHalf],...
    'EdgeColor','m','LineStyle','--','LineWidth',1.2)
light('Position',[10 10 10],'Style','infinite'); lighting gouraud; material shiny

% --- Параметри часу/логіки ---
framesPerSec = 10;    % крок у кадрах ( 10 кадрів = 1 сек)
decisionWindow = 150;  % 15сек
secondDelay = 50; % секунд після DONE для delayed release

% --- Траєкторії з Simulink (очікується, що simData1/2 існують) ---
pose1 = simData1.Pose; % Nx3: [x y yaw]
pose2 = simData2.Pose;
nFrames1 = size(pose1,1);
nFrames2 = size(pose2,1);

% індекси кадрів
idx1 = 1;
idx2 = 1;

% час
timeSec = 0;

% стани
agent1_in_zone = false; agent2_in_zone = false;
agent1_waiting = false; agent2_waiting = false;
agent1_allowed = true; agent2_allowed = true;
agent1_released = false; agent2_released = false;
agent1_done = false; agent2_done = false;

agent1_arrival_time = inf; agent2_arrival_time = inf;
agent1_release_time = inf; agent2_release_time = inf;
agent1_zone_time = -inf; agent2_zone_time = -inf;

% батареї
battery1 = 100; battery2 = 100;

% попередні позиції для виміру швидкості
prevPos1 = pose1(1,1:2);
prevPos2 = pose2(1,1:2);

% графічні об'єкти
hTrail1 = plot(pose1(1,1), pose1(1,2), '-b', 'LineWidth', 1.5);
hTrail2 = plot(pose2(1,1), pose2(1,2), '-r', 'LineWidth', 1.5);
hText1 = text(0,0,'','FontSize',9,'Color','k');
hText2 = text(0,0,'','FontSize',9,'Color','k');

% підготовка STL 
hasSTL = true;
try fv = stlread('groundvehicle.stl'); catch; fv = []; hasSTL=false; end

% Вхід: fv 
V = []; F = [];
if hasSTL && ~isempty(fv)
    if isstruct(fv)
        % Різні формати: vertices/faces або Vertices/Faces
        if isfield(fv,'vertices') && isfield(fv,'faces')
            V = double(fv.vertices);
            F = double(fv.faces);
        elseif isfield(fv,'Vertices') && isfield(fv,'Faces')
            V = double(fv.Vertices);
            F = double(fv.Faces);
        else
            % Пошук числових масивів у полях структури
            fld = fieldnames(fv);
            for i=1:numel(fld)
                val = fv.(fld{i});
                if isnumeric(val) && size(val,2)==3 && isempty(V)
                    V = double(val);
                elseif isnumeric(val) && (size(val,2)==3 || size(val,2)==4) && isempty(F)
                    F = double(val);
                end
            end
        end
    elseif iscell(fv) && numel(fv) >= 2
        try
            A = double(fv{1});
            B = double(fv{2});
            if size(A,2)==3 && size(B,2)>=3
                V = A; F = B;
            elseif size(B,2)==3 && size(A,2)>=3
                V = B; F = A;
            else
                V = []; F = [];
            end
        catch
            V = []; F = [];
        end
    elseif isnumeric(fv) && size(fv,2)==3
        % Рідкісний випадок — один масив вершин
        V = double(fv);
        F = [];
    end
end

% Нормалізація індексів (0-based → 1-based), триангуляція, перевірки
if ~isempty(F)
    F = double(F);
    if any(F(:)==0)
        F = F + 1;
    end
    if size(F,2) == 4
        Q = F;
        F = zeros(size(Q,1)*2,3);
        t = 1;
        for i=1:size(Q,1)
            a=Q(i,1); b=Q(i,2); c=Q(i,3); d=Q(i,4);
            F(t,:) = [a b c]; t=t+1;
            F(t,:) = [a c d]; t=t+1;
        end
    end
    if iscell(F)
        try F = cell2mat(F); catch; end
    end
end

% Якщо не вдалося прочитати STL — застосувати fallback box-модель
if isempty(V) || isempty(F)
    %warning('Не вдалося прочитати groundvehicle.stl — застосовано fallback box-модель.');
    w = 0.7; l = 1.2; h = 0.4;
    V = [ -l/2 -w/2 0;
           -l/2  w/2 0;
            l/2  w/2 0;
            l/2 -w/2 0;
           -l/2 -w/2 h;
           -l/2  w/2 h;
            l/2  w/2 h;
            l/2 -w/2 h];
    F = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 8; 3 8 7; 4 1 5; 4 5 8];
end

% Центруємо і масштабуємо модель
V = double(V);
V = V - mean(V,1);
scaleSTL = 0.6; 
V = V * scaleSTL;


zMin = min(V(:,3));
if abs(zMin) < 1e-6
    V = V + repmat([0 0 0.02], size(V,1),1);
end

% Створюємо patch-и для двох агентів (один раз)
agent1_patch = patch('Faces',F,'Vertices',V,'FaceColor',[0 0.2 0.8], ...
    'EdgeColor','none','FaceLighting','gouraud','FaceAlpha',1);
agent2_patch = patch('Faces',F,'Vertices',V,'FaceColor',[0.8 0.1 0.1], ...
    'EdgeColor','none','FaceLighting','gouraud','FaceAlpha',1);


% ------------------------------
% ГОЛОВНИЙ ЦИКЛ
centerWaitingLogged = false;
centerDecisionMade  = false;
% ------------------------------
while ~(agent1_done && agent2_done) && timeSec < maxSimTime
    timeSec = timeSec + 1;
    
    % ----------------- AGENT 1 -----------------
    % Оновлення індексу (якщо дозволено)
    if agent1_allowed && idx1 < nFrames1
        % крок вперед: рухаємося на framesPerSec кадрів (щоб відповідати timestep)
        idx1 = min(nFrames1, idx1 + framesPerSec);
    end
    
    % Поточна позиція
    cur1 = pose1(idx1, :);
    % швидкість (реалістична: зміна позиції між ітераціями)
    dist1 = norm(cur1(1:2) - prevPos1);
    v1 = dist1 / dt;
    prevPos1 = cur1(1:2);
    
    % Обчислення витрати
    if agent1_allowed
        drain1 = batteryDrainPerSec_move(v1);
        battery1 = max(0, battery1 - drain1*dt);
    elseif agent1_waiting
        battery1 = max(0, battery1 - idle_rate*dt);
    else
        battery1 = max(0, battery1 - stop_rate*dt);
    end
    
    % Оновлення графіки для агента 1
    set(hTrail1, 'XData', pose1(1:idx1,1), 'YData', pose1(1:idx1,2));
    set(hText1, 'Position', [cur1(1)+0.3, cur1(2)], 'String', sprintf('A1 %.0f%%', battery1));
    % оновлення patch'а 
    theta1 = cur1(3);
    Rz1 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];
    newV1 = (Rz1 * (V)')' + repmat([cur1(1) cur1(2) 0], size(V,1), 1);
    set(agent1_patch, 'Vertices', newV1);
    
    % В'їзд у зону
    if ~agent1_in_zone && norm(cur1(1:2) - goalLoc) <= zoneHalf
        agent1_in_zone = true;
        agent1_waiting = true;
        agent1_allowed = false;
        agent1_zone_time = timeSec;
        fprintf('[%2ds] ○ Агент 1 увійшов у зону. Запит на координацію надіслано.\n', timeSec);
        fprintf('[%2ds] ○ Центр: очікує ще запити...\n', timeSec);
    end
   if idx1 >= nFrames1 && ~agent1_done
    agent1_done = true;
    agent1_allowed = false;
    agent1_waiting = false;
    agent1_arrival_time = timeSec;
    fprintf('[%2ds] ○ Агент 1 досяг док-станції.\n', timeSec);
    
    % Якщо агент 2 чекає — встановлюємо час delayed release
    if agent2_waiting && ~agent2_released
        agent2_release_time = agent1_arrival_time + secondDelay;
    end
end

    
    % ----------------- AGENT 2 -----------------

    if agent2_allowed && idx2 < nFrames2
        idx2 = min(nFrames2, idx2 + framesPerSec);
    end
    cur2 = pose2(idx2, :);
    dist2 = norm(cur2(1:2) - prevPos2);
    v2 = (dist2 / dt)*k; 
    prevPos2 = cur2(1:2);
    
    if agent2_allowed
        drain2 = batteryDrainPerSec_move(v2)*k;
        battery2 = max(0, battery2 - drain2*dt);
    elseif agent2_waiting
        battery2 = max(0, battery2 - idle_rate*dt);
    else
        battery2 = max(0, battery2 - stop_rate*dt);
    end
    
    set(hTrail2, 'XData', pose2(1:idx2,1), 'YData', pose2(1:idx2,2));
    set(hText2, 'Position', [cur2(1)+0.3, cur2(2)], 'String', sprintf('A2 %.0f%%', battery2));
    theta2 = cur2(3);
    Rz2 = [cos(theta2) -sin(theta2) 0; sin(theta2) cos(theta2) 0; 0 0 1];
    newV2 = (Rz2 * (V)')' + repmat([cur2(1) cur2(2) 0], size(V,1), 1);
    set(agent2_patch, 'Vertices', newV2);
    
    if ~agent2_in_zone && norm(cur2(1:2) - goalLoc) <= zoneHalf
        agent2_in_zone = true;
        agent2_waiting = true;
        agent2_allowed = false;
        agent2_zone_time = timeSec;
        fprintf('[%2ds] ○ Агент 2 увійшов у зону. Запит на координацію надіслано.\n', timeSec);
        fprintf('[%2ds] ○ Центр: очікує ще запити...\n', timeSec);
    end
    if idx2 >= nFrames2 && ~agent2_done
    agent2_done = true;
    agent2_allowed = false;
    agent2_waiting = false;
    agent2_arrival_time = timeSec;
    fprintf('[%2ds] ○ Агент 2 досяг док-станції.\n', timeSec);
    % Якщо агент 1 чекає — встановлюємо час delayed release
    if agent1_waiting && ~agent1_released
        agent1_release_time = agent2_arrival_time + secondDelay;
    end
    end
    
% ----------------- LOGІКА ЦЕНТРА (універсальна) -----------------
agentsWaiting = [agent1_waiting, agent2_waiting];
agentsZoneTime = [agent1_zone_time, agent2_zone_time];
agentsBattery = [battery1, battery2];

if any(agentsWaiting)
    % Центр чекає, поки хоча б один агент перебуває у зоні
    waitingIndices = find(agentsWaiting);
    maxWait = max(timeSec - agentsZoneTime(waitingIndices));
    
    if maxWait >= decisionWindow && ~centerDecisionMade
        % Приймаємо рішення після decisionWindow
        % Критерій: мінімальна батарея
        [~, chosenIdx] = min(agentsBattery(waitingIndices));
        chosenAgent = waitingIndices(chosenIdx);
        
        % Дозволяємо рух обраному агенту
        if chosenAgent == 1
            agent1_allowed = true; agent1_waiting = false; agent1_released = true;
            agent2_release_time = timeSec + decisionWindow; % другий чекає ще decisionWindow
            secondAgent = 2;
        else
            agent2_allowed = true; agent2_waiting = false; agent2_released = true;
            agent1_release_time = timeSec + decisionWindow; % другий чекає ще decisionWindow
            secondAgent = 1;
        end
        fprintf('[%2ds] ○ Центр: Агент %d продовжує рух (інший очікує) \n', timeSec, chosenAgent);
        centerDecisionMade = true;
    end
end

% ----------------- DELAYED RELEASE для другого агента -----------------
if centerDecisionMade
    if secondAgent == 1 && ~agent1_released && timeSec >= agent1_release_time
        agent1_allowed = true; agent1_waiting = false; agent1_released = true;
        fprintf('[%2ds] ○ Центр: Агент 1 продовжує рух\n', timeSec);
    elseif secondAgent == 2 && ~agent2_released && timeSec >= agent2_release_time
        agent2_allowed = true; agent2_waiting = false; agent2_released = true;
        fprintf('[%2ds] ○ Центр: Агент 2 продовжує рух\n', timeSec);
    end
end





    
    % ----------------- ЗАПИС ІСТОРІЇ -----------------
    if timeSec <= maxSimTime
        battery1Array(timeSec) = battery1;
        battery2Array(timeSec) = battery2;
        timeArray(timeSec) = timeSec;
        speed1Array(timeSec) = v1;
        speed2Array(timeSec) = v2;
    end
    
    drawnow limitrate
    pause(0.1)
end

%% --- ПІДСУМОК І ГРАФІКИ ---
valid = ~isnan(timeArray);
tvec = timeArray(valid);
b1 = battery1Array(valid);
b2 = battery2Array(valid);

figure('Name','Battery vs Time');
plot(tvec, b1, '-b','LineWidth',1.6); hold on;
plot(tvec, b2, '-r','LineWidth',1.6);
xlabel('Час, сек'); ylabel('Заряд батареї, %');
legend('Агент 1','Агент 2','Location','southwest'); grid on;

% Додаткові графіки (швидкість)
figure('Name','Speed vs Time');
plot(tvec, speed1Array(valid)*100,'--b','LineWidth',1.4); hold on;
plot(tvec, speed2Array(valid)*100,'--r','LineWidth',1.4);
xlabel('Час, сек'); ylabel('Швидкість, од.'); legend('Агент 1','Агент 2'); grid on;

fprintf('\n ПІДСУМОК:\n');
fprintf('Агент 1: %.1f%% заряду залишилось\n', battery1);
fprintf('Агент 2: %.1f%% заряду залишилось\n', battery2);

uiwait(gcf);




