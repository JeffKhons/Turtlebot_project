%% 清除所有資料
clc; clear; close all;

%% 連結 MATLAB 與自走車
setenv("ROS_DOMAIN_ID", "30");
mtnode = ros2node("/matlab_turtlebot");
pause(6);

%% 載入場域的佔據柵格地圖
load map_20230804.mat

% 畫出環境地圖
figure(1);
show(map);

%% 定義起終點
sGoal=[   0,   0.0,   pi/2;
        0.6,  1.00,   pi/2];

% 定義起始點與目標點
start = sGoal(1, :);
goal  = sGoal(2, :);

% 於地圖中標注起始點與目標點
hold on;
plot(start(1), start(2), 'b*', 'MarkerSize', 3);
plot( goal(1),  goal(2), 'r*', 'MarkerSize', 3);
hold off;

%% 定義路徑規劃函式資料
stSpace = stateSpaceSE2;
stValidator = validatorOccupancyMap(stSpace); 
stValidator.ValidationDistance = 0.1;

% 定義 RRT* 函式資料
planner = plannerRRTStar(stSpace, stValidator);
planner.ContinueAfterGoalReached = true;
planner.MaxConnectionDistance = 0.2;          
planner.MaxIterations = 3000;                    

% 進行膨脹處理
mapInflated = copy(map);
inflate(mapInflated, 0.07);

% 畫出環境地圖
figure(2);
show(mapInflated);

rng(10, 'twister')

% 計算起始點到目標點的距離
goalDist = norm(goal(1:2)-start(1:2));
pathLong = 2 * goalDist;

% 定義路徑規劃參考地圖與範圍
stSpace.StateBounds = [ mapInflated.XWorldLimits;...
                        mapInflated.YWorldLimits;...
                                       [-pi pi]];
stValidator.Map = mapInflated;

%% 進行路徑規劃
while ((pathLong >= 1.5*goalDist))
    % 進行路徑規劃運算
    [pthObj, solnInfo] = plan(planner, start, goal);
    
    % 如果無法運算結果則放寬給定條件
    while pthObj.NumStates == 0
      planner.MaxIterations = planner.MaxIterations + 100;
      [pthObj, solnInfo] = plan(planner, start, goal)
      disp('planner.MaxIterations + 100!');
    end

    % 畫出膨脹處理後的環境地圖	
    figure(3);
    show(mapInflated);
    hold on

    % 畫出樹狀分枝		
    plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

    % 畫出規劃出的理想路徑	
    plot(pthObj.States(:, 1), pthObj.States(:, 2), 'r-', 'LineWidth', 2);

    % 畫出起始點與目標點	
    plot(start(1), start(2), 'g*', 'MarkerSize', 5);
    plot(goal(1), goal(2), 'r*', 'MarkerSize', 5);
    hold off

    % 連結起始點與目標點後的理想路徑
    path = pthObj.States(:, 1:2);
    if path(end, :) ~= goal(1:2)
      path = [path; goal(1:2)];
    end

    % 計算行駛路徑長度
    pathLong = 0;
    for i = 1:(length(path)-1)
        pathLong = pathLong + norm(path(i,:) - path(i+1,:));
    end
 
    % 檢查是否為有效的理想路徑 
    pathMetricsObj = pathmetrics(pthObj, stValidator);
    if ~isPathValid(pathMetricsObj)
        disp('Invalid path');
        pathLong = 2 * goalDist;
    else
        % 檢查是否為有太靠近障礙物狀況
        if clearance(pathMetricsObj) < 0.1
            disp('clearance is inValid.');
            show(pathMetricsObj,'Metrics',{'StatesClearance'});
            pathLong = 2 * goalDist;
        else
            % 符合檢查條件	  
            disp('Valid path.');  
        end
    end
end
% 畫出環境地圖並包含理想路徑及起始點與目標點
figure(4);
show(map);
hold on;
plot(path(:, 1), path(:, 2), 'k--',  'LineWidth',  3);
plot(  start(1),   start(2),  'b*', 'MarkerSize',  5);
plot(   goal(1),    goal(2),  'r*', 'MarkerSize',  5);
hold off;

path