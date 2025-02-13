clc; clear; close all
%% 設定共用變數 
vMax = 0.15; 		
vMin = 0.03;
vLimit = [vMax, vMin];
wMax = 1;
wMin = 0.03; 
wLimit = [wMax, wMin];
goalRadius = 0.05;
distObstacle = 0.5;  
resolution = 1;
front_idx = 1;
min_idx = 60;  
max_idx = 300; 

%% 連結 MATLAB 與自走車
setenv("ROS_DOMAIN_ID", "30");
mtnode = ros2node("/matlab_turtlebot");
pause(6)

% 載入場域的佔據柵格地圖
load map_20230804.mat

%% 建立訂閱與發佈別名
odomSub = ros2subscriber(mtnode, "/odom",...
                "Reliability", "reliable",...
                "Durability", "volatile",...
                "History", "keeplast",...
                "Depth", 1);
pause(1)
scanSub = ros2subscriber(mtnode, "/scan",...
                "Reliability", "besteffort",...
                "Durability", "volatile",...
                "History", "keeplast",...
                "Depth", 1);
pause(1)

velPub = ros2publisher(mtnode, "/cmd_vel",...
                "Reliability", "reliable",...
                "Durability", "transientlocal",...
                "History", "keeplast",...
                "Depth", 1);
velData = ros2message("geometry_msgs/Twist");       
pause(1)

% 讀取里程計及LiDAR 感測器資料
receive(odomSub, 5);
receive(scanSub, 5);
odomData = odomSub.LatestMessage;
scanData = scanSub.LatestMessage;
pause(0.5);
rate = rateControl(10);

% 顯示已連線
disp("============ Connected to ROS Topic ============")

%% 定義每一個工作站的座標位置
sGoal = [     0,     0, -pi/2;
            0.6,     1,  pi/2];

% 定義起始點與目標點
start = sGoal(1, :);
goal  = sGoal(2, :);

% 由路徑規劃計算出的航點資訊
% path = [      0         0
%          0.0971    0.4895
%          0.5114    0.7549
%          0.6000    1.0000];

% path = [      0,         0;
%          0.4977,    0.0429;
%          0.5508,    0.5400;
%          0.6000,    1.0000];

path = [      0,         0;
        -0.0096,    0.4956;
         0.4523,    0.6764;
         0.6000,    1.0000];

% 畫出環境地圖並包含起始點與目標點
figure(1);
show(map); 
hold on;
plot(  start(1),   start(2), 'b*', 'MarkerSize', 3);
plot(path(2, 1), path(2, 2), 'b*', path(3, 1), path(3, 2), 'b*')
plot(   goal(1),    goal(2), 'r*', 'MarkerSize', 3);
legend("Start", "Waypoint 1", "Waypoint 2", "Goal")
title("Origin map")
hold off;

% 進行膨脹處理並畫出環境地圖
mapInflated = copy(map);
inflate(mapInflated, 0.05);
figure(2);
show(mapInflated);
title("Inflated map")

% 計算行駛路徑長度
pathLong = 0;
for i = 1:(length(path)-1)
  pathLong = pathLong + norm(path(i, :) - path(i+1, :));
end

% 顯示預計長度
fprintf("Expexted path length : %f (m)", pathLong)

%% 顯示里程計初始位置與地圖初始位置
iniPose = getRobotPose(odomSub, zeros(3, 1));
pose    = getRobotPose(odomSub, iniPose);
fprintf("\nIniPose from Odometry : \n     x : %f \n     y : %f \n theta : %f", iniPose)
fprintf("\nIniPose in Map : \n     x : %f \n     y : %f \n theta : %f", pose)

%% 確認前面的資訊是否ok?
while true
    prompt = '\nA = Go, Q = quit ';
    key = input(prompt, 's');
    if key == 'A' || key == 'a'
        disp("============================== Go ==============================")
        break;
    elseif key == 'Q' || key == 'q'
        disp("============================ Finish ============================")
        return;
    end
end
%% 旋轉自走車到預期的行駛方向
disp("=============== Rotate to the right orientation ================")
% 計算自走車初始位置及姿態
iniPose = getRobotPose(odomSub, zeros(3, 1));
pose    = getRobotPose(odomSub, iniPose);

slope = atan2((path(2,2) - pose(2)),(path(2,1) - pose(1)));
alpha = slope - pose(3);
  
while (abs(alpha) >= 0.1)
  % 計算角速度
  w = (wMax * sin(alpha));

  % 驅動自走車旋轉
  velData.linear.x = 0;
  velData.angular.z = w;
  send(velPub, velData);
  waitfor(rate);
  
  % 計算自走車當前位置及姿態
  pose = getRobotPose(odomSub, iniPose);
  slope = atan2((path(2,2) - pose(2)),...
                      (path(2,1) - pose(1)));
  % 計算角度差
  alpha = wrapToPi(slope - pose(3));
end

% 設定速度參數為 0     
velData.linear.x = 0;
velData.angular.z = 0;
send(velPub, velData);

%% 定義 Pure Pursuit 函式資料
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = vMax;
controller.MaxAngularVelocity = wMax;
controller.LookaheadDistance = 0.25;

% 畫出環境地圖並包含起始點、目標點與規劃的行駛路徑
pose = getRobotPose(odomSub, iniPose);
figure(3);
show(map);
hold on;
plot(  start(1),   start(2), 'b*', 'MarkerSize', 5)
plot(path(2, 1), path(2, 2), 'b*', path(3, 1), path(3, 2), 'b*')
plot(path(:, 1), path(:, 2),'k--',  'LineWidth', 2)
plot(   goal(1),    goal(2), 'r*', 'MarkerSize', 5)
title("Recoed")
hold off;

%% 驅動自走車前往目標點
disp("=========================== Tracking ===========================")
reset(rate);   
receive(odomSub, 5);
goalDist = 50 * goalRadius;
idx = 1;
% 判斷是否到達目標點
% while(goalDist >= goalRadius/2)
while(goalDist >= 0.03)
    % 進行 Pure Pursuit 運算
    [v, w, aheadPt] = controller(pose); 

    % 驅動自走車行進  
    velData.linear.x = v;
    velData.angular.z = w;
    send(velPub, velData);
    waitfor(rate);

    % 計算自走車當前位置及姿態
    pose = getRobotPose(odomSub, iniPose);
    fprintf("\nCurrent position in Map : \n  x : %f \n  y : %f", pose(1:2))
    rec.pose(idx, :) = pose;
    idx = idx +1;

    hold on
    % 於地圖中標注自走車當前位置	 
    plot(pose(1), pose(2), 'g*','MarkerSize',2);
    hold off
    
    % 計算自走車與目標點的距離
    goalDist = norm(pose(1:2) - path(end,:));
    % 接近目標點時進行減速的動作      
    if (goalDist <= 0.1)
        release(controller);
        
        % 計算新的線速度值與角速度值
        % 為與目標點距離的正弦函數的乘積
        controller.DesiredLinearVelocity = vMax * sin(goalDist);
        controller.MaxAngularVelocity    = wMax * sin(goalDist);
        
        % 設定新的前視距離
        controller.LookaheadDistance = 0.2;
    end
    fprintf("\nDistance to Goal : \n  %f", goalDist)
    fprintf("\n=====================================================\n")

end

% 自走車運動停止  
velData.linear.x = 0;
velData.angular.z = 0;
send(velPub, velData);

%% 到達目標點時進行方向調整 
disp("=================== Rotate to the Final Pose ===================")
pt2Goal(odomSub, velPub, velData, goal, wLimit, iniPose, goalRadius);
fprintf("\nReach Goal!")

% 到達目標點並停止 
velData.linear.x = 0;
velData.angular.z = 0;
send(velPub, velData);

release(controller);

%% 停止
velData.linear.x = 0;
velData.angular.z = 0;
send(velPub, velData);

disp("============================ Finish ============================")

%% Function
% 計算自走車當前位置及姿態
function Pose = getRobotPose(odomSub, iniPose)
  % 讀取里程計感測器資料
  odomData = odomSub.LatestMessage;
  pause(0.2);
  R = [cos(-iniPose(3)), -sin(-iniPose(3)),   0;
       sin(-iniPose(3)),  cos(-iniPose(3)),   0;
                      0,                 0,   1];

  T = [1,  0, -iniPose(1);
       0,  1, -iniPose(2);
       0,  0,          1];

  % 讀取自走車位置資料
  position = odomData.pose.pose.position;

  % 讀取自走車姿態資料	  
  orientation = odomData.pose.pose.orientation;
  odomQuat = [orientation.w, orientation.x, ...
              orientation.y, orientation.z];
  odomRotation = quat2eul(odomQuat);

  map_position = T * R * [position.x; position.y; 1];
  Pose = [map_position(1:2)', odomRotation(1) - iniPose(3)];
end

% 旋轉自走車朝向目標點方向
function pt2Goal(odomSub, velPub, velData, goal, wLimit, iniPose, goalRadius)
    wMax = wLimit(1);
    wMin = wLimit(2);
    rate = rateControl(5);
    % 到達目標點座標時的姿態方向
    orientation = goal(3);
  
    % 計算自走車當前位置及姿態
    pose = getRobotPose(odomSub, iniPose);
  
    % 計算角度差
    if orientation == pi
        alpha = orientation - sign(pose(3))*pose(3);
        alpha = sign(pose(3))*alpha;    
    else
        alpha = orientation - pose(3);
    end
  
    while (abs(alpha) >= goalRadius/2)
        % 計算角速度
        w = wMax * sin(alpha);
        if (abs(alpha) > 0.5*pi && abs(alpha) < 1.2*pi)
            w = sign(w)*wMax;
        end
  
        if abs(w) > wMax
            w = sign(w)*wMax;
        end
    
        if abs(w) < wMin
           w = sign(w)*wMin;
        end
    
        % 驅動自走車旋轉
        velData.linear.x = 0;
        velData.angular.z = w;
        send(velPub, velData);
        waitfor(rate);
  
        % 計算自走車當前位置及姿態
        pose = getRobotPose(odomSub, iniPose);              
  
        % 計算角度差
        if orientation == pi
            alpha = orientation - sign(pose(3))*pose(3);
            alpha = sign(pose(3))*alpha;        
        else
            alpha = orientation - pose(3);
            if abs(alpha) >= pi
                alpha = sign(pose(3))*(2*pi - abs(alpha));
            end
        end
        fprintf("\nCurrent orientation in Map : %f", alpha)
    end
   
    hold on
    plot(pose(1), pose(2), 'g*', 'MarkerSize', 2);
    hold off      
end