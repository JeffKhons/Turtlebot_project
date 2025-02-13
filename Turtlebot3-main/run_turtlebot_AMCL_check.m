clc; clear; close all
%% 連結 MATLAB 與自走車
setenv("ROS_DOMAIN_ID", "30");
mtnode = ros2node("/matlab_turtlebot");
pause(6)

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

%% Lidar Localization
typical_range = 3.5;
max_range     = 3.0;   
mapResolution = 100;
wMax = 0.1;

load map_20230804.mat

% 定義里程計模型與參數
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2]; 

% 定義感測器模型與參數
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.15 typical_range];
rangeFinderModel.MaxLikelihoodDistance = max_range;
rangeFinderModel.Map = map;
rangeFinderModel.SensorPose = [0 0 0];
rangeFinderModel.NumBeams = 360;  

% 定義 AMCL 函式資料
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

% 定義里程計模型及感測器模型  
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

% 定義超過多少位移量再進行 AMCL 運算
amcl.UpdateThresholds = [0.01, 0.01, 0.05];
amcl.ResamplingInterval = 1;

% 定義定位粒子的最大值與最小值
amcl.ParticleLimits = [500 3500];
% 不進行全域定位  
amcl.GlobalLocalization = false;

%% Initial pose
iniPose = getRobotPose(odomSub, zeros(3, 1))
pose = getRobotPose(odomSub, iniPose)

%% 定義自走車的起始位置
pose = getRobotPose(odomSub, iniPose);
amcl.InitialPose = pose;
amcl.InitialCovariance = eye(3) * 0.35;
%% 旋轉自走車到預期的行駛方向
% 計算自走車初始位置及姿態
slope = atan2((1 - pose(2)),(0 - pose(1)));
alpha = slope - pose(3);
  
while (abs(alpha) >= 0.1)
    pose = getRobotPose(odomSub, iniPose)

    % 讀取 LiDAR 感測器資料
    scanData = receive(scanSub, 5);
    pause(0.2);
    scans = lidarScan(double(scanData.ranges), ...
                        linspace(scanData.angle_min, scanData.angle_max, 360));    
    transScan = scans;

    % 進行 AMCL 自主定位運算  
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, transScan);

    % 計算角速度
    w = (wMax * sin(alpha));

    % 驅動自走車旋轉
    velData.linear.x = 0;
    velData.angular.z = w;
    send(velPub, velData);
    waitfor(rate);
  
    % 計算自走車當前位置及姿態
    pose = getRobotPose(odomSub, iniPose);
    slope = atan2((1 - pose(2)),(0 - pose(1)));
    % 計算角度差
    alpha = wrapToPi(slope - pose(3));
end

% 設定速度參數為 0     
velData.linear.x = 0;
velData.angular.z = 0;
send(velPub, velData);

%% Go straight
for i = 1 : 5
    i
    velData.linear.x  = 0.1;
    velData.angular.z = 0;
    send(velPub, velData);
    waitfor(rate);
    pose = getRobotPose(odomSub, iniPose)
    rec.pose(i, :) = pose;

    % 讀取 LiDAR 感測器資料
    scanData = receive(scanSub, 5);
    pause(0.2);
    scans = lidarScan(double(scanData.ranges), ...
      linspace(scanData.angle_min, scanData.angle_max, 360));    
    transScan = scans;

    % 進行 AMCL 自主定位運算  
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, transScan);
    estimatedPose
    rec.estimatedPose(i, :) = estimatedPose;
    % rec.odomPose(i, :) = getRobotPose(odomSub, zeros(3, 1));
    % rec.mapPose(i, :) = getRobotPose(odomSub, iniPose);
end

velData.linear.x  = 0;
velData.angular.z = 0;
send(velPub, velData);
waitfor(rate);
rec.odomPose(i+1, :) = getRobotPose(odomSub, zeros(3, 1));
rec.mapPose(i+1, :) = getRobotPose(odomSub, iniPose);

%% Stop  
velData.linear.x  = 0;
velData.angular.z = 0;
send(velPub, velData);
waitfor(rate);

%%
figure(1);
show(map); 
hold on;
plot(0, 0, 'b*', 'MarkerSize', 3);
plot(rec.pose(:, 1), rec.pose(:, 2), 'r', ...
    rec.estimatedPose(:, 1), rec.estimatedPose(:, 2), 'g')
legend("Odom", "AMCL")
hold off;

%% Function
% 計算自走車當前位置及姿態
function Pose = getRobotPose(odomSub, iniPose)
    % 讀取里程計感測器資料
    odomData = odomSub.LatestMessage;
    pause(0.2);
    R = [cos(iniPose(3)), -sin(iniPose(3)),   0;
         sin(iniPose(3)),  cos(iniPose(3)),   0;
                       0,                0,   1];

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
function pt2Goal(odomSub, goal, wLimit)
    wMax = wLimit(1);
    wMin = wLimit(2);
    rate = rateControl(5);
    % 到達目標點座標時的姿態方向
    orientation = goal(3);
    
    % 計算自走車當前位置及姿態
    pose = getRobotPose(odomSub);
    pose = pose + offset;
    
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
        pose = getRobotPose(odomSub);
        pose = pose + offset;              
    
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
    end
      
    hold on
    plot(pose(1), pose(2), 'g*', 'MarkerSize', 2);
    hold off      
end