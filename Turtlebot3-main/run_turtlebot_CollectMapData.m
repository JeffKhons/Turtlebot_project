clc; clear; close all
%% 設定共用變數 
vMax = 0.05; 
wMax = 0.50;
typical_range = 3.5;
max_range     = 3.0;   % meters   
mapResolution = 20;

%% 連結 MATLAB 與自走車
setenv("ROS_DOMAIN_ID", "30");
mtnode = ros2node("/matlab_turtlebot");
pause(10);

%% 建立訂閱與發佈別名
odomSub = ros2subscriber(mtnode, "/odom",...
                "Reliability", "reliable",...
                "Durability", "volatile",...
                "History", "keeplast",...
                "Depth", 1);
pause(1);
scanSub = ros2subscriber(mtnode, "/scan",...
                "Reliability", "besteffort",...
                "Durability", "volatile",...
                "History", "keeplast",...
                "Depth", 1);
pause(1);

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
rate = rateControl(2);

%% Initial pose
iniPose = getRobotPose(odomSub, zeros(3, 1))
Pose = getRobotPose(odomSub, iniPose)
%%
idx = 1;
%% 遙控自走車移動
% 計算自走車當前位置及姿態
pose = getRobotPose(odomSub, iniPose);
figure(1);
hold on;
plot(pose(1), pose(2), 'r*','MarkerSize',2);
hold off;

lin_vel = vMax;
ang_vel = wMax;
x_lin = 0.0;
z_ang = 0.0;

while lin_vel < 1
    % 讀取 LiDAR 感測器資料      
    scanData = scanSub.LatestMessage;
    pause(0.1);
    scan = lidarScan(double(scanData.ranges), ...
                linspace(scanData.angle_min, scanData.angle_max, 360));  
            
    % 依照順序紀錄資料      
    gScans{idx} = scan;
    idx = idx +1;
            
    prompt = 'W = front, A = left, X = back, D = right, S = stop, P = quit ';
    key = input(prompt, 's');
    if key == 'w' || key == 'W'
        x_lin = lin_vel;
        z_ang = 0;
    elseif key == 'q' || key == 'Q'
        x_lin = lin_vel;
        z_ang = ang_vel;        
    elseif key == 'e' || key == 'E'
        x_lin = lin_vel;
        z_ang = -ang_vel;     
    elseif key == 'a' || key == 'A'
        x_lin = 0;
        z_ang = ang_vel;       
    elseif key == 'x' || key == 'X'
        x_lin = -lin_vel;
        z_ang = 0;
    elseif key == 'd' || key == 'D'
        x_lin = 0;
        z_ang = -ang_vel;
    elseif key == 's' || key == 'S'
        x_lin = 0;
        z_ang = 0;
    elseif key == 'p' || key == 'P'
        break;      
    else
        velData.linear.x = 0;
        velData.angular.z = 0;   
        send(velPub, velData);
    end        

    % 驅動自走車行進  
    velData.linear.x = x_lin;
    velData.angular.z = z_ang;   
    send(velPub, velData);
    waitfor(rate);  
    waitfor(rate);  

    % 自走車運動停止  
    velData.linear.x = 0;
    velData.angular.z = 0;   
    send(velPub, velData);

    % 計算自走車當前位置及姿態
    % pose = getRobotPose(odomSub, iniPose);
    % figure(1);
    % hold on;
    % plot(pose(1), pose(2), 'r*','MarkerSize',2);
    % hold off;
end

%% 設定自走車速度為 0   
velData.linear.x = 0;
velData.angular.z = 0;
send(velPub, velData);

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