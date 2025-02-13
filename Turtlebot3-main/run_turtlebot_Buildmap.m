clc; clear; close all
%% 讀取Lidar資料
load gScans_20230804.mat
%% 設定共用變數 
vMax = 0.05; 
wMax = 0.50;

typical_range = 3.5;
max_range     = 3.0;   % meters   
mapResolution = 20;

idx = length(gScans);
%% 定義 LidarSLAM 函式資料
slamAlg = lidarSLAM(mapResolution, typical_range);
slamAlg.LoopClosureThreshold = 350;
slamAlg.LoopClosureSearchRadius= max_range;
slamAlg.MovementThreshold = [0, 0];

%% 進行環境地圖運算
tic
for i = 1:(idx-1)
    i
    scan = gScans{i}; 
    addScan(slamAlg, scan);
    pause(0.2);

    if rem(i, 5) == 0
       figure(3); show(slamAlg);
    end
 end
toc
%% 畫出環境地圖運算結果
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, typical_range);
figure(5)
show(map);
title('Occupancy Map');
hold on;
show(slamAlg, 'Poses', 'off');
hold off

%% 儲存環境地圖資料
save VSC_map_20230804_7 'map' 'slamAlg'