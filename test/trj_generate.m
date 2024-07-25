glvs;
ts = 0.1;  % 采样间隔
avp0 = [[0; 0; 0]; [0; 0; 0]; glv.pos0]; % 初始姿态、速度、位置
seg = trjsegment([], 'init', 0); % 初始化轨迹段

% 设计一个复杂的轨迹，包含多种机动和动作
totalTime = 0; % 记录总仿真时间
while totalTime < 2000 % 目标是生成总时长约2000秒的仿真轨迹
    seg = trjsegment(seg, 'accelerate', 30, [], 2);  % 加速30秒
    seg = trjsegment(seg, 'uniform', 60);  % 匀速飞行60秒
    seg = trjsegment(seg, 'deaccelerate', 30, [], 2); % 减速30秒
    seg = trjsegment(seg, 'turnleft', 45, 15); % 左转45秒，角速度15度/秒
    seg = trjsegment(seg, 'turnright', 45, 15); % 右转45秒，角速度15度/秒
    seg = trjsegment(seg, 'climb', 60, 10, [], 50); % 爬升60秒，10度/秒，匀速爬升50秒
    seg = trjsegment(seg, 'descent', 60, 10, [], 50); % 下降60秒，10度/秒，匀速下降50秒
    totalTime = totalTime + 30 + 60 + 30 + 45 + 45 + 60 + 60; % 更新总时间
end

% 调整最后一段轨迹，确保总时长约2000秒
if totalTime > 2000
    lastUniformTime = seg.wat(end, 1) - (totalTime - 2000);
    seg.wat(end, 1) = lastUniformTime;
end
% generate, save & plot
trj = trjsimu(avp0, seg.wat, ts, 1);
trjfile('similarTrajectory.mat', trj);
insplot(trj.avp);
imuplot(trj.imu);
