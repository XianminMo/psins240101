addpath 'G:\Master Research\psins240101\test\error_calculate'
glvs
trj = trjfile('trj10ms.mat');
imu00 = trj.imu;
%% error setting
eb = 0.1;
db = 180;
web = 0.03;
wdb = 30;
imuerr = imuerrset(eb, db, web, wdb);
imu0 = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
avp00 = avpadderr(trj.avp0, davp0); 
trj = bhsimu(trj, 1, 10, 3, trj.ts);
%% pure inertial navigation
% 四元数更新阶数差异形成的不同INS算法
% [avp4, avp1, avp2] = inspure0(imu0, avp00, trj.bh, 0); % 分别对应4阶、1阶和2阶四元数更新近似
% 等效旋转矢量假设的差异形成的不同INS算法
avp_1 = inspure_single(imu0, avp00, trj.bh, 0); % single_previous sample, sample rate n=1
avp_2 = inspure_polynominal_sample2(imu0, avp00, trj.bh, 0); % polynominal compentation sample rate n=2
avp_3 = inspure_polynominal_sample3(imu0, avp00, trj.bh, 0); % polynominal compentation sample rate n=3
avp_4 = inspure_cone(imu0, avp00, trj.bh, 0); % coning compentation sample rate n=3
% plot_trajectory_from_position(trj.avp, avp2);
% plot_avp_differences(avp1, avp2);
%% compare and caculate error using ground truth
 [rmse1_attitude, rmse1_velocity, rmse1_position, aee1_attitude, aee1_velocity, aee1_position] = computeErrors(trj.avp, avp_1);
 [rmse2_attitude, rmse2_velocity, rmse2_position, aee2_attitude, aee2_velocity, aee2_position] = computeErrors(trj.avp, avp_2);
 [rmse3_attitude, rmse3_velocity, rmse3_position, aee3_attitude, aee3_velocity, aee3_position] = computeErrors(trj.avp, avp_3);
 [rmse4_attitude, rmse4_velocity, rmse4_position, aee4_attitude, aee4_velocity, aee4_position] = computeErrors(trj.avp, avp_4);
%% compare and calulate error
R = 10; % 实验数据添加噪声次数
numSimulations = 100;

imu1 = avp2imu(avp_1);
imu2 = avp2imu(avp_2);
imu3 = avp2imu(avp_3);
imu4 = avp2imu(avp_4); 

% 实验数据添加噪声 Z->{Z}
imu1_set = generateIMUDataWithErrors(imu1, R, eb, db, web, wdb);
imu2_set = generateIMUDataWithErrors(imu2, R, eb, db, web, wdb);
imu3_set = generateIMUDataWithErrors(imu3, R, eb, db, web, wdb);
imu4_set = generateIMUDataWithErrors(imu4, R, eb, db, web, wdb);

% 初始化存储每次模拟的RMSE和AEE结果
allRMSEs1 = zeros(numSimulations, 6);
allAEEs1 = zeros(numSimulations, 6);
allRMSEs2 = zeros(numSimulations, 6);
allAEEs2 = zeros(numSimulations, 6);
allRMSEs3 = zeros(numSimulations, 6);
allAEEs3 = zeros(numSimulations, 6);
allRMSEs4 = zeros(numSimulations, 6);
allAEEs4 = zeros(numSimulations, 6);

% Monte-Carlo method
for i = 1:numSimulations
    % 给“真实”IMU数据添加噪声
    imu00WithErr = generateIMUDataWithErrors(imu00, 1, eb, db, web, wdb); % R=1, 相当于添加一个噪声，不形成一组
     
    % 计算实验数据和“新的真实数据”之间的RMSE和AEE
    simulatedRMSE1 = calculateAverageRMSE(imu1_set, imu00WithErr, R);
    simulatedAEE1 = calculateAverageAEE(imu1_set, imu00WithErr, R);
    simulatedRMSE2 = calculateAverageRMSE(imu2_set, imu00WithErr, R);
    simulatedAEE2 = calculateAverageAEE(imu2_set, imu00WithErr, R); 
    simulatedRMSE3 = calculateAverageRMSE(imu3_set, imu00WithErr, R);
    simulatedAEE3 = calculateAverageAEE(imu3_set, imu00WithErr, R); 
    simulatedRMSE4 = calculateAverageRMSE(imu4_set, imu00WithErr, R);
    simulatedAEE4 = calculateAverageAEE(imu4_set, imu00WithErr, R); 
    
    % 存储每次模拟的结果
    allRMSEs1(i, :) = simulatedRMSE1;
    allAEEs1(i, :) = simulatedAEE1;
    allRMSEs2(i, :) = simulatedRMSE2;
    allAEEs2(i, :) = simulatedAEE2;
    allRMSEs3(i, :) = simulatedRMSE3;
    allAEEs3(i, :) = simulatedAEE3;
    allRMSEs4(i, :) = simulatedRMSE4;
    allAEEs4(i, :) = simulatedAEE4;
end

% 计算所有模拟的平均RMSE和AEE
averageRMSE1 = mean(allRMSEs1, 1);
averageAEE1 = mean(allAEEs1, 1);
averageRMSE2 = mean(allRMSEs2, 1);
averageAEE2 = mean(allAEEs2, 1);
averageRMSE3 = mean(allRMSEs3, 1);
averageAEE3 = mean(allAEEs3, 1);
averageRMSE4 = mean(allRMSEs4, 1);
averageAEE4 = mean(allAEEs4, 1);

% 计算陀螺仪和加速度计三轴平均
% 陀螺仪三轴的RMSE和AEE平均值
gyroRMSE1_Average = mean(averageRMSE1(1:3));
gyroAEE1_Average = mean(averageAEE1(1:3));
gyroRMSE2_Average = mean(averageRMSE2(1:3));
gyroAEE2_Average = mean(averageAEE2(1:3));
gyroRMSE3_Average = mean(averageRMSE3(1:3));
gyroAEE3_Average = mean(averageAEE3(1:3));
gyroRMSE4_Average = mean(averageRMSE4(1:3));
gyroAEE4_Average = mean(averageAEE4(1:3));

% 加速度计三轴的RMSE和AEE平均值
accelRMSE1_Average = mean(averageRMSE1(4:6));
accelAEE1_Average = mean(averageAEE1(4:6));
accelRMSE2_Average = mean(averageRMSE2(4:6));
accelAEE2_Average = mean(averageAEE2(4:6));
accelRMSE3_Average = mean(averageRMSE3(4:6));
accelAEE3_Average = mean(averageAEE3(4:6));
accelRMSE4_Average = mean(averageRMSE4(4:6));
accelAEE4_Average = mean(averageAEE4(4:6));

% 将性能评估值组合成一对
performanceEvaluation1 = [gyroRMSE1_Average, accelRMSE1_Average; gyroAEE1_Average, accelAEE1_Average];
performanceEvaluation2 = [gyroRMSE2_Average, accelRMSE2_Average; gyroAEE2_Average, accelAEE2_Average];
performanceEvaluation3 = [gyroRMSE3_Average, accelRMSE3_Average; gyroAEE3_Average, accelAEE3_Average];
performanceEvaluation4 = [gyroRMSE4_Average, accelRMSE4_Average; gyroAEE4_Average, accelAEE4_Average];

% 输出性能评估数组
disp('性能评估数组（第一行为RMSE，第二行为AEE，第一列为陀螺仪，第二列为加速度计）：');
disp(performanceEvaluation1);
disp(performanceEvaluation2);
disp(performanceEvaluation3);
disp(performanceEvaluation4);
