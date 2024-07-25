function plot_avp_differences(avp1, avp2)
    % 确保avp1和avp2在时间戳上是对齐的
    time = avp1(:,end); % 假设最后一列是时间戳
    
    % 计算姿态差异
    att_diff = avp1(:,1:3) - avp2(:,1:3);
    % 计算速度差异
    vel_diff = avp1(:,4:6) - avp2(:,4:6);
    % 计算位置差异
    pos_diff = avp1(:,7:9) - avp2(:,7:9);
    
    % 创建一个新的图形窗口
    figure;
    
    % 绘制姿态差异
    subplot(3,1,1);
    plot(time, att_diff, 'LineWidth', 1.5);
    title('Attitude Differences');
    xlabel('Time (s)');
    ylabel('Difference (Degrees)');
    legend('Pitch', 'Roll', 'Yaw');
    grid on;
    ytickformat('%.1e'); % 使用科学计数法表示y轴

    % 绘制速度差异
    subplot(3,1,2);
    plot(time, vel_diff, 'LineWidth', 1.5);
    title('Velocity Differences');
    xlabel('Time (s)');
    ylabel('Difference (m/s)');
    legend('Vx', 'Vy', 'Vz');
    grid on;
    ytickformat('%.1e'); % 使用科学计数法表示y轴

    % 绘制位置差异
    subplot(3,1,3);
    plot(time, pos_diff, 'LineWidth', 1.5);
    title('Position Differences');
    xlabel('Time (s)');
    ylabel('Difference (m)');
    legend('East', 'North', 'Up');
    grid on;
    ytickformat('%.1e'); % 使用科学计数法表示y轴
end
