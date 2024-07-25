function plot_trajectory_from_position(standard_avp, experiment_avp1)
    % 提取位置数据
    std_pos_east = standard_avp(:,7);
    std_pos_north = standard_avp(:,8);
    std_pos_up = standard_avp(:,9);
    
    exp1_pos_east = experiment_avp1(:,7);
    exp1_pos_north = experiment_avp1(:,8);
    exp1_pos_up = experiment_avp1(:,9);
    
    % 绘制轨迹图
    figure;
    grid on; box on; % 显示网格并开启边框
    hold on; % 保持图形以便绘制多个数据集
    
    % 标准数据轨迹
    plot3(std_pos_east, std_pos_north, std_pos_up, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Standard Data');
    % 实验数据1轨迹
    plot3(exp1_pos_east, exp1_pos_north, exp1_pos_up, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Experiment Data 1');
    
    % 标记起点和终点
    scatter3(std_pos_east(1), std_pos_north(1), std_pos_up(1), 'ko', 'filled', 'DisplayName', 'Start');
    scatter3(std_pos_east(end), std_pos_north(end), std_pos_up(end), 'kx', 'DisplayName', 'End');
    
    % 增强视觉效果
    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Up (m)');
    title('Combined 3D Trajectory Plot');
    view(3); % 设置视角为3D
    legend('show', 'Location', 'best');
    set(gca, 'FontSize', 10);
end
