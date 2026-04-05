function uwb_plot_report(matFile)
%UWB_PLOT_REPORT Plot MATLAB log output for report figures.
%
% Example:
%   uwb_plot_report('uwb_matlab_log.mat')

arguments
    matFile (1,1) string
end

loaded = load(matFile, 'logData');
logData = loaded.logData;

figure('Name', 'Trajectory Comparison', 'Color', 'w');
hold on; grid on; axis equal;
plotTrajectory(logData.odom, 'Odometry', [0.4 0.4 0.4]);
plotTrajectory(logData.eskf_pose, 'ESKF', [0.0 0.45 0.74]);
if isfield(logData, 'tf') && ~isempty(logData.tf)
    % TF is stored for auditing, but the robot pose comparison is plotted from
    % odometry and ESKF outputs, which are the report-critical quantities.
end
xlabel('X (m)');
ylabel('Y (m)');
title('Robot Trajectory');
legend('Location', 'best');

figure('Name', 'ESKF Covariance', 'Color', 'w');
if ~isempty(logData.eskf_cov.Time)
    plot(logData.eskf_cov.Time, logData.eskf_cov.Values, 'LineWidth', 1.25);
    grid on;
    xlabel('Time (s)');
    ylabel('Covariance / diagonal value');
    title('ESKF Covariance Diagonal Over Time');
else
    text(0.1, 0.5, 'No covariance samples were logged.', 'FontSize', 12);
    axis off;
end

figure('Name', 'Velocity Command Profile', 'Color', 'w');
if ~isempty(logData.cmd_vel.Time)
    yyaxis left;
    plot(logData.cmd_vel.Time, logData.cmd_vel.LinearX, 'LineWidth', 1.25);
    ylabel('Linear X (m/s)');
    yyaxis right;
    plot(logData.cmd_vel.Time, logData.cmd_vel.AngularZ, 'LineWidth', 1.25);
    ylabel('Angular Z (rad/s)');
    grid on;
    xlabel('Time (s)');
    title('Commanded Velocity Profile');
else
    text(0.1, 0.5, 'No cmd_vel samples were logged.', 'FontSize', 12);
    axis off;
end

figure('Name', 'UWB Ranges', 'Color', 'w');
if ~isempty(logData.uwb_ranges.Time)
    plot(logData.uwb_ranges.Time, logData.uwb_ranges.Values, 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('Range (m)');
    title('UWB Range Measurements');
else
    text(0.1, 0.5, 'No UWB ranges were logged.', 'FontSize', 12);
    axis off;
end

figure('Name', 'Mission Events', 'Color', 'w');
if ~isempty(logData.mission.Time)
    stem(logData.mission.Time, ones(size(logData.mission.Time)), 'filled');
    grid on;
    xlabel('Time (s)');
    ylabel('Event');
    title('Mission Configuration Updates');
    yticks([]);
else
    text(0.1, 0.5, 'No mission messages were logged.', 'FontSize', 12);
    axis off;
end

end

function plotTrajectory(series, labelText, colorValue)
if isempty(series.Time)
    return;
end
plot(series.X, series.Y, 'LineWidth', 1.6, 'DisplayName', labelText, 'Color', colorValue);
end
