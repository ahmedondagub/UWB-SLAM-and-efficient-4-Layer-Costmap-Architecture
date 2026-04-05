function logData = uwb_matlab_logger(options)
%UWB_MATLAB_LOGGER Log ROS 2 topics to a MAT file for offline analysis.
%
% This is a MATLAB fallback when the Gazebo simulation or ROS bag pipeline is
% not available. It subscribes to the active ROS 2 graph, samples the topics
% used by the report, and stores them in a struct that can be saved to disk.
%
% Example:
%   logData = uwb_matlab_logger(struct('DurationSec', 120, ...
%       'OutputFile', 'uwb_run.mat', 'UseSimTime', true));
%
% Required topics:
%   /tf
%   /tf_static
%   /odom
%   /eskf/pose
%   /eskf/covariance_diagonal
%   /uwb/ranges
%   /cmd_vel
%   /mission/config

arguments
    options.DurationSec (1,1) double {mustBePositive} = 120
    options.OutputFile (1,1) string = "uwb_matlab_log.mat"
    options.PollPeriodSec (1,1) double {mustBePositive} = 0.05
    options.UseSimTime (1,1) logical = false
end

node = ros2node("/uwb_matlab_logger");
subscribers = struct();
subscribers.tf = ros2subscriber(node, "/tf", "tf2_msgs/TFMessage");
subscribers.tf_static = ros2subscriber(node, "/tf_static", "tf2_msgs/TFMessage");
subscribers.odom = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
subscribers.eskf_pose = ros2subscriber(node, "/eskf/pose", "geometry_msgs/PoseStamped");
subscribers.eskf_cov = ros2subscriber(node, "/eskf/covariance_diagonal", "std_msgs/Float32MultiArray");
subscribers.uwb_ranges = ros2subscriber(node, "/uwb/ranges", "std_msgs/Float32MultiArray");
subscribers.cmd_vel = ros2subscriber(node, "/cmd_vel", "geometry_msgs/Twist");
subscribers.mission = ros2subscriber(node, "/mission/config", "std_msgs/String");

logData = struct();
logData.meta.startedAt = char(datetime("now"));
logData.meta.durationSec = options.DurationSec;
logData.meta.useSimTime = options.UseSimTime;
logData.meta.outputFile = char(options.OutputFile);
logData.tf = {};
logData.tf_static = {};
logData.odom = struct('Time', [], 'X', [], 'Y', [], 'Yaw', [], 'LinearX', [], 'AngularZ', []);
logData.eskf_pose = struct('Time', [], 'X', [], 'Y', [], 'Yaw', []);
logData.eskf_cov = struct('Time', [], 'Values', []);
logData.uwb_ranges = struct('Time', [], 'Values', []);
logData.cmd_vel = struct('Time', [], 'LinearX', [], 'AngularZ', []);
logData.mission = struct('Time', [], 'Raw', strings(0,1));

startTime = tic;
nextPoll = 0.0;
while toc(startTime) < options.DurationSec
    pause(options.PollPeriodSec);
    elapsedSec = toc(startTime);
    if elapsedSec < nextPoll
        continue;
    end
    nextPoll = elapsedSec + options.PollPeriodSec;

    msg = take(subscribers.odom, 1);
    if ~isempty(msg)
        pose = msg.pose.pose;
        twist = msg.twist.twist;
        logData.odom.Time(end+1,1) = elapsedSec;
        logData.odom.X(end+1,1) = pose.position.x;
        logData.odom.Y(end+1,1) = pose.position.y;
        logData.odom.Yaw(end+1,1) = quaternionToYaw(pose.orientation);
        logData.odom.LinearX(end+1,1) = twist.linear.x;
        logData.odom.AngularZ(end+1,1) = twist.angular.z;
    end

    msg = take(subscribers.eskf_pose, 1);
    if ~isempty(msg)
        pose = msg.pose;
        logData.eskf_pose.Time(end+1,1) = elapsedSec;
        logData.eskf_pose.X(end+1,1) = pose.position.x;
        logData.eskf_pose.Y(end+1,1) = pose.position.y;
        logData.eskf_pose.Yaw(end+1,1) = quaternionToYaw(pose.orientation);
    end

    msg = take(subscribers.eskf_cov, 1);
    if ~isempty(msg)
        logData.eskf_cov.Time(end+1,1) = elapsedSec;
        logData.eskf_cov.Values(end+1,1:numel(msg.data)) = double(msg.data(:).'); %#ok<AGROW>
    end

    msg = take(subscribers.uwb_ranges, 1);
    if ~isempty(msg)
        logData.uwb_ranges.Time(end+1,1) = elapsedSec;
        logData.uwb_ranges.Values(end+1,1:numel(msg.data)) = double(msg.data(:).'); %#ok<AGROW>
    end

    msg = take(subscribers.cmd_vel, 1);
    if ~isempty(msg)
        logData.cmd_vel.Time(end+1,1) = elapsedSec;
        logData.cmd_vel.LinearX(end+1,1) = msg.linear.x;
        logData.cmd_vel.AngularZ(end+1,1) = msg.angular.z;
    end

    msg = take(subscribers.mission, 1);
    if ~isempty(msg)
        logData.mission.Time(end+1,1) = elapsedSec;
        logData.mission.Raw(end+1,1) = string(msg.data);
    end

    tfMsg = take(subscribers.tf, 1);
    if ~isempty(tfMsg)
        logData.tf{end+1,1} = tfMsg; %#ok<AGROW>
    end

    tfStaticMsg = take(subscribers.tf_static, 1);
    if ~isempty(tfStaticMsg)
        logData.tf_static{end+1,1} = tfStaticMsg; %#ok<AGROW>
    end
end

save(options.OutputFile, 'logData');
fprintf('Saved MATLAB log to %s\n', options.OutputFile);
end

function yaw = quaternionToYaw(q)
%QUATERNIONTOYAW Convert a quaternion message to yaw angle.

siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
yaw = atan2(siny_cosp, cosy_cosp);
end
