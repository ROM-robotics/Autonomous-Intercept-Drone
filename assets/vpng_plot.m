%% vpng_plot.m
%  uav_vision_png 拦截过程数据可视化
%  运行方式：在 MATLAB 中 cd 到此目录后执行 vpng_plot
%  数据文件：vpng_intercept_stats.csv（与本脚本同目录）

clear; clc; close all;

%% ========== 1. 读取数据 ==========
csv_file = fullfile(fileparts(mfilename('fullpath')), 'vpng_intercept_stats.csv');
if ~exist(csv_file, 'file')
    error('找不到 CSV 文件：%s', csv_file);
end

% readtable 不支持 CommentStyle，末尾 # 注释行会变成 NaN 行，需手动过滤
raw = readtable(csv_file);
raw = raw(~isnan(raw.time_s), :);   % 删除所有含 NaN 的注释残留行

% 列名（与 CSV 表头一致）
% time_s, self_x,self_y,self_z, target_x,target_y,target_z,
% dist_m, los_angle_v,los_angle_z, d_v_angle_v,d_v_angle_z,
% ex,ey, vx,vy,vz, detect_w, state

t       = raw.time_s - raw.time_s(1);   % 相对时间（s）
self_x  = raw.self_x;   self_y  = raw.self_y;   self_z  = raw.self_z;
tgt_x   = raw.target_x; tgt_y   = raw.target_y; tgt_z   = raw.target_z;
dist    = raw.dist_m;
los_v   = raw.los_angle_v;
los_z   = raw.los_angle_z;
dv_v    = raw.d_v_angle_v;
dv_z    = raw.d_v_angle_z;
ex      = raw.ex;
ey      = raw.ey;
vx      = raw.vx;  vy = raw.vy;  vz = raw.vz;
V_total = sqrt(vx.^2 + vy.^2 + vz.^2);
state   = raw.state;

% 命中时刻（dist 最小值）
[min_dist, idx_hit] = min(dist);
t_hit = t(idx_hit);

fprintf('=== 拦截统计 ===\n');
fprintf('飞行时长：%.2f s\n', t(end));
fprintf('最近接距离：%.4f m（t=%.2f s）\n', min_dist, t_hit);
fprintf('最终速度：%.2f m/s\n', V_total(idx_hit));

%% ========== 2. 三维轨迹图 ==========
figure('Name', '三维拦截轨迹', 'NumberTitle', 'off', 'Position', [100 100 800 600]);

% NED z 取负转为高度（Up 为正，方便视觉直观）
plot3(self_y,  self_x,  -self_z,  'b-',  'LineWidth', 1.5); hold on;
plot3(tgt_y,   tgt_x,   -tgt_z,   'r--', 'LineWidth', 1.5);

% 标注起点、命中点
plot3(self_y(1),   self_x(1),   -self_z(1),   'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot3(self_y(end), self_x(end), -self_z(end), 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'cyan');
plot3(tgt_y(1),    tgt_x(1),    -tgt_z(1),   'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(tgt_y(idx_hit), tgt_x(idx_hit), -tgt_z(idx_hit), ...
    'k*', 'MarkerSize', 14, 'LineWidth', 2);

legend('拦截机轨迹', '目标机轨迹', '拦截机起点', '拦截机终点', '目标机起点', '命中点', ...
    'Location', 'best');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
title(sprintf('三维拦截轨迹  |  最近接距离 = %.3f m', min_dist));
grid on; axis equal; view(35, 25);

%% ========== 3. 相对距离随时间变化 ==========
figure('Name', '相对距离', 'NumberTitle', 'off', 'Position', [100 100 900 400]);

plot(t, dist, 'b-', 'LineWidth', 1.8); hold on;
xline(t_hit, 'r--', sprintf('命中 t=%.2fs', t_hit), 'LineWidth', 1.5, 'FontSize', 10);
yline(min_dist, 'k:', sprintf('最近 %.3fm', min_dist), 'LineWidth', 1.2, 'FontSize', 10);
xlabel('时间 (s)'); ylabel('相对距离 (m)');
title('拦截机与目标机相对距离');
grid on; ylim([0, max(dist)*1.1]);

%% ========== 4. 速度曲线 ==========
figure('Name', '速度曲线', 'NumberTitle', 'off', 'Position', [100 550 900 500]);

subplot(2,1,1);
plot(t, vx, 'r-', 'LineWidth', 1.2); hold on;
plot(t, vy, 'g-', 'LineWidth', 1.2);
plot(t, vz, 'b-', 'LineWidth', 1.2);
plot(t, V_total, 'k-', 'LineWidth', 2);
legend('vx(N)', 'vy(E)', 'vz(D)', '|V|', 'Location', 'best');
xlabel('时间 (s)'); ylabel('速度 (m/s)');
title('拦截机速度（NED）');
grid on;

subplot(2,1,2);
plot(t, V_total, 'k-', 'LineWidth', 1.8); hold on;
xline(t_hit, 'r--', '命中', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('合速度 (m/s)');
title('合速度 |V|');
grid on;

%% ========== 5. LOS 角度 & 期望速度角 ==========
figure('Name', 'LOS与PNG角度', 'NumberTitle', 'off', 'Position', [950 100 900 600]);

subplot(2,2,1);
plot(t, rad2deg(los_v), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(dv_v),  'r--','LineWidth', 1.5);
legend('LOS仰角', 'PNG期望仰角', 'Location', 'best');
xlabel('时间 (s)'); ylabel('角度 (°)');
title('俯仰方向（Elevation）');
grid on;

subplot(2,2,2);
plot(t, rad2deg(los_z), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(dv_z),  'r--','LineWidth', 1.5);
legend('LOS方位角', 'PNG期望方位角', 'Location', 'best');
xlabel('时间 (s)'); ylabel('角度 (°)');
title('方位方向（Azimuth）');
grid on;

subplot(2,2,3);
% LOS 变化率（近似角速率）
dt = diff(t);
dt(dt == 0) = 1e-6;   % 防止除零（重复时间戳）
dlos_v_dt = diff(los_v) ./ dt;
dlos_z_dt = diff(los_z) ./ dt;
plot(t(2:end), rad2deg(dlos_v_dt), 'b-', 'LineWidth', 1.2); hold on;
plot(t(2:end), rad2deg(dlos_z_dt), 'r-', 'LineWidth', 1.2);
legend('dLOS_v/dt', 'dLOS_z/dt', 'Location', 'best');
xlabel('时间 (s)'); ylabel('角速率 (°/s)');
title('LOS 角速率（PNG 输入）');
grid on; ylim([-60 60]);

subplot(2,2,4);
% 脱靶量代理：LOS 角速率 × 距离 ≈ 法向接近速度
if length(dist) == length(dlos_v_dt)+1
    d_mid = (dist(1:end-1) + dist(2:end)) / 2;
else
    d_mid = dist(1:end-1);
end
nc_v = dlos_v_dt .* d_mid;   % 法向接近速度（俯仰）
nc_z = dlos_z_dt .* d_mid;   % 法向接近速度（方位）
plot(t(2:end), nc_v, 'b-', 'LineWidth', 1.2); hold on;
plot(t(2:end), nc_z, 'r-', 'LineWidth', 1.2);
legend('法向速度（仰角）', '法向速度（方位）', 'Location', 'best');
xlabel('时间 (s)'); ylabel('m/s');
title('法向接近速度（理想PNG=0）');
grid on; ylim([-5 5]);

%% ========== 6. 像素误差（视场跟踪质量）==========
figure('Name', '像素误差', 'NumberTitle', 'off', 'Position', [950 550 900 400]);

subplot(1,2,1);
plot(t, ex, 'b-', 'LineWidth', 1.2); hold on;
plot(t, ey, 'r-', 'LineWidth', 1.2);
yline(0, 'k:', 'LineWidth', 1);
xline(t_hit, 'k--', '命中', 'LineWidth', 1.2);
legend('ex（水平）', 'ey（垂直）', 'Location', 'best');
xlabel('时间 (s)'); ylabel('像素误差 (px)');
title('视场像素误差（相对图像中心）');
grid on;

subplot(1,2,2);
% 绘制检测中心在图像上的轨迹（热图/散点）
scatter(ex, ey, 6, t, 'filled');
colormap(jet); cb = colorbar; cb.Label.String = '时间 (s)';
xline(0, 'k:'); yline(0, 'k:');
xlim([-960 960]); ylim([-540 540]);
% 绘制图像边界
rectangle('Position', [-960 -540 1920 1080], 'EdgeColor', 'k', 'LineWidth', 1.5);
xlabel('ex (px)'); ylabel('ey (px)');
title('检测中心在图像中的运动轨迹');
axis equal; grid on;

%% ========== 7. 二维俯视图（XY 平面，NED North-East）==========
figure('Name', '俯视轨迹', 'NumberTitle', 'off', 'Position', [100 100 700 700]);

plot(self_y, self_x, 'b-', 'LineWidth', 2); hold on;
plot(tgt_y,  tgt_x,  'r--','LineWidth', 2);
plot(self_y(1),      self_x(1),      'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
plot(tgt_y(1),       tgt_x(1),       'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot(self_y(idx_hit),self_x(idx_hit),'k*', 'MarkerSize', 16, 'LineWidth', 2);

% 画命中半径圆
theta = linspace(0, 2*pi, 100);
hit_r = min_dist;   % 用实际最近距离画圆
plot(tgt_y(idx_hit) + hit_r*cos(theta), ...
     tgt_x(idx_hit) + hit_r*sin(theta), ...
     'k:', 'LineWidth', 1.2);

legend('拦截机', '目标机', '拦截机起点', '目标机起点', ...
    sprintf('命中点(%.3fm)', min_dist), '最近接圆', 'Location', 'best');
xlabel('East (m)'); ylabel('North (m)');
title(sprintf('俯视拦截轨迹  |  最近接 = %.3f m  @  t = %.2f s', min_dist, t_hit));
axis equal; grid on;

fprintf('\n所有图表已生成完毕。\n');
