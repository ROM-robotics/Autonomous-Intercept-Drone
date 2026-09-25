import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

# === 字体设置（请根据你上一步跑通的方案调整） ===
plt.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Micro Hei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  

# ========== 0. 初始化路径与输出文件夹 ==========
try:
    base_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    base_dir = os.getcwd()

# 创建保存图片的文件夹
output_dir = os.path.join(base_dir, 'plots_output')
os.makedirs(output_dir, exist_ok=True)

csv_file = os.path.join(base_dir, 'vpng_intercept_stats_circle.csv')
if not os.path.exists(csv_file):
    print(f"找不到 CSV 文件：{csv_file}")
    sys.exit(1)

# ========== 1. 读取数据 ==========
raw = pd.read_csv(csv_file, comment='#')
raw = raw.dropna(subset=['time_s']).reset_index(drop=True)

t       = raw['time_s'].values - raw['time_s'].values[0]
self_x  = raw['self_x'].values;  self_y  = raw['self_y'].values;  self_z  = raw['self_z'].values
tgt_x   = raw['target_x'].values; tgt_y   = raw['target_y'].values; tgt_z   = raw['target_z'].values
dist    = raw['dist_m'].values
los_v   = raw['los_angle_v'].values; los_z   = raw['los_angle_z'].values
dv_v    = raw['d_v_angle_v'].values; dv_z    = raw['d_v_angle_z'].values
ex      = raw['ex'].values; ey      = raw['ey'].values
vx      = raw['vx'].values; vy      = raw['vy'].values; vz      = raw['vz'].values
V_total = np.sqrt(vx**2 + vy**2 + vz**2)
state   = raw['state'].values

idx_hit = np.argmin(dist)
min_dist = dist[idx_hit]
t_hit = t[idx_hit]

print('=== 拦截统计 ===')
print(f'飞行时长：{t[-1]:.2f} s')
print(f'最近接距离：{min_dist:.4f} m（t={t_hit:.2f} s）')
print(f'最终速度：{V_total[idx_hit]:.2f} m/s')

# ========== 2. 三维轨迹图 ==========
# 1. 提取绘图坐标 (NED -> Visual Up-is-Z)
plot_self_e = self_y; plot_self_n = self_x; plot_self_u = -self_z
plot_tgt_e  = tgt_y;  plot_tgt_n  = tgt_x;  plot_tgt_u  = -tgt_z

# 2. 计算空间极差，用于严格等比例缩放
r_e = np.ptp(np.concatenate([plot_self_e, plot_tgt_e]))
r_n = np.ptp(np.concatenate([plot_self_n, plot_tgt_n]))
r_u = np.ptp(np.concatenate([plot_self_u, plot_tgt_u]))

# 3. 动态调整画布宽度（防止 East 轴太长被压缩）
base_height = 8
width_ratio = r_e / r_n if r_n > 0.1 else 1.0
dynamic_width = max(8, min(16, base_height * width_ratio))

fig1 = plt.figure('三维拦截轨迹', figsize=(dynamic_width, base_height))
ax1 = fig1.add_subplot(111, projection='3d')

# 4. 强制执行 1:1:1 的物理空间比例
ax1.set_box_aspect((r_e, r_n, r_u))

ax1.plot(plot_self_e, plot_self_n, plot_self_u, 'b-', linewidth=1.5, label='拦截机轨迹')
ax1.plot(plot_tgt_e,  plot_tgt_n,  plot_tgt_u,  'r--', linewidth=1.5, label='目标机轨迹')

ax1.plot([plot_self_e[0]], [plot_self_n[0]], [plot_self_u[0]], 'bs', markersize=8, label='拦截机起点')
ax1.plot([plot_self_e[-1]], [plot_self_n[-1]], [plot_self_u[-1]], 'b^', markersize=8, color='cyan', label='拦截机终点')
ax1.plot([plot_tgt_e[0]], [plot_tgt_n[0]], [plot_tgt_u[0]], 'rs', markersize=8, label='目标机起点')
ax1.plot([plot_tgt_e[idx_hit]], [plot_tgt_n[idx_hit]], [plot_tgt_u[idx_hit]], 'k*', markersize=14, label='命中点')

ax1.legend(loc='best')
ax1.set_xlabel('East (m)'); ax1.set_ylabel('North (m)'); ax1.set_zlabel('Up (m)')
ax1.set_title(f'三维拦截轨迹 (严格等比例)  |  最近接距离 = {min_dist:.3f} m')

# 调整初始视角，更利于观察较长的 East 轴
ax1.view_init(elev=20, azim=-60)

# 保存图片，dpi=300保证高清，bbox_inches='tight'防止标签被裁剪
fig1.savefig(os.path.join(output_dir, '1_3D_Trajectory.png'), dpi=300, bbox_inches='tight')

# ========== 3. 相对距离随时间变化 ==========
fig2, ax2 = plt.figure('相对距离', figsize=(10, 4)), plt.gca()
ax2.plot(t, dist, 'b-', linewidth=1.8)
ax2.axvline(t_hit, color='r', linestyle='--', linewidth=1.5, label=f'命中 t={t_hit:.2f}s')
ax2.axhline(min_dist, color='k', linestyle=':', linewidth=1.2, label=f'最近 {min_dist:.3f}m')
ax2.set_xlabel('时间 (s)'); ax2.set_ylabel('相对距离 (m)')
ax2.set_title('拦截机与目标机相对距离')
ax2.grid(True); ax2.set_ylim([0, np.max(dist)*1.1]); ax2.legend()
fig2.savefig(os.path.join(output_dir, '2_Relative_Distance.png'), dpi=300, bbox_inches='tight')

# ========== 4. 速度曲线 ==========
fig3, (ax3a, ax3b) = plt.subplots(2, 1, num='速度曲线', figsize=(10, 8))
ax3a.plot(t, vx, 'r-', linewidth=1.2, label='vx(N)')
ax3a.plot(t, vy, 'g-', linewidth=1.2, label='vy(E)')
ax3a.plot(t, vz, 'b-', linewidth=1.2, label='vz(D)')
ax3a.plot(t, V_total, 'k-', linewidth=2, label='|V|')
ax3a.legend(loc='best'); ax3a.set_xlabel('时间 (s)'); ax3a.set_ylabel('速度 (m/s)')
ax3a.set_title('拦截机速度（NED）'); ax3a.grid(True)

ax3b.plot(t, V_total, 'k-', linewidth=1.8)
ax3b.axvline(t_hit, color='r', linestyle='--', linewidth=1.5, label='命中')
ax3b.set_xlabel('时间 (s)'); ax3b.set_ylabel('合速度 (m/s)')
ax3b.set_title('合速度 |V|'); ax3b.legend(); ax3b.grid(True)
fig3.tight_layout()
fig3.savefig(os.path.join(output_dir, '3_Velocity.png'), dpi=300, bbox_inches='tight')

# ========== 5. LOS 角度 & 期望速度角 ==========
fig4, axes4 = plt.subplots(2, 2, num='LOS与PNG角度', figsize=(12, 8))
ax4a, ax4b, ax4c, ax4d = axes4.flatten()

ax4a.plot(t, np.rad2deg(los_v), 'b-', linewidth=1.5, label='LOS仰角')
ax4a.plot(t, np.rad2deg(dv_v), 'r--', linewidth=1.5, label='PNG期望仰角')
ax4a.legend(loc='best'); ax4a.set_xlabel('时间 (s)'); ax4a.set_ylabel('角度 (°)')
ax4a.set_title('俯仰方向（Elevation）'); ax4a.grid(True)

ax4b.plot(t, np.rad2deg(los_z), 'b-', linewidth=1.5, label='LOS方位角')
ax4b.plot(t, np.rad2deg(dv_z), 'r--', linewidth=1.5, label='PNG期望方位角')
ax4b.legend(loc='best'); ax4b.set_xlabel('时间 (s)'); ax4b.set_ylabel('角度 (°)')
ax4b.set_title('方位方向（Azimuth）'); ax4b.grid(True)

dt = np.diff(t); dt[dt == 0] = 1e-6  
dlos_v_dt = np.diff(los_v) / dt; dlos_z_dt = np.diff(los_z) / dt
ax4c.plot(t[1:], np.rad2deg(dlos_v_dt), 'b-', linewidth=1.2, label='dLOS_v/dt')
ax4c.plot(t[1:], np.rad2deg(dlos_z_dt), 'r-', linewidth=1.2, label='dLOS_z/dt')
ax4c.legend(loc='best'); ax4c.set_xlabel('时间 (s)'); ax4c.set_ylabel('角速率 (°/s)')
ax4c.set_title('LOS 角速率（PNG 输入）'); ax4c.grid(True); ax4c.set_ylim([-60, 60])

d_mid = (dist[:-1] + dist[1:]) / 2 if len(dist) == len(dlos_v_dt) + 1 else dist[:-1]
nc_v = dlos_v_dt * d_mid; nc_z = dlos_z_dt * d_mid
ax4d.plot(t[1:], nc_v, 'b-', linewidth=1.2, label='法向速度（仰角）')
ax4d.plot(t[1:], nc_z, 'r-', linewidth=1.2, label='法向速度（方位）')
ax4d.legend(loc='best'); ax4d.set_xlabel('时间 (s)'); ax4d.set_ylabel('m/s')
ax4d.set_title('法向接近速度（理想PNG=0）'); ax4d.grid(True); ax4d.set_ylim([-5, 5])
fig4.tight_layout()
fig4.savefig(os.path.join(output_dir, '4_LOS_PNG_Angles.png'), dpi=300, bbox_inches='tight')

# ========== 6. 像素误差（视场跟踪质量）==========
fig5, (ax5a, ax5b) = plt.subplots(1, 2, num='像素误差', figsize=(12, 5))

ax5a.plot(t, ex, 'b-', linewidth=1.2, label='ex（水平）')
ax5a.plot(t, ey, 'r-', linewidth=1.2, label='ey（垂直）')
ax5a.axhline(0, color='k', linestyle=':', linewidth=1)
ax5a.axvline(t_hit, color='k', linestyle='--', linewidth=1.2, label='命中')
ax5a.legend(loc='best'); ax5a.set_xlabel('时间 (s)'); ax5a.set_ylabel('像素误差 (px)')
ax5a.set_title('视场像素误差（相对图像中心）'); ax5a.grid(True)

sc = ax5b.scatter(ex, ey, s=6, c=t, cmap='jet')
cb = plt.colorbar(sc, ax=ax5b); cb.set_label('时间 (s)')
ax5b.axhline(0, color='k', linestyle=':'); ax5b.axvline(0, color='k', linestyle=':')
ax5b.set_xlim([-960, 960]); ax5b.set_ylim([-540, 540])

rect = patches.Rectangle((-960, -540), 1920, 1080, linewidth=1.5, edgecolor='k', facecolor='none')
ax5b.add_patch(rect)
ax5b.set_xlabel('ex (px)'); ax5b.set_ylabel('ey (px)')
ax5b.set_title('检测中心在图像中的运动轨迹')
ax5b.set_aspect('equal'); ax5b.grid(True)
fig5.tight_layout()
fig5.savefig(os.path.join(output_dir, '5_Pixel_Error.png'), dpi=300, bbox_inches='tight')

# ========== 7. 二维俯视图（XY 平面，NED North-East）==========
fig6, ax6 = plt.figure('俯视轨迹', figsize=(8, 8)), plt.gca()

ax6.plot(self_y, self_x, 'b-', linewidth=2, label='拦截机')
ax6.plot(tgt_y, tgt_x, 'r--', linewidth=2, label='目标机')
ax6.plot(self_y[0], self_x[0], 'bs', markersize=10, label='拦截机起点')
ax6.plot(tgt_y[0], tgt_x[0], 'rs', markersize=10, label='目标机起点')
ax6.plot(self_y[idx_hit], self_x[idx_hit], 'k*', markersize=14, label=f'命中点({min_dist:.3f}m)')

theta = np.linspace(0, 2*np.pi, 100); hit_r = min_dist
ax6.plot(tgt_y[idx_hit] + hit_r*np.cos(theta), 
         tgt_x[idx_hit] + hit_r*np.sin(theta), 
         'k:', linewidth=1.2, label='最近接圆')

ax6.legend(loc='best'); ax6.set_xlabel('East (m)'); ax6.set_ylabel('North (m)')
ax6.set_title(f'俯视拦截轨迹  |  最近接 = {min_dist:.3f} m  @  t = {t_hit:.2f} s')
ax6.set_aspect('equal'); ax6.grid(True)
fig6.savefig(os.path.join(output_dir, '6_2D_Top_View.png'), dpi=300, bbox_inches='tight')

print(f'\n所有图表已生成完毕，并保存在本地文件夹：{output_dir}')

# 如果你是在无界面的纯终端环境跑代码，可以把下面这行注释掉
plt.show()