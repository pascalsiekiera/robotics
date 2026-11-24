#!/usr/bin/env python3
"""Task 2a: figure, video and table of a real-robot waypoint run (rosbag + waypoint log).

Reads the rosbag offline (rosbag2_py, no ROS nodes) and the waypoint log written by waypoint_nav.py,
splits the run into legs (leg k = drive to waypoint k, from t_start to t_end of its attempt) and
creates
  figures/task2a_<name>.png  map, waypoints, estimated path and global plans per leg
  videos/task2a_<name>.mp4   animation (2x real time): plan, trail, robot, current laser scan
and prints a per-leg table.

Estimated pose = AMCL map->odom (computed from /amcl_pose and /odom, see map_odom_from_amcl; held
until the next update) composed with the odometry (/odom, interpolated). It equals /amcl_pose at every AMCL update (only every 0.25 m / 0.2 rad) and
is what Nav2 uses as the robot pose in between. Laser points are placed with the pose at the scan
time stamp (robot clock, same clock as /odom). Times in the bag and in the log are wall-clock
seconds of the laptop; /odom and /scan stamps come from the robot clock and are shifted by the
median receive delay of /odom.

Needs ROS 2 Humble (rosbag2_py) and ffmpeg. Example:
    python3 plot_lab_run.py ~/mp2_bags/dijkstra_dwb_test \\
        ../data/task2a/waypoints_dijkstra_dwb_test_20261124_121107.json --name dijkstra_dwb_run1
"""
import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter, FuncAnimation  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

from metrics import StaticMap, path_length  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MP2 = os.path.dirname(HERE)
REPO = os.path.dirname(MP2)
FIG_DIR = os.path.join(MP2, 'figures')
VIDEO_DIR = os.path.join(MP2, 'videos')
PLANNER_NAMES = {'dijkstra': 'Dijkstra/NavFn', 'rrt': 'RRT'}
DEFAULT_MAP = os.path.join(REPO, 'mp1', 'maps', 'map_2a_merged.yaml')

# one color per leg (tab colors as in the other figures; orange and green not adjacent)
LEG_COLORS = ['#1f77b4', '#ff7f0e', '#9467bd', '#2ca02c']
SCAN_COLOR = '#d62728'
LASER_X = -0.064          # base_scan in base_footprint [m] (no rotation)
ROBOT_RADIUS = 0.15       # footprint radius in the Nav2 costmaps [m]
XLIM, YLIM = (-2.2, 6.4), (-2.4, 6.3)
SPEEDUP, FPS = 2.0, 15
POSE_DT = 0.05            # sampling of the estimated path [s]
TOPICS = ['/odom', '/tf', '/amcl_pose', '/plan', '/scan']


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_bag(bag_dir):
    """Messages of TOPICS as numpy arrays; t = receive time (wall clock of the laptop) [s]."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    msg_types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in msg_types]))

    odom, map_odom, amcl, plans, scans = [], [], [], [], []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        msg = deserialize_message(raw, msg_types[topic])
        if topic == '/odom':
            p = msg.pose.pose
            odom.append((stamp_of(msg.header), t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == '/tf':
            for tf in msg.transforms:
                if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                    tr = tf.transform
                    map_odom.append((t, tr.translation.x, tr.translation.y, yaw_of(tr.rotation)))
        elif topic == '/amcl_pose':
            p = msg.pose.pose
            amcl.append((t, p.position.x, p.position.y, yaw_of(p.orientation), stamp_of(msg.header)))
        elif topic == '/plan':
            xy = np.array([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])
            plans.append({'t': t, 'xy': xy})
        elif topic == '/scan':
            scans.append({'stamp': stamp_of(msg.header), 'ranges': np.array(msg.ranges, dtype=float),
                          'angle_min': msg.angle_min, 'angle_inc': msg.angle_increment,
                          'range_min': msg.range_min, 'range_max': msg.range_max})
    return {'odom': np.array(odom), 'map_odom': np.array(map_odom), 'amcl': np.array(amcl),
            'plans': plans, 'scans': scans}


def map_odom_from_amcl(amcl, odom):
    """map->odom at every AMCL update: AMCL pose composed with the inverse odometry pose at the
    stamp of the AMCL pose (scan time, robot clock), valid from its receive time on.

    Used instead of map->odom from /tf: the recorder lost many /tf messages in the lab (gaps of up
    to 17 s in map->odom while /amcl_pose was complete), the held map->odom then put the robot up to
    2 m away from the AMCL pose."""
    order = np.argsort(odom[:, 0])
    o_t, o_xy, o_yaw = odom[order, 0], odom[order, 2:4], np.unwrap(odom[order, 4])
    out = []
    for t, x, y, yaw, stamp in amcl:
        ox, oy = np.interp(stamp, o_t, o_xy[:, 0]), np.interp(stamp, o_t, o_xy[:, 1])
        myaw = yaw - np.interp(stamp, o_t, o_yaw)
        c, s = np.cos(myaw), np.sin(myaw)
        out.append((t, x - (c * ox - s * oy), y - (s * ox + c * oy),
                    np.arctan2(np.sin(myaw), np.cos(myaw))))
    return np.array(out)


class PoseEstimate:
    """Robot pose in the map frame: AMCL map->odom (hold last) composed with odometry (interpolated)."""

    def __init__(self, odom, map_odom):
        # robot clock -> laptop clock: median delay between stamp and receive time of /odom
        self.clock_offset = float(np.median(odom[:, 1] - odom[:, 0]))
        order = np.argsort(odom[:, 0])
        self.odom_t = odom[order, 0] + self.clock_offset
        self.odom_xy = odom[order, 2:4]
        self.odom_yaw = np.unwrap(odom[order, 4])
        self.mo = map_odom

    def __call__(self, t):
        """(N, 3) array x, y, yaw for laptop times t."""
        t = np.atleast_1d(t)
        ox = np.interp(t, self.odom_t, self.odom_xy[:, 0])
        oy = np.interp(t, self.odom_t, self.odom_xy[:, 1])
        oyaw = np.interp(t, self.odom_t, self.odom_yaw)
        k = np.clip(np.searchsorted(self.mo[:, 0], t, side='right') - 1, 0, len(self.mo) - 1)
        mx, my, myaw = self.mo[k, 1], self.mo[k, 2], self.mo[k, 3]
        c, s = np.cos(myaw), np.sin(myaw)
        return np.column_stack([mx + c * ox - s * oy, my + s * ox + c * oy,
                                np.arctan2(np.sin(myaw + oyaw), np.cos(myaw + oyaw))])

    def corrections(self, threshold=0.05):
        """AMCL updates that moved the estimated robot position by more than threshold [m].

        (t, jump [m], heading change [deg]) per map->odom change: robot pose with the new minus
        with the previous map->odom at the same odometry pose."""
        out = []
        for i in range(1, len(self.mo)):
            t = self.mo[i, 0]
            ox = np.interp(t, self.odom_t, self.odom_xy[:, 0])
            oy = np.interp(t, self.odom_t, self.odom_xy[:, 1])
            pts = []
            for mx, my, myaw in (self.mo[i - 1, 1:], self.mo[i, 1:]):
                pts.append((mx + np.cos(myaw) * ox - np.sin(myaw) * oy,
                            my + np.sin(myaw) * ox + np.cos(myaw) * oy))
            jump = float(np.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]))
            dyaw = np.degrees(np.arctan2(np.sin(self.mo[i, 3] - self.mo[i - 1, 3]),
                                         np.cos(self.mo[i, 3] - self.mo[i - 1, 3])))
            if jump > threshold:
                out.append((t, jump, dyaw))
        return out


def scan_points(scan, pose):
    """Valid laser points of one scan in the map frame, (N, 2)."""
    r = scan['ranges']
    a = scan['angle_min'] + scan['angle_inc'] * np.arange(len(r))
    ok = np.isfinite(r) & (r >= scan['range_min']) & (r <= scan['range_max'])
    bx, by = LASER_X + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([x + c * bx - s * by, y + s * bx + c * by])


def split_legs(log, bag, est):
    """One dict per attempt: time window, plans, estimated path, lengths and final error."""
    wps = {w['name']: w for w in log['waypoints']}
    legs = []
    for k, a in enumerate(log['attempts']):
        t0, t1 = a['t_start'], a['t_end']
        # plans of this leg: published after the goal was sent (0.5 s margin) and before t_end
        plans = [p for p in bag['plans'] if t0 - 0.5 <= p['t'] <= t1]
        ts = np.arange(t0, t1, POSE_DT)
        path = est(np.append(ts, t1))
        amcl = bag['amcl'][(bag['amcl'][:, 0] >= t0) & (bag['amcl'][:, 0] <= t1)]
        wp = wps[a['waypoint']]
        legs.append({
            'index': k, 'waypoint': a['waypoint'], 'wp': (wp['x'], wp['y']), 'result': a['result'],
            't_start': t0, 't_end': t1, 'duration': t1 - t0, 'plans': plans, 'path': path,
            'planned_length': path_length(plans[0]['xy']) if plans else float('nan'),
            'driven_length': path_length(path[:, :2]),
            'amcl_length': path_length(amcl[:, 1:3]) if len(amcl) > 1 else float('nan'),
            'final_dist': float(np.hypot(path[-1, 0] - wp['x'], path[-1, 1] - wp['y'])),
            'color': LEG_COLORS[k % len(LEG_COLORS)],
        })
    return legs


def print_table(name, legs, bag, est):
    print(f'\n{name}: clock offset robot->laptop {est.clock_offset * 1000:.0f} ms, '
          f'{len(bag["plans"])} plans, {len(bag["amcl"])} AMCL updates')
    print(f'{"leg":<5}{"wp":<4}{"result":<11}{"time":>7}{"planned":>9}{"driven":>8}{"(amcl)":>8}'
          f'{"plans":>7}{"final":>7}')
    print(f'{"":<5}{"":<4}{"":<11}{"[s]":>7}{"[m]":>9}{"[m]":>8}{"[m]":>8}{"":>7}{"[m]":>7}')
    for leg in legs:
        print(f'{leg["index"] + 1:<5}{leg["waypoint"]:<4}{leg["result"]:<11}{leg["duration"]:>7.1f}'
              f'{leg["planned_length"]:>9.2f}{leg["driven_length"]:>8.2f}{leg["amcl_length"]:>8.2f}'
              f'{len(leg["plans"]):>7}{leg["final_dist"]:>7.2f}')
        for p in leg['plans'][1:]:
            print(f'      replan at +{p["t"] - leg["t_start"]:.1f} s, length {path_length(p["xy"]):.2f} m')
    print(f'{"sum":<20}{sum(lg["duration"] for lg in legs):>7.1f}'
          f'{sum(lg["planned_length"] for lg in legs):>9.2f}{sum(lg["driven_length"] for lg in legs):>8.2f}')
    jumps = est.corrections()
    print(f'AMCL corrections that moved the robot > 5 cm: {len(jumps)}'
          + ''.join(f'\n  t={t - legs[0]["t_start"]:+.1f} s: {d:.2f} m, {a:+.1f} deg' for t, d, a in jumps))


def draw_map(ax, static_map):
    shown = np.where(static_map.occupied, 0.0, np.where(static_map.unknown, 0.8, 1.0))
    ax.imshow(shown, cmap='gray', vmin=0, vmax=1, extent=static_map.extent(), interpolation='nearest')
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def draw_waypoints(ax, legs, size=15):
    for leg in legs:
        x, y = leg['wp']
        ax.plot(x, y, '*', color=leg['color'], ms=size, mec='k', mew=0.8, zorder=6)
        ax.annotate(leg['waypoint'], (x, y), xytext=(7, 7), textcoords='offset points', fontsize=11,
                    fontweight='bold', zorder=7,
                    bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.8))


def leg_label(leg):
    return (f'leg {leg["index"] + 1} to wp {leg["waypoint"]}: {leg["result"]}, {leg["duration"]:.1f} s, '
            f'planned {leg["planned_length"]:.2f} m, driven {leg["driven_length"]:.2f} m'
            + (f', {len(leg["plans"]) - 1} replan' + ('s' if len(leg['plans']) > 2 else '')
               if len(leg['plans']) > 1 else ''))


def plot_static(name, legs, static_map, out, planner='Dijkstra/NavFn'):
    fig, ax = plt.subplots(figsize=(9.5, 10.2))
    draw_map(ax, static_map)
    for leg in legs:
        for j, p in enumerate(leg['plans']):
            if j == 0:
                ax.plot(p['xy'][:, 0], p['xy'][:, 1], '--', color=leg['color'], lw=1.6, alpha=0.9)
            else:
                ax.plot(p['xy'][:, 0], p['xy'][:, 1], ':', color=leg['color'], lw=1.6, alpha=0.9)
                ax.plot(*p['xy'][0], 'x', color=leg['color'], ms=10, mew=2.5, zorder=5)
        ax.plot(leg['path'][:, 0], leg['path'][:, 1], '-', color=leg['color'], lw=2.2, label=leg_label(leg))
    start = legs[0]['path'][0]
    ax.plot(start[0], start[1], 'ko', ms=8, zorder=6)
    ax.annotate('start', start[:2], xytext=(-10, -16), textcoords='offset points', fontsize=9)
    draw_waypoints(ax, legs)
    handles, labels = ax.get_legend_handles_labels()
    style = [Line2D([], [], color='k', lw=2.2, label='estimated path (AMCL + odometry)'),
             Line2D([], [], color='k', ls='--', lw=1.6, label=f'global plan ({planner}), first of the leg'),
             Line2D([], [], color='k', ls=':', lw=1.6, marker='x', ms=8, mew=2,
                    label='replan (x = robot position)'),
             Line2D([], [], color='k', marker='o', ls='', label='start'),
             Line2D([], [], color='w', marker='*', mec='k', ms=13, ls='', label='waypoint')]
    ax.legend(handles=handles + style, labels=labels + [h.get_label() for h in style], fontsize=8,
              loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=1, frameon=False)
    ax.set_title(f'Task 2a, real robot: {name} ({planner} + DWB, AMCL), '
                 f'{sum(lg["duration"] for lg in legs):.0f} s', fontsize=11)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'saved {out}')


def render_video(name, legs, bag, est, static_map, out):
    t_begin, t_stop = legs[0]['t_start'], legs[-1]['t_end'] + 2.0
    frame_times = np.arange(t_begin, t_stop, SPEEDUP / FPS)
    scan_t = np.array([s['stamp'] for s in bag['scans']]) + est.clock_offset
    plan_t = np.array([p['t'] for p in bag['plans']])
    trail_t = np.arange(t_begin, t_stop, POSE_DT)
    trail = est(trail_t)
    leg_starts = np.array([lg['t_start'] for lg in legs])

    def leg_at(t):
        return int(np.clip(np.searchsorted(leg_starts, t, side='right') - 1, 0, len(legs) - 1))

    fig, ax = plt.subplots(figsize=(10.24, 9.6), dpi=125)    # 1280 x 1200 px
    draw_map(ax, static_map)
    draw_waypoints(ax, legs, size=17)
    plan_line, = ax.plot([], [], '--', lw=1.8, zorder=3)
    trails = [ax.plot([], [], '-', color=lg['color'], lw=2.2, zorder=4)[0] for lg in legs]
    scan_dots = ax.scatter([], [], s=5, color=SCAN_COLOR, zorder=5, linewidths=0)
    body = Circle((0, 0), ROBOT_RADIUS, fc='#ffffffcc', ec='k', lw=1.5, zorder=8)
    ax.add_patch(body)
    heading, = ax.plot([], [], 'k-', lw=2.2, zorder=9)
    info = ax.text(0.015, 0.985, '', transform=ax.transAxes, va='top', fontsize=12, family='monospace',
                   bbox=dict(boxstyle='round', fc='white', ec='0.6', alpha=0.9), zorder=10)
    ax.legend(handles=[Line2D([], [], color='0.3', ls='--', lw=1.8, label='current global plan'),
                       Line2D([], [], color='0.3', lw=2.2, label='estimated path (color = leg)'),
                       Line2D([], [], color=SCAN_COLOR, marker='.', ls='', label='laser scan'),
                       Line2D([], [], color='k', marker='o', mfc='w', ms=10, ls='', label='robot'),
                       Line2D([], [], color='w', marker='*', mec='k', ms=13, ls='', label='waypoint')],
              loc='lower right', fontsize=9, framealpha=0.9)
    ax.set_title(f'Task 2a, real robot: {name} ({SPEEDUP:.0f}x real time)', fontsize=12)
    fig.tight_layout()

    def update(i):
        t = frame_times[i]
        k = leg_at(t)
        n = np.searchsorted(trail_t, t, side='right')
        for j, line in enumerate(trails):
            lo, hi = legs[j]['t_start'], (legs[j + 1]['t_start'] if j + 1 < len(legs) else t_stop)
            sel = (trail_t[:n] >= lo) & (trail_t[:n] < hi)
            line.set_data(trail[:n][sel, 0], trail[:n][sel, 1])
        p = np.searchsorted(plan_t, t, side='right') - 1
        if p >= 0 and plan_t[p] >= legs[k]['t_start'] - 0.5:
            plan_line.set_data(bag['plans'][p]['xy'][:, 0], bag['plans'][p]['xy'][:, 1])
            plan_line.set_color(legs[k]['color'])
        else:
            plan_line.set_data([], [])
        s = np.searchsorted(scan_t, t, side='right') - 1
        if s >= 0:
            pose = est(scan_t[s])[0]
            scan_dots.set_offsets(scan_points(bag['scans'][s], pose))
        x, y, yaw = est(t)[0]
        body.center = (x, y)
        heading.set_data([x, x + 0.25 * np.cos(yaw)], [y, y + 0.25 * np.sin(yaw)])
        if t <= legs[-1]['t_end']:
            leg = legs[k]
            done = t > leg['t_end']
            status = f'{leg["result"]}' if done else f'{t - leg["t_start"]:.1f} s'
            label = f'leg to waypoint {leg["waypoint"]} ({status})'
        else:
            label = 'all waypoints done'
        info.set_text(f'{label}\nelapsed {t - t_begin:6.1f} s')
        return trails + [plan_line, scan_dots, body, heading, info]

    anim = FuncAnimation(fig, update, frames=len(frame_times), blit=False)
    anim.save(out, writer=FFMpegWriter(fps=FPS, codec='libx264', bitrate=3000,
                                       extra_args=['-pix_fmt', 'yuv420p']))
    plt.close(fig)
    print(f'saved {out} ({len(frame_times)} frames, {len(frame_times) / FPS:.0f} s)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('bag', help='rosbag2 directory (sqlite3)')
    parser.add_argument('log', help='waypoint log (JSON) of waypoint_nav.py')
    parser.add_argument('--name', required=True, help='output name: task2a_<name>.png/.mp4')
    parser.add_argument('--map', default=None,
                        help='map yaml (default: map of the log, else mp1/maps/map_2a_merged.yaml)')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()

    with open(args.log) as f:
        log = json.load(f)
    map_yaml = args.map or (os.path.join(REPO, log['map']) if 'map' in log else DEFAULT_MAP)
    static_map = StaticMap(map_yaml)
    bag = read_bag(args.bag)
    est = PoseEstimate(bag['odom'], map_odom_from_amcl(bag['amcl'], bag['odom']))
    legs = split_legs(log, bag, est)
    print_table(args.name, legs, bag, est)

    os.makedirs(FIG_DIR, exist_ok=True)
    # planner of the run: from the log of waypoint_nav.py --planners (one planner per round),
    # otherwise the default of lab.launch.py
    planners = {a.get('planner') for a in log['attempts']} - {None}
    planner = ' / '.join(PLANNER_NAMES[p] for p in sorted(planners)) or PLANNER_NAMES['dijkstra']
    plot_static(args.name, legs, static_map, os.path.join(FIG_DIR, f'task2a_{args.name}.png'),
                planner)
    if not args.no_video:
        os.makedirs(VIDEO_DIR, exist_ok=True)
        render_video(args.name, legs, bag, est, static_map,
                     os.path.join(VIDEO_DIR, f'task2a_{args.name}.mp4'))


if __name__ == '__main__':
    main()
