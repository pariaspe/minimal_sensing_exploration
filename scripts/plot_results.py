"""
plot_results.py
"""

from pathlib import Path
from world_visualizer import WorldFigure, Pose2D, zenithal_view
from viz_evaluation import LogData, plot_area, plot_total_path, plot_path, plot_area_with_error
from overlap import total_overlap_ratio
import matplotlib.pyplot as plt
import numpy as np


def world_a() -> dict[str, Pose2D]:
    """Return world A"""
    poles: dict[str, Pose2D] = {
        "A1": (-1.4, 0.0), "#1": (1.8, 1.7)}
    return poles


def world_b() -> dict[str, Pose2D]:
    """Return world B"""
    poles: dict[str, Pose2D] = {
        "A1": (-1.4, 0.0), "A2": (3.3, 3.4), "#1": (1.8, 1.7), "#2": (2.1, -2.5)}
    return poles


def world_c() -> dict[str, Pose2D]:
    """Return world C"""
    poles: dict[str, Pose2D] = {
        "A1": (-1.4, 0.0), "A2": (3.3, 3.4), "A3": (-1.3, -3.9),
        "A4": (4.0, -1.9), "#1": (1.8, 1.7), "#2": (2.1, -2.4)}
    return poles


def world_d() -> dict[str, Pose2D]:
    """Return world D"""
    poles: dict[str, Pose2D] = {
        "A1": (-0.6, 3.5), "A2": (-1.5, 0.2), "A3": (-2, -3.2),
        "#1": (2.0, -2.3), "#2": (2.1, 1.8), "A4": (2.5, 1.5)}
    return poles


def plot_stats(bags: list[str]):
    """ Plot Stats: Area and Path Length """
    fig, fig2 = None, None
    for bag in bags:
        data = LogData.from_rosbag(Path(bag))

        print(data)
        fig = plot_area(data, fig)
        fig2 = plot_total_path(data, fig2)

        # plot_path(data)
        # print(data.stats(25.0))
    plt.show()


def experiments_one_drone():
    """ Experiment with one drone """
    bags = ["rosbags/experiment_1a",  # Obstacle configuration A
            "rosbags/experiment_2a",  # Obstacle configuration B
            "rosbags/experiment_3a",  # Obstacle configuration C
            "rosbags/experiment_4a",  # Obstacle configuration D, starting point X
            "rosbags/experiment_5a",  # Obstacle configuration D, starting point Y
            "rosbags/experiment_6a",  # Obstacle configuration D, starting point Z
            "rosbags/experiment_7a",  # Obstacle configuration D, starting point Z, repeat 2
            "rosbags/experiment_8a",  # Obstacle configuration D, starting point Z, repeat 3
            "rosbags/experiment_9a",  # Obstacle configuration D, starting point Z, repeat 4
            "rosbags/experiment_10a"  # Obstacle configuration D, starting point Z, repeat 5
            ]

    poles = []
    poles.append(world_a())
    poles.append(world_b())
    poles.append(world_c())
    poles.append(world_d())
    drones: dict[str, Pose2D] = {"cf0": (-3.0, 2.5)}

    for pole, bag in zip(poles, bags):
        fig = WorldFigure("One Drone Exploration", 4.0)
        fig.draw_drones(drones)
        fig.draw_obstacles(pole)

        data = LogData.from_rosbag(bag)
        fig.draw_paths(data.poses, 'r')

        fig.show_legend()

        # plot_stats(bag)
        fig.show()
    plot_stats(bags)


def experiment_one_drone_repetition_analisys():
    """ Experiment repetition analisys """
    bags = [
        "rosbags/experiment_6a_edit",
        "rosbags/experiment_7a_edit",
        "rosbags/experiment_8a_edit",
        "rosbags/experiment_9a_edit",
        "rosbags/experiment_10a_edit"
    ]
    colors = ['b', 'g', 'c', 'm', 'y', 'k', 'w']

    poles = world_d()
    drones: dict[str, Pose2D] = {"cf0": (-3.6, -2.1)}

    fig = WorldFigure("One Drone Exploration, different executions", 4.0)
    fig.draw_drones(drones)
    fig.draw_obstacles(poles)

    used_colors = colors[:len(bags)]
    for rosbag, c in zip(bags, used_colors):
        data = LogData.from_rosbag(rosbag)
        fig.draw_paths(data.poses, c)

    fig.show_legend()

    plot_stats(bags)
    fig.show()


def experiment_diff_starting_points():
    """ Experiment with different starting points """
    bags = ["rosbags/experiment_4a",
            "rosbags/experiment_5a",
            "rosbags/experiment_6a",]
    colors = ['r', 'b', 'g', 'c', 'm', 'y', 'k', 'w']

    poles = world_d()
    drones: list[dict[str, Pose2D]] = [
        {"cf0_X": (-3.0, 2.5)},
        {"cf0_Y": (1.75, -0.5)},
        {"cf0_Z": (-3.6, -2.1)}
    ]

    fig = WorldFigure(
        "One Drone Exploration, starting from different origins", 4.0)
    fig.draw_obstacles(poles)

    for drone, rosbag, c in zip(drones, bags, colors):
        fig.draw_drones(drone, c + 'D')
        data = LogData.from_rosbag(rosbag)
        fig.draw_paths(data.poses, c)

    fig.show_legend()

    plot_stats(bags)
    fig.show()


def experiments_two_drones():
    """ Experiment with two drones """
    bags = ["rosbags/experiment_11a",
            "rosbags/experiment_11b",
            "rosbags/experiment_12a",
            "rosbags/experiment_13a"]
    colors = ['r', 'b', 'g', 'c', 'm', 'y', 'k', 'w']

    poles = world_d()
    drones: list[dict[str, Pose2D]] = [
        {"cf0": (-2.9, 2.7)},
        {"cf1": (4.0, -0.7)}
    ]

    fig = WorldFigure("Two Drone Exploration", 4.0)
    fig.draw_obstacles(poles)

    data = LogData.from_rosbag(bags[0])

    for drone, pose, c in zip(drones, data.poses.values(), colors):
        fig.draw_drones(drone, c + 'D')
        pose_dict = {list(drone.keys())[0]: pose}
        fig.draw_paths(pose_dict, c)

    fig.show_legend()

    plot_stats(bags)
    fig.show()


def experiments_three_drones():
    """ Experiment with two drones """
    bags = ["rosbags/experiment_14c"]
    bags = ["rosbags/experiment_15"]
    bags = ["rosbags/experiment_16"]
    # bags = ["rosbags/exploration_20231212_151319"]
    # bags = ["rosbags/exploration_20231212_150717"]

    colors = ['r', 'b', 'g', 'c', 'm', 'y', 'k', 'w']

    poles = world_d()
    drones: list[dict[str, Pose2D]] = [
        {"cf0": (0.65, -0.75)},
        {"cf1": (0.7, 0.64)},
        {"cf2": (2.0, 0.1)}
    ]

    fig = WorldFigure("Three Drone Exploration", 4.0)
    fig.draw_obstacles(poles)

    data = LogData.from_rosbag(bags[0])

    for drone, pose, c in zip(drones, data.poses.values(), colors):
        fig.draw_drones(drone, c + 'D')
        pose_dict = {list(drone.keys())[0]: pose}
        fig.draw_paths(pose_dict, c)

    fig.show_legend()

    plot_stats(bags)
    fig.show()


def plot_area_mean(experiments: dict[str, LogData], longest_ts: str, label: str, fig=None):
    """ Plot area figure with error """
    m_data = None
    for data in experiments.values():
        tail = [data.area_pct[-1]] * \
            (len(experiments[longest_ts].area_pct) - len(data.area_pct))
        vaux = np.array(data.area_pct + tail, dtype=np.float64)

        m_data = vaux if m_data is None else np.vstack([m_data, vaux])

    ts = np.array(experiments[longest_ts].timestamps, dtype=np.float64)
    mean = np.mean(m_data, axis=0, dtype=np.float64)
    std = np.std(m_data, axis=0, dtype=np.float64)
    return plot_area_with_error(ts, mean, std, label, fig)


def print_stats(label: str, experiments: dict[str, LogData]):
    """ Print stats """
    ts = []
    area_pct = []
    path_length = []
    overlap_ratio = []
    for k, v in experiments.items():
        ts.append(v.timestamps[-1])
        area_pct.append(v.area_pct[-1])
        path_length.append(v.total_path[-1])
        overlap_ratio.append(total_overlap_ratio(k))

    ts_mean = np.mean(ts)
    ts_std = np.std(ts)
    area_pct_mean = np.mean(area_pct)
    area_pct_std = np.std(area_pct)
    path_length_mean = np.mean(path_length)
    path_length_std = np.std(path_length)
    overlap_ratio_mean = np.mean(overlap_ratio)
    overlap_ratio_std = np.std(overlap_ratio)
    print(f"\nStats for {label}")
    print(f"Timestamps mean: {ts_mean:.2f} std: {ts_std:.2f}")
    print(f"Area mean: {area_pct_mean:.2f} std: {area_pct_std:.2f}")
    print(
        f"Path length mean: {path_length_mean:.2f} std: {path_length_std:.2f}")
    print(
        f"Overlap ratio mean: {overlap_ratio_mean:.2f} std: {overlap_ratio_std:.2f}\n")


def experiment_number_drones_analisys():
    """ Experiment number of drones analisys """
    one_drone_bags = ["rosbags/experiment_6a",
                      "rosbags/experiment_7a",
                      "rosbags/experiment_8a",
                      "rosbags/experiment_9a",
                      "rosbags/experiment_10a"
                      ]
    two_drones_bags = ["rosbags/experiment_11a",
                       #    "rosbags/experiment_11b",  # noisy
                       "rosbags/experiment_12a",
                       "rosbags/experiment_13a"]
    three_drones_bags = ["rosbags/experiment_14c",
                         "rosbags/experiment_15",
                         "rosbags/experiment_15a",
                         "rosbags/experiment_15b",
                         "rosbags/experiment_15c",
                         "rosbags/experiment_15d",
                         "rosbags/experiment_15e",
                         "rosbags/experiment_16"]

    fig = None
    bags_dict = {'1 drone': one_drone_bags,
                 '2 drones': two_drones_bags,
                 '3 drones': three_drones_bags}

    for key, value in bags_dict.items():
        bags = value
        experiments: dict[str, LogData] = {}
        longest_ts = bags[0]

        for rosbag in bags:
            data = LogData.from_rosbag(rosbag)
            experiments[rosbag] = data
            longest_ts = rosbag if len(data.timestamps) > len(
                experiments[longest_ts].timestamps) else longest_ts

        print_stats(key, experiments)
        # plot_stats(bags)

        fig = plot_area_mean(experiments, longest_ts, key, fig)
    plt.show()


def one_drone_zenithal_view():
    """ One drone zenithal view """
    zenithal_view('rosbags/experiment_8a', world_d())
    print_stats('1 drone', {'rosbags/experiment_8a': LogData.from_rosbag('rosbags/experiment_8a')})


def two_drone_zenithal_view():
    """ Two drone zenithal view """
    zenithal_view("rosbags/experiment_13a", world_d())
    print_stats(
        '2 drones', {'rosbags/experiment_13a': LogData.from_rosbag('rosbags/experiment_13a')})


def three_drone_zenithal_view():
    """ Three drone zenithal view """
    zenithal_view("rosbags/experiment_16", world_d())
    print_stats(
        '3 drones', {'rosbags/experiment_16': LogData.from_rosbag('rosbags/experiment_16')})


if __name__ == "__main__":
    import os
    if not os.path.exists("rosbags"):
        print("rosbags directory not found.")
        exit()
    # experiments_one_drone()
    # experiment_one_drone_repetition_analisys()
    # experiment_diff_starting_points()
    # experiments_two_drones()
    # experiments_three_drones()
    # experiment_number_drones_analisys()
    one_drone_zenithal_view()
    two_drone_zenithal_view()
    three_drone_zenithal_view()
