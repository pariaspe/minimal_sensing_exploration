"""
world_visualizer.py
"""
import json
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from viz_evaluation import LogData

Pose2D = tuple[float, float]


def parse_json(filename: str) -> tuple[str, dict[str, Pose2D]]:
    """Parse json file and return XML world name and drone positions"""
    with open(filename, 'r', encoding='utf-8') as f:
        environment = json.loads(f.read())
    world_name = environment['world_name']
    drones = {}
    for drone in environment['drones']:
        drones[drone['model_name']] = drone['xyz'][:-1]
    return world_name, drones


def parse_xml(filename: str) -> dict[str, Pose2D]:
    """Parse XML file and return pole positions"""
    world_tree = ET.parse(filename).getroot()
    models = {}
    for model in world_tree.iter('include'):
        if model.find('uri').text == 'model://pole':
            x, y, *_ = model.find('pose').text.split(' ')
            models[model.find('name').text] = (float(x), float(y))

    return models


class WorldFigure:
    """World matplotlib figure"""

    def __init__(self, name: str, side_length: float) -> None:
        fig: Figure = plt.figure(name)
        side_length: float = side_length

        self.main_plot = fig.add_subplot(1, 1, 1)
        self.main_plot.axis('equal')
        self.main_plot.set_xlabel('X-axis')
        self.main_plot.set_ylabel('Y-axis')
        self.main_plot.set_title(f"{name}")

        # Draw world boundaries
        self.main_plot.plot([side_length, side_length, -side_length, -side_length, side_length],
                            [side_length, -side_length, -side_length,
                                side_length, side_length],
                            'k-')

    def show_legend(self) -> None:
        """Show legend"""
        self.main_plot.legend()

    def draw_drones(self, drones: dict[str, Pose2D], color: str = 'rD') -> None:
        """Draw drones on plot"""
        drone_xs = [item[0] for item in drones.values()]
        drone_ys = [item[1] for item in drones.values()]
        labels = [item for item in drones.keys()]
        if len(labels) == 1:
            labels = labels[0]
        self.main_plot.plot(drone_xs, drone_ys, color, label=labels)

    def draw_obstacles(self, obstacles: dict[str, Pose2D], color: str = 'ko') -> None:
        """Draw obstacles on plot"""
        xpoints = [item[0] for item in obstacles.values()]
        ypoints = [item[1] for item in obstacles.values()]
        self.main_plot.plot(xpoints, ypoints, color)

    def draw_paths(self, paths: dict[str, Pose2D], color: str = 'b') -> None:
        """Draw paths on plot"""
        for path in paths.values():
            x, y = [], []
            for msg in path:
                x.append(msg[0])
                y.append(msg[1])
            self.main_plot.plot(x, y, color)

    def show(self) -> None:
        """Show plot"""
        plt.show()


def main(rosbag: str):
    """Main function"""
    poles = {}
    drones: dict[str, Pose2D] = {"cf0_X": (-3.0, 2.5)}

    fig = WorldFigure("World", 4.0)
    fig.draw_drones(drones)
    fig.draw_obstacles(poles)

    data = LogData.from_rosbag(rosbag)
    fig.draw_paths(data.poses, 'r')
    fig.show()


if __name__ == "__main__":
    main('rosbags/experiment_10a_edit')
