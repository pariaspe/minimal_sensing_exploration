"""
overlap.py
"""

import numpy as np
import cv2
from itertools import combinations
from nav_msgs.msg import OccupancyGrid
from grid_map_msgs.msg import GridMap
from bag_reader import read_rosbag, deserialize_rosbag


def to_image(grid_map: GridMap, layer: str) -> np.array:
    """From GridMap to image (np.array). Get the layer specified by layer"""
    layer_idx = grid_map.layers.index(layer)
    data = np.array(grid_map.data[layer_idx].data, dtype=np.float64)

    data = data.reshape((int(grid_map.info.length_x/grid_map.info.resolution),
                        int(grid_map.info.length_y/grid_map.info.resolution)))
    data = np.nan_to_num(data, nan=-1.0).astype(np.int8)
    data[data == 0] = 127
    data[data == 100] = 0

    data_aux = cv2.convertScaleAbs(data, alpha=100).astype(np.uint8)
    data_aux[data_aux == 100] = 128
    return data_aux


def overlap(layer: np.array, other: np.array) -> np.array:
    """Get the overlapping between two layers. Return a binary array with the overlapping"""
    aux = layer.copy()
    aux[aux != 128] = 255
    aux[aux == 128] = 0

    aux2 = other.copy()
    aux2[aux2 != 128] = 255
    aux2[aux2 == 128] = 0
    return aux & aux2


def overlap_ratio(overlapping: np.array) -> float:
    """Get the overlapping ratio between two layers.
    Input binary image with overlapping. Return the ratio in percentage"""
    unique, counts = np.unique(overlapping, return_counts=True)
    counter = dict(zip(unique, counts))
    try:
        intersection = counter[255]
    except KeyError:
        intersection = 0
    return 100*intersection/counts.sum()


def total_overlap_ratio(rosbag: str) -> float:
    """Print overlapping ratio"""
    info = read_rosbag(rosbag)
    info = deserialize_rosbag(info, {"/map_server/map_filtered": OccupancyGrid,
                                     "/map_server/grid_map": GridMap})

    last_grid_map: GridMap = info["/map_server/grid_map"][-1]
    last_occ_grid: OccupancyGrid = info["/map_server/map_filtered"][-1]

    total = np.zeros((last_occ_grid.info.height, last_occ_grid.info.width))
    for comb in combinations(last_grid_map.layers, 2):
        layer0 = to_image(last_grid_map, comb[0])
        layer1 = to_image(last_grid_map, comb[1])
        total += overlap(layer0, layer1)
    return overlap_ratio(total)


if __name__ == "__main__":
    print(total_overlap_ratio("rosbags/experiment_16"), "%")
