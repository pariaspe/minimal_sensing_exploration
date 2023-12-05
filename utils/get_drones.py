#!/bin/python3

"""
get_drones.py
"""

import argparse


def get_drones_list(filename: str) -> list[str]:
    """Get drone names listed in swarm config file"""
    with open(filename, "r", encoding='utf-8') as f:
        lines = f.readlines()
        drones_ = []
        for line in lines:
            if line.strip().startswith("#"):
                continue
            if line.strip().startswith("uri"):
                continue

            drones_.append(line.strip().split(":")[0])
    return drones_


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", help="Path to swarm config file")
    parser.add_argument("--sep", help="Separator", default=":")
    args = parser.parse_args()
    drones = get_drones_list(args.config_file)
    print(args.sep.join(drones))
