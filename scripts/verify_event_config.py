#!/usr/bin/env python3
"""Validates TP-42 C without ROS or a hardware connection; --ready rejects placeholders."""
import argparse
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/hackathon_bringup'))
from hackathon_bringup.event_config import validate_event


def main():
    config = ROOT/'ros2_ws/src/hackathon_bringup/config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', type=Path, default=config/'event.yaml')
    parser.add_argument('--hardware', type=Path, default=config/'event_hardware.yaml')
    parser.add_argument('--mission', type=Path, default=config/'missions/event.yaml')
    parser.add_argument('--ready', action='store_true')
    args = parser.parse_args()
    try:
        data = [yaml.safe_load(path.read_text()) for path in (args.params,args.hardware,args.mission)]
        errors, pending = validate_event(*data)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'ERROR: incomplete or invalid configuration: {exc}')
        return 1
    for error in errors:
        print(f'ERROR: {error}')
    for item in pending:
        print(f'PENDING: {item}')
    if errors or (args.ready and pending):
        return 1
    print('OK: static template consistency; it does not validate hardware, measured frames or flight.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
