#!/usr/bin/env python
import math

import yaml


def calculate_horizontal_fov(width, fx):
    """Calculate horizontal FOV in radians, then convert to degrees"""
    return 2 * math.atan(width / (2 * fx))


def main():
    # Load YAML file
    with open("usb_camera_info.yaml", "r") as f:
        data = yaml.safe_load(f)

    # Extract image size and camera intrinsics
    width = data["image_width"]
    height = data["image_height"]
    fx = data["camera_matrix"]["data"][0]
    distortion = data["distortion_coefficients"]["data"]

    # Calculate horizontal FOV in radians → degrees
    horizontal_fov_rad = calculate_horizontal_fov(width, fx)
    horizontal_fov = round(horizontal_fov_rad, 2)  # rounded to 2 decimal places

    # Output in required XML format
    print(f"""<camera name="camera">
  <horizontal_fov>{horizontal_fov}</horizontal_fov>
  <image>
    <width>{width}</width>
    <height>{height}</height>
    <format>B8G8R8</format>
  </image>
  <distortion>
    <k1>{distortion[0]}</k1>
    <k2>{distortion[1]}</k2>
    <p1>{distortion[2]}</p1>
    <p2>{distortion[3]}</p2>
    <k3>{distortion[4]}</k3>
  </distortion>
</camera>""")


if __name__ == "__main__":
    main()
