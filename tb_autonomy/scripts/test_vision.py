#!/usr/bin/env python3
import argparse
import cv_bridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ColorThresholdTester(Node):
    def __init__(self, args):
        super().__init__("test_vision")
        self.sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.img_callback, 10
        )

        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask RED", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask GREEN", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask BLUE", cv2.WINDOW_NORMAL)

        # Blob detector tuned for binary masks
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 100000
        params.filterByColor = False
        params.filterByInertia = False
        params.filterByConvexity = False
        params.thresholdStep = 60
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.color_mode = args.color
        self.min_area_px = args.min_area_px

        # HSV ranges (OpenCV Hue is typically 0..179)
        # These are reasonable starting points for saturated blocks; tune if needed.
        self.ranges = {
            "green": (np.array([40, 220, 0]), np.array([90, 255, 255])),
            "blue":  (np.array([100, 220, 0]), np.array([150, 255, 255])),
        }

        # Red wraps around hue axis -> two intervals then OR the masks. [web:1][web:3]
        self.red_ranges = [
            (np.array([0,   220, 0]), np.array([10,  255, 255])),
            (np.array([160, 220, 0]), np.array([179, 255, 255])),
        ]

        self.get_logger().info(
            f"color_mode={self.color_mode}, min_area_px={self.min_area_px}"
        )

    def make_mask(self, hsv, color_name: str):
        if color_name == "red":
            m1 = cv2.inRange(hsv, self.red_ranges[0][0], self.red_ranges[0][1])
            m2 = cv2.inRange(hsv, self.red_ranges[1][0], self.red_ranges[1][1])
            mask = cv2.bitwise_or(m1, m2)  # combine red masks [web:1][web:3]
            return mask

        lo, hi = self.ranges[color_name]
        return cv2.inRange(hsv, lo, hi)

    def color_seen(self, mask):
        # Decide "seen" by counting mask pixels (area).
        # This is often more stable than blob count for simple tasks.
        area = int(cv2.countNonZero(mask))
        return area >= self.min_area_px, area

    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        colors_to_check = ["red", "green", "blue"] if self.color_mode == "all" else [self.color_mode]

        seen_any = False
        labeled = img.copy()

        # Show masks for debugging even if not selected
        mask_r = self.make_mask(hsv, "red")
        mask_g = self.make_mask(hsv, "green")
        mask_b = self.make_mask(hsv, "blue")
        cv2.imshow("Mask RED", mask_r)
        cv2.imshow("Mask GREEN", mask_g)
        cv2.imshow("Mask BLUE", mask_b)

        for c in colors_to_check:
            mask = {"red": mask_r, "green": mask_g, "blue": mask_b}[c]
            seen, area = self.color_seen(mask)
            if seen:
                seen_any = True
                self.get_logger().info(f"SEEN {c.upper()} (mask_area={area}px)")

                # Optional: blob detect and draw keypoints for that color
                keypoints = self.detector.detect(mask)
                labeled = cv2.drawKeypoints(
                    labeled, keypoints, None, (255, 0, 0),
                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
                )

        if not seen_any:
            self.get_logger().info("No target color seen")

        cv2.imshow("Detections", labeled)
        cv2.waitKey(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HSV Color Thresholding test script")
    parser.add_argument("--color", choices=["red", "green", "blue", "all"], default="all")
    parser.add_argument("--min_area_px", type=int, default=1500)
    args = parser.parse_args()

    rclpy.init()
    node = ColorThresholdTester(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
