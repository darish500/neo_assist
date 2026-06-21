#!/usr/bin/env python3
"""
NeoAssist auto_mapper.py — v4
==============================
Fixes vs v3:

  1. ROTATION-SAFE CLEARANCE CHECK
     v3 only checked a ~110° forward arc (front/fleft/fright) before
     declaring a turn "done". The robot's actual footprint (base_link
     box 0.55m x 0.45m) sweeps a circle of radius 0.355m around its
     center when rotating in place:
         R = sqrt((0.55/2)^2 + (0.45/2)^2) = 0.355 m
     An obstacle sitting beside the robot (90°/270°) was invisible to
     the old check, so the robot would rotate until the FRONT looked
     clear, declare success, drive forward, and re-hit the same
     obstacle that had simply rotated out of the front cone.
     Fix: TURNING now exits only when nothing in ANY direction is
     closer than ROBOT_RADIUS + SAFETY_MARGIN, using the full 360°
     scan, debounced over 5 consecutive ticks (not 1) to avoid acting
     on a single noisy/gap reading.

  2. YAW-AWARE, STATE-AGNOSTIC STUCK DETECTION
     v3's check_stuck() only measured (x,y) displacement and only ever
     fired recovery when state == "FORWARD". A robot wedged WHILE
     TURNING (wheels spinning, body jammed against a wall/corner)
     never tripped recovery — explaining "stuck there forever".
     Fix: track heading (yaw) from /odom too. Recovery can now trigger
     from FORWARD or TURNING if BOTH position and heading fail to
     change meaningfully over the check interval.

  3. HARD TIMEOUT ON TURNING
     If a turn still can't clear after 15s, something is wrong with
     that approach (too tight, geometry trap) — force a real recovery
     (back up + reassess) instead of rotating forever.

  4. ESCALATING RECOVERY
     Repeated recovery at the same spot increases back-up distance and
     turn angle each time, instead of repeating an identical maneuver
     that already failed.

Geometry reference (from neo_robot_urdf.xacro):
  base_length = 0.55, base_width = 0.45         -> circumscribed R = 0.355 m
  caster reach (corner casters)  = 0.304 m       (inside the box corner, not limiting)
  LiDAR mounted at (x=0, y=0)                    -> sensor center == rotation center
  Gazebo DiffDrive: max_linear_accel = 0.8 m/s², max_angular_accel = 1.5 rad/s²
      -> FWD_SPEED=0.20 m/s ramps up in ~0.25s, TURN_SPEED=0.50 rad/s in ~0.33s.
         Neither is accel-limited in a way that causes collisions; the v3 bug
         was purely in the clearance/stuck LOGIC, not in speed/accel values.
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# ── Robot geometry (derived from URDF, see header) ───────────────────
ROBOT_RADIUS   = 0.355   # m — circumscribed radius, sweeps during in-place rotation
SAFETY_MARGIN  = 0.15    # m — buffer beyond the physical hull
TURN_CLEARANCE = ROBOT_RADIUS + SAFETY_MARGIN   # 0.505 m — required in ANY direction to exit a turn

STOP_DIST  = 0.50   # m — obstacle triggers turn (front-cone, distance from sensor/center)
SLOW_DIST  = 0.90   # m — start slowing
FWD_SPEED  = 0.20   # m/s — slow = cleaner SLAM
TURN_SPEED = 0.40   # rad/s — slightly reduced from v3's 0.5 for finer control in tight spaces

# Odom-based stuck detection (now also tracks yaw)
STUCK_CHECK_INTERVAL   = 3.0    # check every N seconds
STUCK_MOVE_THRESHOLD   = 0.08   # must move at least 8cm in that interval
STUCK_YAW_THRESHOLD    = 0.15   # rad (~8.6°) — must rotate at least this much if state is TURNING

# Turning safety
TURN_CLEAR_DEBOUNCE = 5     # consecutive clear ticks required before exiting TURNING
MAX_TURN_TICKS      = 150   # 15s hard cap — beyond this, force recovery instead of turning forever

# Spin to fill map gaps
SPIN_EVERY = 100  # seconds


class AutoMapper(Node):
    def __init__(self):
        super().__init__("auto_mapper")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan_fixed", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10)

        self.ranges = []
        self.pos_x  = None
        self.pos_y  = None
        self.yaw    = None

        # State machine
        self.state        = "FORWARD"   # FORWARD | TURNING | RECOVERING | SPINNING
        self.turn_dir     = 1
        self.turn_ticks   = 0
        self.clear_streak = 0           # consecutive all-clear ticks while turning
        self.recover_ticks = 0

        # Odom stuck detection
        self.last_check_x   = None
        self.last_check_y   = None
        self.last_check_yaw = None

        # Escalating recovery — resets once the robot makes real progress
        self.recovery_attempts = 0

        self.last_spin_time = self.get_clock().now()

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Separate 3s timer for odom-based stuck check
        self.stuck_timer = self.create_timer(STUCK_CHECK_INTERVAL, self.check_stuck)

        self.get_logger().info(
            f"AutoMapper v4 ready — turn_clearance={TURN_CLEARANCE:.3f}m, "
            f"yaw-aware stuck detection enabled")

    # ── Callbacks ────────────────────────────────────────────────────

    def scan_cb(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

    def odom_cb(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (roll/pitch assumed ~0 for a ground robot)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ── Odom stuck check (every 3s) — now yaw-aware, checks FORWARD + TURNING ──
    def check_stuck(self):
        if self.pos_x is None or self.yaw is None:
            return
        if self.state in ("RECOVERING", "SPINNING"):
            # Reset reference so we don't immediately re-trigger after recovery
            self.last_check_x   = self.pos_x
            self.last_check_y   = self.pos_y
            self.last_check_yaw = self.yaw
            return

        if self.last_check_x is None:
            self.last_check_x   = self.pos_x
            self.last_check_y   = self.pos_y
            self.last_check_yaw = self.yaw
            return

        dist = math.hypot(self.pos_x - self.last_check_x,
                           self.pos_y - self.last_check_y)
        dyaw = abs(_angle_diff(self.yaw, self.last_check_yaw))

        self.last_check_x   = self.pos_x
        self.last_check_y   = self.pos_y
        self.last_check_yaw = self.yaw

        if self.state == "FORWARD" and dist < STUCK_MOVE_THRESHOLD:
            self.get_logger().warn(
                f"STUCK (forward)! Only moved {dist:.3f}m in "
                f"{STUCK_CHECK_INTERVAL}s — triggering recovery")
            self._start_recovery()

        elif self.state == "TURNING" and dist < STUCK_MOVE_THRESHOLD and dyaw < STUCK_YAW_THRESHOLD:
            # Wedged WHILE turning: wheels commanded to spin but the body is
            # neither translating nor rotating. v3 never caught this case.
            self.get_logger().warn(
                f"STUCK (turning)! moved {dist:.3f}m, rotated {math.degrees(dyaw):.1f}° "
                f"in {STUCK_CHECK_INTERVAL}s — wedged, triggering recovery")
            self._start_recovery()

    # ── LiDAR sector helper ───────────────────────────────────────────
    def sector_min(self, center_deg: float, half_deg: float) -> float:
        if not self.ranges:
            return float("inf")
        n    = len(self.ranges)
        step = 360.0 / n
        vals = []
        lo = int((center_deg - half_deg) / step)
        hi = int((center_deg + half_deg) / step)
        for i in range(lo, hi + 1):
            r = self.ranges[i % n]
            if r and not math.isnan(r) and not math.isinf(r) and r > 0.05:
                vals.append(r)
        return min(vals) if vals else float("inf")

    def min_range_all_around(self) -> float:
        """Minimum valid return across the FULL 360° scan.

        Used to decide whether it's safe to STOP turning. Unlike the old
        front-only cone, this catches obstacles beside or behind the robot
        that the in-place rotation would otherwise sweep into.
        """
        if not self.ranges:
            return float("inf")
        vals = [r for r in self.ranges
                 if r and not math.isnan(r) and not math.isinf(r) and r > 0.05]
        return min(vals) if vals else float("inf")

    # ── Recovery trigger ──────────────────────────────────────────────
    def _start_recovery(self):
        self.state    = "RECOVERING"
        self.turn_dir = random.choice([-1, 1])
        self.recovery_attempts += 1

        # Escalate: each consecutive failed recovery backs up further and
        # turns harder, instead of repeating a maneuver that already failed.
        back_ticks = min(25 + 10 * (self.recovery_attempts - 1), 70)   # cap 7s
        turn_ticks = min(35 + 10 * (self.recovery_attempts - 1), 80)   # cap 8s
        self.recover_ticks = back_ticks + turn_ticks
        self._recover_back_ticks = back_ticks
        self._recover_turn_ticks = turn_ticks

        self.get_logger().info(
            f"Recovery #{self.recovery_attempts}: back={back_ticks*0.1:.1f}s, "
            f"turn={turn_ticks*0.1:.1f}s")

    # ── Main control loop (10 Hz) ─────────────────────────────────────
    def control_loop(self):
        twist = Twist()

        # ── RECOVERING: back up then turn hard ───────────────────────
        # self.recover_ticks counts down from (back_ticks + turn_ticks).
        # While it's still above turn_ticks, we're in the back-up phase;
        # once it drops to/below that, we're in the turn-hard phase.
        if self.state == "RECOVERING":
            self.recover_ticks -= 1
            if self.recover_ticks > self._recover_turn_ticks:
                twist.linear.x  = -0.18   # back up
                twist.angular.z =  0.0
            elif self.recover_ticks > 0:
                twist.angular.z = TURN_SPEED * 1.3 * self.turn_dir  # turn hard
                twist.linear.x  = 0.0
            else:
                self.state = "FORWARD"
                self.clear_streak = 0
                self.get_logger().info("Recovery done — resuming forward")
            self.cmd_pub.publish(twist)
            return

        # ── SPINNING: 360° map fill ───────────────────────────────────
        if self.state == "SPINNING":
            self.recover_ticks -= 1
            twist.angular.z = 0.4
            if self.recover_ticks <= 0:
                self.state = "FORWARD"
                self.get_logger().info("Spin done — resuming forward")
                self.cmd_pub.publish(Twist())
            else:
                self.cmd_pub.publish(twist)
            return

        # ── Periodic spin ─────────────────────────────────────────────
        now = self.get_clock().now()
        elapsed = (now - self.last_spin_time).nanoseconds / 1e9
        if elapsed > SPIN_EVERY:
            self.get_logger().info("Starting 360° map-fill spin")
            self.state = "SPINNING"
            self.recover_ticks = int((2 * math.pi / 0.4) * 10) + 5  # ticks for full spin
            self.last_spin_time = now
            return

        # ── LiDAR readings ────────────────────────────────────────────
        front  = self.sector_min(0,   25)
        fleft  = self.sector_min(35,  20)
        fright = self.sector_min(325, 20)
        left   = self.sector_min(80,  30)
        right  = self.sector_min(280, 30)

        # ── TURNING state ─────────────────────────────────────────────
        if self.state == "TURNING":
            self.turn_ticks += 1

            # Hard timeout — never turn forever. If we still can't clear
            # after 15s, this approach is bad (too tight / geometry trap);
            # fall back to a real recovery instead of spinning indefinitely.
            if self.turn_ticks >= MAX_TURN_TICKS:
                self.get_logger().warn(
                    "TURNING exceeded 15s without clearing — forcing recovery")
                self._start_recovery()
                self.cmd_pub.publish(Twist())
                return

            # Exit condition now uses the FULL 360° clearance (not just the
            # forward cone) so we don't declare "clear" while still close
            # to something beside/behind us that rotation would sweep into.
            all_around_clear = self.min_range_all_around() > TURN_CLEARANCE

            if all_around_clear:
                self.clear_streak += 1
            else:
                self.clear_streak = 0

            # Require a minimum turn duration AND a debounced streak of
            # clear readings — avoids exiting on one noisy/gap reading.
            if self.turn_ticks >= 25 and self.clear_streak >= TURN_CLEAR_DEBOUNCE:
                self.state = "FORWARD"
                self.turn_ticks = 0
                self.clear_streak = 0
                self.recovery_attempts = 0   # real progress — reset escalation
            else:
                twist.angular.z = TURN_SPEED * self.turn_dir
                self.cmd_pub.publish(twist)
                return

        # ── FORWARD state: check for obstacles ───────────────────────
        if front < STOP_DIST or fleft < STOP_DIST * 0.85 or fright < STOP_DIST * 0.85:
            self.state        = "TURNING"
            self.turn_ticks    = 0
            self.clear_streak  = 0
            # Pick direction toward most open space
            if left > right + 0.3:
                self.turn_dir = 1
            elif right > left + 0.3:
                self.turn_dir = -1
            else:
                self.turn_dir = random.choice([-1, 1])
            self.get_logger().debug(
                f"Obstacle: front={front:.2f} fl={fleft:.2f} fr={fright:.2f} "
                f"→ turning {'L' if self.turn_dir > 0 else 'R'}")
            twist.angular.z = TURN_SPEED * self.turn_dir
            self.cmd_pub.publish(twist)
            return

        # ── Drive forward ─────────────────────────────────────────────
        speed = FWD_SPEED
        if front < SLOW_DIST:
            speed = FWD_SPEED * (front / SLOW_DIST)
        twist.linear.x = max(0.05, speed)

        # Gentle wall-follow correction
        if left < 0.65:
            twist.angular.z = -0.15
        elif right < 0.65:
            twist.angular.z =  0.15

        self.cmd_pub.publish(twist)



def _angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a-b, wrapped to [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()