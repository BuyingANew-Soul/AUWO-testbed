#!/usr/bin/env python3
"""Sample the arm's reachable workspace via /compute_ik and report the band.

Sweeps a grid of (x,y,z) points in base_link, asks compute_ik which solve,
writes a CSV point cloud, and prints the empirical reach bounds — the honest
numbers to set reach_r_* / reach_z_* in the situational_context node.
"""
import csv, math, time
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK

ARM_GROUP = "arm"; EE_LINK = "hand_tcp"; PLANNING_FRAME = "base_link"


class ReachProbe(Node):
    def __init__(self):
        super().__init__("reach_probe")
        # grid bounds (base_link) — wide enough to find the true edges
        self.declare_parameter("x_min", -0.10); self.declare_parameter("x_max", 0.50)
        self.declare_parameter("y_min", -0.40); self.declare_parameter("y_max", 0.40)
        self.declare_parameter("z_min", -0.20); self.declare_parameter("z_max", 0.40)
        self.declare_parameter("step", 0.03)          # grid resolution (m)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("out_csv", "/tmp/reach.csv")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")

    def ik_ok(self, x, y, z):
        req = GetPositionIK.Request(); ik = req.ik_request
        ik.group_name = ARM_GROUP; ik.ik_link_name = EE_LINK
        ik.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        ik.pose_stamped.header.frame_id = PLANNING_FRAME
        ik.pose_stamped.pose.position.x = float(x)
        ik.pose_stamped.pose.position.y = float(y)
        ik.pose_stamped.pose.position.z = float(z)
        ik.pose_stamped.pose.orientation.w = 1.0
        ik.timeout.sec = 0; ik.timeout.nanosec = 200_000_000
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        r = fut.result()
        return (r is not None) and (r.error_code.val == 1)

    def run(self):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_ik not available — is demo.launch.py running?")
            return
        p = lambda n: self.get_parameter(n).value
        step = p("step")
        xs = self._range(p("x_min"), p("x_max"), step)
        ys = self._range(p("y_min"), p("y_max"), step)
        zs = self._range(p("z_min"), p("z_max"), step)
        total = len(xs) * len(ys) * len(zs)
        self.get_logger().info(f"probing {total} points at {step*1000:.0f}mm resolution…")

        rows = []; reach = []
        t0 = time.time(); n = 0
        for x in xs:
            for y in ys:
                for z in zs:
                    ok = self.ik_ok(x, y, z)
                    rows.append((round(x,4), round(y,4), round(z,4), int(ok)))
                    if ok: reach.append((x, y, z))
                    n += 1
                    if n % 200 == 0:
                        self.get_logger().info(f"  {n}/{total}  ({len(reach)} reachable)")

        with open(p("out_csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(["x","y","z","reachable"]); w.writerows(rows)

        self.get_logger().info(f"done in {time.time()-t0:.0f}s. CSV → {p('out_csv')}")
        if not reach:
            self.get_logger().warn("no reachable points — widen the grid or check IK")
            return
        rad = [math.hypot(x, y) for x, y, _ in reach]
        zz  = [z for _, _, z in reach]
        self.get_logger().info("── empirical reach band (base_link) ──")
        self.get_logger().info(f"  radius : {min(rad):.3f} … {max(rad):.3f} m")
        self.get_logger().info(f"  z      : {min(zz):.3f} … {max(zz):.3f} m")
        self.get_logger().info(f"  x      : {min(x for x,_,_ in reach):.3f} … {max(x for x,_,_ in reach):.3f}")
        self.get_logger().info(f"  reachable: {len(reach)}/{total} sampled points")

    @staticmethod
    def _range(a, b, step):
        out, v = [], a
        while v <= b + 1e-9:
            out.append(round(v, 4)); v += step
        return out


def main():
    rclpy.init()
    node = ReachProbe()
    node.run()
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
