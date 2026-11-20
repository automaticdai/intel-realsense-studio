#!/usr/bin/env python3
"""Live viewer for the RealSense D435i: colour, depth, both IR cameras and IMU.

Run with the project venv (uses the RSUSB librealsense build so the IMU works):
    .venv/bin/python src/realsense_viewer.py

Keys:
    q / Esc   quit
    s         save a snapshot (PNG of the window + raw depth .npy) to ./snapshots
    e         toggle the IR laser emitter (dot pattern in the IR images)
    p         pause / resume
Mouse: hover over the depth panel to read the distance at that pixel.
"""
import argparse
import collections
import os
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

W, H, FPS = 848, 480, 30
ACCEL_HZ, GYRO_HZ = 200, 400  # D435i supports accel 100/200/400, gyro 200/400
IMU_WINDOW_S = 4.0
DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 4.0

BG = (24, 24, 24)
FG = (230, 230, 230)
AXIS_COLORS = [(80, 80, 255), (80, 220, 80), (255, 160, 60)]  # x red, y green, z blue (BGR)


class ImuBuffer:
    """Thread-safe ring buffers for accel and gyro samples fed by the IMU callback."""

    def __init__(self):
        self.lock = threading.Lock()
        n = int(IMU_WINDOW_S * GYRO_HZ * 1.5)
        self.data = {"Accel": collections.deque(maxlen=n), "Gyro": collections.deque(maxlen=n)}
        self.counts = {"Accel": 0, "Gyro": 0}

    def callback(self, frame):
        frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
        for f in frames:
            mf = f.as_motion_frame()
            if not mf:
                continue
            name = f.get_profile().stream_name()
            m = mf.get_motion_data()
            with self.lock:
                if name in self.data:
                    self.data[name].append((f.get_timestamp() / 1000.0, m.x, m.y, m.z))
                    self.counts[name] += 1

    def snapshot(self):
        with self.lock:
            return {k: np.array(v) if v else np.zeros((0, 4)) for k, v in self.data.items()}, dict(self.counts)


def label(img, text, org=(8, 22), scale=0.55, color=FG):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def colorize_depth(depth_m):
    clipped = np.clip((depth_m - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M), 0, 1)
    img = cv2.applyColorMap((255 - clipped * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth_m <= 0] = 0  # invalid pixels black
    return img


def plot_imu(samples, title, unit, pw, ph, fixed_range):
    img = np.full((ph, pw, 3), BG, np.uint8)
    top, bottom, left, right = 34, ph - 22, 46, pw - 8
    cv2.rectangle(img, (left, top), (right, bottom), (70, 70, 70), 1)
    if len(samples) < 2:
        label(img, f"{title}: waiting for data...")
        return img
    t = samples[:, 0]
    t_now = t[-1]
    keep = t >= t_now - IMU_WINDOW_S
    t, xyz = t[keep], samples[keep, 1:]
    rng = max(fixed_range, float(np.abs(xyz).max()) * 1.1)
    mid = (top + bottom) / 2
    yscale = (bottom - top) / (2 * rng)
    cv2.line(img, (left, int(mid)), (right, int(mid)), (60, 60, 60), 1)
    for v in (rng, -rng):
        label(img, f"{v:+.1f}", (2, int(mid - v * yscale) + 5), 0.4, (160, 160, 160))
    xs = left + (t - (t_now - IMU_WINDOW_S)) / IMU_WINDOW_S * (right - left)
    for i in range(3):
        ys = mid - xyz[:, i] * yscale
        pts = np.stack([xs, ys], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, AXIS_COLORS[i], 1, cv2.LINE_AA)
    x, y, z = xyz[-1]
    label(img, f"{title} [{unit}]", (8, 22))
    label(img, f"|v|={np.linalg.norm(xyz[-1]):.3f}", (pw - 120, 22), 0.5)
    for i, (n, v) in enumerate(zip("xyz", (x, y, z))):
        label(img, f"{n}={v:+.3f}", (left + 4 + i * ((right - left) // 3), ph - 6), 0.45, AXIS_COLORS[i])
    return img


def fit(img, pw, ph):
    return cv2.resize(img, (pw, ph), interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--panel-width", type=int, default=480, help="width of each of the 6 panels (px)")
    ap.add_argument("--snapshot", metavar="PNG", help="render ~3s, save one composed frame to PNG and exit (no window)")
    args = ap.parse_args()
    pw = args.panel_width
    ph = int(pw * H / W)

    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        raise SystemExit("No RealSense device found. Is it attached to WSL (usbipd attach)?")
    dev = devs[0]
    serial = dev.get_info(rs.camera_info.serial_number)
    name = dev.get_info(rs.camera_info.name)

    # Video and IMU in separate pipelines: video is polled, IMU is pushed via callback at high rate.
    vcfg = rs.config()
    vcfg.enable_device(serial)
    vcfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    vcfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    vcfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
    vcfg.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8, FPS)
    vpipe = rs.pipeline(ctx)
    vprofile = vpipe.start(vcfg)
    depth_sensor = vprofile.get_device().first_depth_sensor()
    depth_units = depth_sensor.get_depth_scale()

    # IMU: start the Motion Module directly on the device the pipeline already opened
    # (with the RSUSB backend a second pipeline cannot open the same device).
    imu = ImuBuffer()
    motion = next((s for s in vprofile.get_device().query_sensors()
                   if s.get_info(rs.camera_info.name) == "Motion Module"), None)
    imu_ok = False
    if motion is None:
        print("IMU unavailable (no Motion Module); showing video only. Are you using the RSUSB build in .venv?")
    else:
        want = {(rs.stream.accel, ACCEL_HZ), (rs.stream.gyro, GYRO_HZ)}
        profiles = [p for p in motion.get_stream_profiles() if (p.stream_type(), p.fps()) in want]
        try:
            motion.open(profiles)
            motion.start(imu.callback)
            imu_ok = True
        except RuntimeError as e:
            print(f"IMU unavailable ({e}); showing video only.")

    win = f"{name} {serial}"
    mouse = {"pos": None}
    if not args.snapshot:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(win, pw * 3, ph * 2)

        def on_mouse(event, x, y, flags, _):
            # Depth panel occupies column 1, row 0 of the grid.
            mouse["pos"] = (x - pw, y) if pw <= x < 2 * pw and 0 <= y < ph else None

        cv2.setMouseCallback(win, on_mouse)

    paused = False
    last_canvas = None
    last_depth_m = None
    fps_t, fps_n, fps = time.time(), 0, 0.0
    imu_t, imu_prev, imu_rates = time.time(), {"Accel": 0, "Gyro": 0}, {"Accel": 0.0, "Gyro": 0.0}
    t_start = time.time()
    os.makedirs("snapshots", exist_ok=True) if not args.snapshot else None

    try:
        while True:
            if not paused:
                fs = vpipe.wait_for_frames(5000)
                depth = fs.get_depth_frame()
                color = fs.get_color_frame()
                ir1, ir2 = fs.get_infrared_frame(1), fs.get_infrared_frame(2)
                if not (depth and color and ir1 and ir2):
                    continue
                depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * depth_units
                last_depth_m = depth_m

                fps_n += 1
                now = time.time()
                if now - fps_t >= 1.0:
                    fps, fps_t, fps_n = fps_n / (now - fps_t), now, 0
                    _, counts = imu.snapshot()
                    dt = now - imu_t
                    imu_rates = {k: (counts[k] - imu_prev[k]) / dt for k in counts}
                    imu_prev, imu_t = counts, now

                p_color = fit(np.asanyarray(color.get_data()), pw, ph)
                p_depth = fit(colorize_depth(depth_m), pw, ph)
                p_ir1 = fit(cv2.cvtColor(np.asanyarray(ir1.get_data()), cv2.COLOR_GRAY2BGR), pw, ph)
                p_ir2 = fit(cv2.cvtColor(np.asanyarray(ir2.get_data()), cv2.COLOR_GRAY2BGR), pw, ph)

                cx, cy = W // 2, H // 2
                c_d = depth_m[cy, cx]
                cv2.drawMarker(p_depth, (pw // 2, ph // 2), (255, 255, 255), cv2.MARKER_CROSS, 12, 1)
                valid = depth_m > 0
                label(p_color, f"Colour  {fps:.1f} fps")
                label(p_depth, f"Depth  centre {c_d:.3f} m  valid {valid.mean() * 100:.0f}%")
                label(p_depth, f"range {DEPTH_MIN_M}-{DEPTH_MAX_M} m (near=red)", (8, ph - 10), 0.45)
                emitter = depth_sensor.get_option(rs.option.emitter_enabled) if depth_sensor.supports(rs.option.emitter_enabled) else -1
                label(p_ir1, f"IR left   emitter {'ON' if emitter == 1 else 'OFF'} [e]")
                label(p_ir2, "IR right")

                if imu_ok:
                    data, _ = imu.snapshot()
                    p_acc = plot_imu(data["Accel"], f"Accel {imu_rates['Accel']:.0f} Hz", "m/s^2", pw, ph, 12.0)
                    p_gyr = plot_imu(data["Gyro"], f"Gyro {imu_rates['Gyro']:.0f} Hz", "rad/s", pw, ph, 1.0)
                else:
                    p_acc = np.full((ph, pw, 3), BG, np.uint8)
                    p_gyr = p_acc.copy()
                    label(p_acc, "IMU unavailable")
                    label(p_gyr, "IMU unavailable")

                top = np.hstack([p_color, p_depth, p_ir1])
                bottom = np.hstack([p_acc, p_gyr, p_ir2])
                last_canvas = np.vstack([top, bottom])

            if last_canvas is None:
                continue
            canvas = last_canvas.copy()

            if mouse["pos"] is not None and last_depth_m is not None:
                mx, my = mouse["pos"]
                sx, sy = int(mx * W / pw), int(my * H / ph)
                if 0 <= sx < W and 0 <= sy < H:
                    d = last_depth_m[sy, sx]
                    cv2.circle(canvas, (pw + mx, my), 4, (255, 255, 255), 1, cv2.LINE_AA)
                    label(canvas, f"{d:.3f} m" if d > 0 else "no depth", (pw + mx + 8, my - 8), 0.55, (255, 255, 255))
            if paused:
                label(canvas, "PAUSED [p]", (canvas.shape[1] // 2 - 60, canvas.shape[0] // 2), 0.8, (0, 220, 255))

            if args.snapshot:
                if time.time() - t_start >= 3.0:
                    cv2.imwrite(args.snapshot, canvas)
                    print(f"saved {args.snapshot}")
                    break
                continue

            cv2.imshow(win, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("p"):
                paused = not paused
            elif key == ord("e") and depth_sensor.supports(rs.option.emitter_enabled):
                cur = depth_sensor.get_option(rs.option.emitter_enabled)
                depth_sensor.set_option(rs.option.emitter_enabled, 0 if cur else 1)
            elif key == ord("s"):
                stamp = time.strftime("%Y%m%d-%H%M%S")
                cv2.imwrite(f"snapshots/{stamp}.png", canvas)
                if last_depth_m is not None:
                    np.save(f"snapshots/{stamp}_depth_m.npy", last_depth_m)
                print(f"saved snapshots/{stamp}.png (+ depth .npy)")
    finally:
        if imu_ok:
            motion.stop()
            motion.close()
        vpipe.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
