# intel-realsense-studio

Tools for working with an Intel RealSense **D435i**, including on **WSL2**.

## Viewer

`src/realsense_viewer.py` opens one window with six live panels:

| | | |
|---|---|---|
| Colour | Depth (colourised 0.2–4 m, hover for distance) | IR left |
| Accelerometer (x/y/z, 4 s) | Gyro (x/y/z, 4 s) | IR right |

```bash
.venv/bin/python src/realsense_viewer.py              # interactive window
.venv/bin/python src/realsense_viewer.py --snapshot out.png   # render one frame, no window
```

| Key | Action |
|---|---|
| `q` / Esc | quit |
| `s` | save the window as PNG and the depth (metres) as `.npy` to `snapshots/` |
| `e` | toggle the IR laser emitter |
| `p` | pause / resume |

Video runs at 848×480 @ 30 fps. The IMU runs with the accelerometer at 200 Hz and the gyro at 400 Hz. The D435i supports accelerometer rates of 100, 200 and 400 Hz and gyro rates of 200 and 400 Hz only.

## Setup on WSL2 (needed for the IMU)

The PyPI `pyrealsense2` wheel streams video under WSL2 but **cannot see the IMU**. The stock WSL kernel lacks `CONFIG_HID_SENSOR_HUB`, the driver the default backend needs to read the D435i's motion sensor. The fix is to build librealsense with the **RSUSB** backend, which talks to the camera over libusb instead.

1. **Attach the camera to WSL** from Windows (admin PowerShell, [usbipd-win](https://github.com/dorssel/usbipd-win)):
   ```powershell
   usbipd list
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```
   Check in WSL that `lsusb -t` shows the camera at `5000M` (USB 3).

2. **System packages and udev rule**. Get the source first (step 3), then run:
   ```bash
   sudo apt-get install -y libusb-1.0-0-dev libudev-dev
   sudo cp librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. **Build librealsense** (tested with v2.59.2):
   ```bash
   git clone --depth 1 -b v2.59.2 https://github.com/IntelRealSense/librealsense.git
   uv venv .venv && uv pip install --python .venv/bin/python -r requirements.txt
   cmake -S librealsense -B librealsense/build -DCMAKE_BUILD_TYPE=Release \
     -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS=ON \
     -DPYTHON_EXECUTABLE=$PWD/.venv/bin/python \
     -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=ON \
     -DCHECK_FOR_UPDATES=OFF -DBUILD_UNIT_TESTS=OFF
   cmake --build librealsense/build -j$(nproc)
   ```

4. **Point the venv at the built bindings** without running `cmake --install`, which tries to write to `/usr/local`:
   ```bash
   echo "$PWD/librealsense/build/Release" > \
     "$(.venv/bin/python -c 'import site;print(site.getsitepackages()[0])')/pyrealsense2-local.pth"
   .venv/bin/python -c "import pyrealsense2 as rs; print(rs.__file__)"   # should point into librealsense/build
   ```
   Do **not** `pip install pyrealsense2` into this venv. It would shadow the RSUSB build and the IMU would disappear.

Tools such as `rs-enumerate-devices` and `rs-fw-update` end up in `librealsense/build/Release/`.

## Notes

- With the RSUSB backend, only one pipeline can open the device. The viewer therefore starts the IMU directly on the Motion Module sensor of the device that the video pipeline opened.
- If you re-attach the camera with usbipd and access fails, run `sudo udevadm trigger`.
- On native Linux with the default V4L backend, the PyPI wheel is usually enough and steps 2 to 4 are optional.
