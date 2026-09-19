import numpy as np
import pyrealsense2 as rs
import threading
import time


ALPHA = 0.98 #Determines weigth in filter. 0.98 = 98% trust with gyro, correct with 2% accell

class IMUTracker:
    """
    Reads gyroscope and accelerometer data od the camera and computes stable camera rotation
    """

    def __init__(self):
        self._lock    = threading.Lock()
        self._running = False
        self._pitch   = 0.0
        self._yaw     = 0.0
        self._roll    = 0.0
        self._last_ts = None
        self._pipeline = rs.pipeline()
        self._config   = rs.config()
        self._config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        self._config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)

    def start(self, profile=None):
        self._pipeline.start(self._config)
        self._running = True
        self._last_ts = time.time()
        # Start reading in background thread
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("IMU tracker started!")

    def _read_loop(self):
        """Runs in background — continuously reads IMU frames."""
        while self._running:
            try:
                frameset    = self._pipeline.wait_for_frames()
                gyro_frame  = frameset.first_or_default(rs.stream.gyro)
                accel_frame = frameset.first_or_default(rs.stream.accel)

                if not gyro_frame or not accel_frame:
                    continue

                gyro  = gyro_frame.as_motion_frame().get_motion_data()
                accel = accel_frame.as_motion_frame().get_motion_data()

                now = time.time()
                dt  = now - self._last_ts
                self._last_ts = now

                if dt <= 0 or dt > 1.0:
                    continue

                # Gyro integration
                gyro_pitch = self._pitch + gyro.x * dt
                gyro_yaw   = self._yaw   + gyro.y * dt
                gyro_roll  = self._roll  + gyro.z * dt

                # Accel angle estimation
                ax, ay, az = accel.x, accel.y, accel.z
                norm = np.sqrt(ax**2 + ay**2 + az**2)

                if norm == 0:
                    continue

                ax /= norm
                ay /= norm
                az /= norm

                accel_pitch = np.arctan2(ay, az)
                accel_roll  = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

                # Complementary filter
                with self._lock:
                    self._pitch = ALPHA * gyro_pitch + (1 - ALPHA) * accel_pitch
                    self._yaw   = gyro_yaw
                    self._roll  = ALPHA * gyro_roll  + (1 - ALPHA) * accel_roll

            except Exception as e:
                if self._running:
                    print(f"[IMU] Error: {e}")

    def stop(self):
        self._running = False
        try:
            self._pipeline.stop()
        except Exception:
            pass
        print("IMU tracker stopped!")


    def get_rotation_matrix(self):
        """
        Returns a 3x3 rotation matriz from current pitch, yaw and roll.
        Used by point_map.py to rotate points into world space
        """
        with self._lock:
            pitch = self._pitch
            yaw   = self._yaw
            roll  = self._roll
            #Rotation matrix aroud X axis (pitch)
            Rx = np.array([
                [1,             0,              0],
                [0, np.cos(pitch), -np.sin(pitch)],
                [0, np.sin(pitch),  np.cos(pitch)]
            ])
            Ry = np.array([
                [np.cos(yaw),   0,    np.sin(yaw)],
                [0,             1,             0 ],
                [-np.sin(yaw),  0,    np.cos(yaw)]
            ])
            Rz = np.array([
                [np.cos(roll), -np.sin(roll), 0],
                [np.sin(roll), np.cos(roll),  0],
                [0,           0,              1]
            ])
            return Rz @ Ry @ Rx
        
    def get_euler_angles(self):
        """
        Returns current pitch, yaw and roll n degrees.
        """
        with self._lock:
            return(
                np.degrees(self._pitch),
                np.degrees(self._yaw),
                np.degrees(self._roll)
            )
