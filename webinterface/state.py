from dataclasses import dataclass


# camera + IMU, running?
# LiDAR, running?
# recording currently running?
#   which storage location
#   recording display name (= file name)


class State:
    """Manage the state of the recording application.

    State is used to prevent duplicate recorings.
    """

    def __init__(self) -> None:
        # True or False
        self._cam_imu_running = False
        self._lidar_running = False
        # str or None
        self._storage_location = None
        self._active_recording = None
        # whether recording is currently active
        self._currently_recording = False

    def set_devices_running(self, cam_imu: bool, lidar: bool) -> None:
        self._cam_imu_running = cam_imu
        self._lidar_running = lidar

    def set_recording_path(self, base_path: str, filename: str) -> None:
        self._storage_location = base_path
        self._active_recording = filename
        self._currently_recording = True
    
    def set_recording_stopped(self) -> None:
        # NOTE: self._storage_location is kept for next recording
        self._active_recording = None
        self._currently_recording = False


    @property
    def lidar_running(self) -> bool:
        return self._lidar_running

    @property
    def cam_imu_running(self) -> bool:
        return self._cam_imu_running

    @property
    def recording_basepath(self) -> str:
        return self._storage_location

    @property
    def recording_filename(self) -> str:
        return self._active_recording

    @property
    def currently_recording(self) -> bool:
        return self._currently_recording

