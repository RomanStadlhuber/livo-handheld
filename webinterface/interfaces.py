import subprocess
import os
import time
from datetime import datetime
import psutil
import pathlib


class Interfaces:

    @staticmethod
    def get_storage_devices():
        """Get a list of external devices that can be used to store recorded data."""
        # locations where we allow for external media
        media_roots = ["media", "mnt"]
        # the external media that we found
        media_dirs = []
        # find all disk partitions and use their mountpoints to identify external drives
        parts = psutil.disk_partitions()
        # try to identify if a partitions is an external drive
        for p in parts:
                path = pathlib.Path(p.mountpoint)
                if len(path.parents) > 1:
                        # name of all parent directories
                        pdir = [x.name for x in path.parents]
                        # needs to unpack at least two elements!
                        *_, root, _ = pdir
                        # check if 2nd root directory is in allowed media dirs
                        if root in media_roots:
                                # print(f"{path.name} is media!")
                                media_dirs.append(str(path.resolve()))
        # print("found the following media directories:")	
        return media_dirs


    @staticmethod
    def get_active_bags(storage):
        return [f"storage/mock3.active.bag"]

    @staticmethod
    def get_bags(storage):
        return [f"storage/mock1.bag", f"storage/mock2.bag"]

    pkg_livo = "livo_runner"
    launchfile_cam = "camera.launch"
    launchfile_lidar = "lidar.launch"

    def __init__(self) -> None:
        # initialize launch handles as None so I don't have to check the attr exists
        self.launch_lidar = None
        self.launch_cam = None
        # flags set if the devices could be launched
        self.cam_launched = False
        self.lidar_launched = False
        # start roscore so nodes can be launched
        self.__start_roscore()
        self.devices_started = False
        self.rosbag_record = None

    def start_device_nodes(self):
        if not self.devices_started:
            print("Starting devices.")
            # try to launch the devices
            self.__roslaunch_camera()
            self.__roslaunch_lidar()
            self.devices_started = True
        # return success listing
        print("Devices started!")
        return dict(lidar=self.lidar_launched, camera=self.cam_launched)

    def start_recording(self, base_path, filename):
        bag_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        bag_name = f"{bag_name}.bag"
        cmd = ["rosbag", "record", "-a"]
        if filename is not None and len(filename) > 0:
            bag_name = f"{filename}_{bag_name}"
            cmd.extend(["-o", f"{base_path}/{filename}"])
        print(f"""Recording bag from '{("").join(cmd)}'""")
        self.rosbag_record = subprocess.Popen(cmd)
        return bag_name

    def stop_recording(self):
        if self.rosbag_record is None:
            print("recording subprocess unavailable!!")
        try:
            self.rosbag_record.terminate()
        except Exception as e:
            print(f"Failed to stop recording: {e}")

    def stop_device_nodes(self):
        if self.launch_cam is not None:
            self.launch_cam.terminate()
            self.cam_launched = False
            self.devices_started = False
        if self.launch_lidar is not None:
            self.launch_lidar.terminate()
            self.lidar_launched = False
            self.devices_started = False

    def __check_lidar_available(self, ip_lidar: str = "192.168.1.125"):
        """checks if the LiDAR can be reached over the network"""
        # TODO: this will not not work, replace it!
        response = os.system(f"ping -c 1 {ip_lidar}")
        return response != 0

    def __check_ros_available(self):
        response = os.system("rosnode list")
        return response == 0

    def __start_roscore(self):
        if self.__check_ros_available():
            print("Found existing roscore instance!")
            return True
        print("Starting new roscore instance!")
        subprocess.Popen("roscore")
        while not self.__check_ros_available():
            print("wating for roscore to start")
            time.sleep(0.1)

    def __roslaunch_camera(self):
        try:
            self.launch_cam = subprocess.Popen(["roslaunch", Interfaces.pkg_livo, Interfaces.launchfile_cam])
            self.cam_launched = True
        except Exception as e:
            print(f"failed to launch camera: {str(e)}")

    def __roslaunch_lidar(self):
        try:
            self.launch_lidar = subprocess.Popen(["roslaunch", Interfaces.pkg_livo, Interfaces.launchfile_lidar])
            self.lidar_launched = True
        except Exception as e:
            print(f"Failed to launch LiDAR: {str(e)}")


if __name__ == "__main__":
    print(Interfaces.get_storage_devices())
