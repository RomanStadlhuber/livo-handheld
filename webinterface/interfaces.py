import subprocess
import os
import time
from datetime import datetime
import psutil
import pathlib
import rospy
import roslaunch
from typing import List
import numpy as np

class Interfaces:

    @staticmethod
    def get_storage_devices() -> List[str]:
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
    def get_bags(storage_location: str) -> List[str]:
        """List all previously recorded rosbags in storage_location.
        
        Currently, functionality only returns the names without further info."""
        if storage_location is None or not pathlib.Path(storage_location).exists():
            return []
        recorded_bags = []
        for x in pathlib.Path(storage_location).iterdir():
            if x.is_file() and ".bag" in x.suffixes:
                filename = x.name
                filesize_str = Interfaces.__str_filesize(os.path.getsize(str(x)))
                # add buffering note to active bags
                # doing this because buffering takes a while for compressed images
                if ".active" in x.suffixes:
                    filesize_str += " (buffering..)"
                recorded_bags.append((filename, filesize_str))

        return recorded_bags
        

    pkg_livo = "livo_runner"
    launchfile_cam = "camera_imu.launch"
    launchfile_lidar = "lidar.launch"

    @staticmethod
    def rospack_find(package_name:str) -> str:
        """Find the absolute path to a ROS package."""
        # https://stackoverflow.com/a/1724723
        for root, dirs, files in os.walk(path):
            if package_name in dirs:
                return os.path.join(root, name)
        print(f"ERROR: Interfaces unable to find rospack '{package_name}'")
        raise FileNotFoundError(f"Unable to find rospack '{package_name}'")

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
        # initialize roslaunch API node
        # (see: https://wiki.ros.org/roslaunch/API%20Usage)
        # NOTE: implementation delayed until additional MVP features work
        """rospy.init_node("launcher_node", anonymous=True)
        self.roslaunch_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.roslaunch_uuid)
        # roslaunch instances for cam+imu and LiDAR
        self.roslaunch_cam_imu = None
        self.roslaunch_lidar = None"""

    @staticmethod
    def __str_filesize(size_bytes) -> str:
        """Convert the size form os.path.getsize to human-readable filesize.

        Code obtained from https://stackoverflow.com/a/14822210"""
        if size_bytes == 0:
            return "0B"
        size_name = size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
        i = int(np.floor(np.emath.logn(x=size_bytes, n=1024)))
        p = np.power(1024, i)
        s = np.round(size_bytes / p, decimals=2)
        return "%s %s" % (s, size_name[i])


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
        cmd = ["rosbag", "record", "--buffsize=2048"]
        if filename is not None and len(filename) > 0:
            bag_name = f"{filename}_{bag_name}"
            cmd.extend(["-o", f"{base_path}/{filename}"])
        cmd.extend([
            "/image/compressed", # record only compressed to save space
            "/imu_raw",
            "/livox/lidar",
            "/livox/imu",
            "/clock"
        ])
        # NOTE: when running the bag, decompress with image_transport republish
        # see: https://github.com/TixiaoShan/LVI-SAM/blob/master/launch/include/module_sam.launch#L21
        print(f"""Recording bag from '{(" ").join(cmd)}'""")
        self.rosbag_record = subprocess.Popen(cmd)
        return bag_name

    @property
    def is_recording(self):
        return self.rosbag_record is not None

    def stop_recording(self):
        if self.rosbag_record is None:
            print("recording subprocess unavailable!!")
        try:
            self.rosbag_record.terminate()
            self.rosbag_record = None
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
