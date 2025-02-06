from __future__ import annotations # python 3.8 compatibility

import subprocess
import os
import time
from datetime import datetime
import psutil
import pathlib
from typing import List
import numpy as np
import ipaddress
import json
import time
import shutil
from multiprocessing import Process


def copy_finished_recordings(base_path: str):
    """Process target to wait for recordings to finish.

    Copies all finished recordings to the desired `base_path`.
    """
    tmp = pathlib.Path("/tmp")
    # loop until all bags are copied or time expires
    while True:
        # all rosbag files
        bagfiles = [
                x for x in tmp.iterdir()
                if x.is_file()
                and ".bag" in x.suffixes
            ]
        # bags that are still being recorded/buffered
        active_bags = [x for x in bagfiles if ".active" in x.suffixes]
        # bags that finished recording
        finished_bags = [x for x in bagfiles if ".active" not in x.suffixes]
        # loop until there are no active bag files
        if len(active_bags) == 0 and len(finished_bags) == 0:
            # done
            return
        elif len(finished_bags) != 0:
            # copy all finished recordings to desired directory
            for bagfile in finished_bags:
                print(f"::: moving finished recording {str(bagfile)} to storage device {base_path}:::")
                # TODO: maybe use rsync instead?
                source = str(bagfile)
                # name during transfer
                target_buf = os.path.join(base_path, bagfile.name + ".active")
                # name during transfer (".active" makes the front-end indicate that it's buffering)
                target_fin = os.path.join(base_path, bagfile.name)
                # transfer file from /tmp to storage device
                shutil.move(source, target_buf)
                # when transfer is done, rename back to original name
                os.rename(target_buf, target_fin)
                print("::: done copying recording :::")

        # short delay
        time.sleep(0.1) # [sec]


class Interfaces:
    """Class to interface with the processes involved in creating recordings."""

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
        

    @staticmethod
    def get_ros2_bags(storage_location: str) -> List[str]:
        """List all previously recorded rosbags in storage_location.
        
        Currently, functionality only returns the names without further info."""
        if storage_location is None or not pathlib.Path(storage_location).exists():
            return []
        recorded_bags = []
        for x in pathlib.Path(storage_location).iterdir():
            if Interfaces.__is_ros2_bag(x):
                filename = x.name
                filesize_str = Interfaces.__str_filesize(Interfaces.__get_folder_size(x))
                # add buffering note to active bags
                # doing this because buffering takes a while for compressed images
                if not any("metadata" in y.name for y in x.iterdir() if y.is_file()):
                    filesize_str += " (buffering..)"
                recorded_bags.append((filename, filesize_str))
        return recorded_bags


    @staticmethod
    def __is_ros2_bag(pth: pathlib.Path, storage_extensions: list[str] = [".db3", ".mcap"]) -> bool:
        # cannot be a ros2 bag if it's not a directory
        if not pth.is_dir():
            return False
        # check if directory structure is valid
        else:
            files = [x for x in pth.iterdir() if x.is_file()]
            if not files: # cannot determine if folder is a bag when it's empty
                return False            
            elif not any((f.suffix in storage_extensions) for f in files): # need at least one storage file
                return False
            # all checks passed
            else:
                return True

            
    @staticmethod
    def __get_file_size(x: str) -> int:
        """Get the size of a file in bytes. Does not apply to folders!"""
        return os.path.getsize(str(x))

    @staticmethod
    def __get_folder_size(pth: pathlib.Path) -> int:
        """Get the size of a folder in bytes."""
        # source: https://stackoverflow.com/a/1392549/10493834
        return sum(f.stat().st_size for f in pth.glob('**/*') if f.is_file())
            

    pkg_ros2_recorder = "recorder_runner"
    launchfile_ros2_all = "recording.launch.py"
    # ROS1 (legacy)
    pkg_livo = "livo_runner"
    launchfile_cam = "camera_imu.launch"
    launchfile_lidar = "lidar.launch"

    @staticmethod
    def rospack_find(package_name:str) -> str:
        """Find the absolute path to a ROS package."""
        # https://stackoverflow.com/a/1724723
        for root, dirs, files in os.walk("/catkin_ws"):
            if package_name in dirs:
                return os.path.join(root, package_name)
        print(f"ERROR: Interfaces unable to find rospack '{package_name}'")
        raise FileNotFoundError(f"Unable to find rospack '{package_name}'")

    @staticmethod
    def ros2_pkg_find(package_name: str) -> str:
        """Find the absolute path to a ROS package."""
        # https://stackoverflow.com/a/1724723
        for root, dirs, files in os.walk("/ros2_ws/src"):
            if package_name in dirs:
                return os.path.join(root, package_name)
        print(f"""ERROR: Interfaces unable to find ros2 pkg '{package_name}'
    Can you find it using >$ ros2 pkg prefix {package_name}< ?"""
)
        raise FileNotFoundError(f"Unable to find ros2 pkg '{package_name}'")


    def __init__(self) -> None:
        # initialize launch handles as None so I don't have to check the attr exists
        self.launch_lidar = None
        self.launch_cam = None
        # flags set if the devices could be launched
        self.cam_launched = False
        self.lidar_launched = False
        # start roscore so nodes can be launched
        # NOTE: not needed in ROS2 setup
        # self.__start_roscore()
        self.devices_started = False
        self.rosbag_record = None
        # placeholder for the base-path used to store finished recordings
        self.recording_base_path = None
        # ROS2 refactoring
        self.launch_ros2_recorder = None

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
            #self.__roslaunch_camera()
            #self.__roslaunch_lidar()
            self.__ros2_launch_sensors()
            self.devices_started = True
        # return success listing
        print("Devices started!")
        # TODO: is there a better way to find out if the lidar was launched?
        # the camera should always be launched, though
        self.lidar_launched = self.__check_lidar_available()
        self.cam_launched = True
        return dict(lidar=self.lidar_launched, camera=self.cam_launched)

    def start_recording(self, base_path, filename):
        # store base_path of new recording, which is required when stopping the recording process
        self.recording_base_path = base_path
        bag_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        cmd = ["ros2", "bag" , "record", f"-b={int(1e9)}"] 
        # set full bag filename, if provided
        if filename is not None and len(filename) > 0:
            bag_name = f"{filename}"
        cmd.extend(["-o", f"{base_path}/{bag_name}"])
        cmd.extend([
            "/camera/image_raw",
            "/camera/image_raw/compressed", # record only compressed to save space
            "/livox/lidar",
            "/livox/imu",
            # "/clock"
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
            # NOTE: with ROS2, this will be obsolete because the bag is recorded to external storage in splits
            ## # start a separate process that will copy finished recordings to the storage device
            ## if self.recording_base_path is not None:
            ##     print("::: staring rosbag copy waiting process :::")
            ##     p = Process(target=copy_finished_recordings, args=(self.recording_base_path,))
            ##     p.start()
            # reset base_path buffer until next recording
            self.recording_base_path = None
        except Exception as e:
            print(f"Failed to stop recording: {e}")

    def stop_device_nodes(self):
        # ROS1 (legacy)
        if self.launch_cam is not None:
            self.launch_cam.terminate()
            self.cam_launched = False
            self.devices_started = False
        if self.launch_lidar is not None:
            self.launch_lidar.terminate()
            self.lidar_launched = False
            self.devices_started = False
        # ROS2
        if self.launch_ros2_recorder is not None:
            self.launch_ros2_recorder.terminate()
            self.lidar_launched = self.cam_launched = self.devices_started = False


    def __check_lidar_available(self) -> bool:
        # path to the JSON config file containing the LiDAR IP address
        try:
            config_path = os.path.join(
                    Interfaces.ros2_pkg_find(Interfaces.pkg_ros2_recorder),
                    "launch/MID360_config.json"
                )
            with open(config_path) as f:
                conf = json.load(f)
                ip_lidar = conf["lidar_configs"][0]["ip"]
                ip_lidar = ipaddress.ip_address(ip_lidar)
                network_eth0 = ipaddress.ip_network("192.168.1.0/24")
                return ip_lidar in network_eth0
        except Exception as e:
            print("::: ERROR occurred when trying to find LiDAR availability :::")
            print(e)
            return False


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
        if not self.__check_lidar_available():
            self.lidar_launched = False
            return
        try:
            self.launch_lidar = subprocess.Popen(["roslaunch", Interfaces.pkg_livo, Interfaces.launchfile_lidar])
            self.lidar_launched = True
        except Exception as e:
            print(f"Failed to launch LiDAR: {str(e)}")

    def __ros2_launch_sensors(self):
        try:
            self.launch_ros2_recorder = subprocess.Popen(["ros2", "launch", Interfaces.pkg_ros2_recorder, Interfaces.launchfile_ros2_all])
        except Exception as e:
            print(f"""Failed to launch ROS2 recorder node! Is it installed?
{str(e)}
""")

if __name__ == "__main__":
    print(Interfaces.get_storage_devices())
