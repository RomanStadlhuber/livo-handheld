from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml

# NOTE: all positions in [m]


"""Setup translations and rotations"""

# LiDAR position in CAD frame
CAD_p_L = np.array([0.0, 0.0, 0.025917]).reshape((3, 1))  # [m]
# IMU position in LiDAR frame
# see MID-360 manual v2024 p. 15 first paragraph
L_p_I = np.array([0.011, 0.02329, 0.04412]).reshape((3, 1))  # [m]
# (approximate) camera position in CAD frame
CAD_p_C = np.array([0.037613, 0.0, -0.01838]).reshape((3, 1))  # [m]
# orientation of the camera in CAD frame
CAD_r_C = R.from_euler(  # tilt by -10° degrees about y-axis
    "xyz",
    (0, -10, 0),
    degrees=True,
) * R.from_matrix(  # convert from camera axes to IMU axes
    [
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0],
    ]
)

"""Setup transform matrices"""

# LiDAR in IMU frame
I_Tf_L = np.block(
    [
        [np.eye(3), -1 * L_p_I],
        [np.array([0, 0, 0, 1])],
    ]
)

# LiDAR in CAD frame
L_Tf_CAD = np.block(
    [
        [np.eye(3), -1 * CAD_p_L],
        [np.array([0, 0, 0, 1])],
    ]
)

# Camera in CAD frame
CAD_Tf_C = np.block(
    [
        [CAD_r_C.as_matrix(), CAD_p_C],
        [np.array([0, 0, 0, 1])],
    ]
)


# compute transformation from IMU to Camera
I_Tf_C = I_Tf_L @ L_Tf_CAD @ CAD_Tf_C


# output relevant transformations
print("(Lidar in IMU frame) I_Tf_L:")
print(I_Tf_L)
print("(Camera in IMU frame) I_Tf_C:")
print(I_Tf_C)
