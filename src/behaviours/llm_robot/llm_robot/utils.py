from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Quaternion, PoseStamped


def adjust_tool_orientation_xyz(current_quaternion: Quaternion, target_quaternion: Quaternion)-> Quaternion:
    # Convert quaternions to scipy Rotation objects
    current_rotation = R.from_quat([
        current_quaternion.x,
        current_quaternion.y,
        current_quaternion.z,
        current_quaternion.w
    ])
    target_rotation = R.from_quat([
        target_quaternion.x,
        target_quaternion.y,
        target_quaternion.z,
        target_quaternion.w
    ])

    # Decompose the current rotation into XYZ Euler angles
    current_euler = current_rotation.as_euler('xyz', degrees=True)

    # Decompose the target rotation into XYZ Euler angles
    target_euler = target_rotation.as_euler('xyz', degrees=True)

    # Calculate the difference in yaw (Z component in XYZ) angles in degrees
    current_yaw = current_euler[2]
    target_yaw = target_euler[2]

    # Calculate the absolute difference in yaw angles
    yaw_difference = target_yaw - current_yaw
    abs_yaw_difference = abs(yaw_difference)

    if abs_yaw_difference > 45 and abs_yaw_difference <= 90:
        adjusted_yaw = target_yaw + 90 if yaw_difference < 0 else target_yaw - 90
    elif abs_yaw_difference > 90:
        adjusted_yaw = target_yaw + 180 if yaw_difference < 0 else target_yaw - 180
    else:
        adjusted_yaw = target_yaw

    # Ensure adjusted_yaw is within the range of -180 to 180 degrees
    if adjusted_yaw > 180:
        adjusted_yaw -= 360
    elif adjusted_yaw < -180:
        adjusted_yaw += 360
    print([current_yaw,target_yaw,adjusted_yaw])

    # Adjust the yaw (Z component in XYZ) in the adjusted Euler angles
    adjusted_euler = current_euler.copy()
    adjusted_euler[2] = adjusted_yaw

    # Convert the adjusted Euler angles back to a quaternion
    adjusted_rotation = R.from_euler('xyz', adjusted_euler, degrees=True)
    adjusted_quaternion = adjusted_rotation.as_quat()

    # Convert to geometry_msgs.msg.Quaternion
    adjusted_quat_msg = Quaternion()
    adjusted_quat_msg.x = adjusted_quaternion[0]
    adjusted_quat_msg.y = adjusted_quaternion[1]
    adjusted_quat_msg.z = adjusted_quaternion[2]
    adjusted_quat_msg.w = adjusted_quaternion[3]

    return adjusted_quat_msg