import rospy
import tf
from geometry_msgs.msg import PointStamped, Quaternion
import math

def get_camera_to_footprint_transform(camera_frame, footprint_frame):
    # Initialize Transform Listener
    listener = tf.TransformListener()

    # Wait until the listener can get the transform
    listener.waitForTransform(footprint_frame, camera_frame, rospy.Time(0), rospy.Duration(3.0))

    try:
        # Get the transform (translation and rotation) between frames
        (trans, rot) = listener.lookupTransform(footprint_frame, camera_frame, rospy.Time(0))

        # Convert rotation from quaternion to Euler angles (yaw)
        euler = tf.transformations.euler_from_quaternion(rot)
        yaw = euler[2]  # Yaw is the third Euler angle (roll, pitch, yaw)

        rospy.loginfo(f"Transform from {camera_frame} to {footprint_frame}: translation={trans}, yaw={yaw}")

        return trans, yaw

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not get transformation!")
        return None, None


def save_camera_global_coordinates():
    # Initialize the ROS node
    rospy.init_node('camera_global_coordinates')

    # Define the camera frames
    camera_frame = "camera_3/base_footprint"
    footprint_frame = "camera_3/base_footprint"

    # Define the robot base frame (could be the world or a fixed reference frame)
    robot_base_frame = "base_footprint"

#      <arg name="fourth_jb_x_pos" default=" -9.917057"/>
#   <arg name="fourth_jb_y_pos" default="-9.841491"/>
#   <arg name="fourth_jb_z_pos" default=" 0.0"/>
#   <arg name="fourth_jb_yaw"   default=" 0.676506"/>

#   <arg name="fiveth_jb_x_pos" default=" -2.414609"/>
#   <arg name="fiveth_jb_y_pos" default="-9.853471"/>
#   <arg name="fiveth_jb_z_pos" default=" 0.0"/>
#   <arg name="fiveth_jb_yaw"   default=" 2.432570"/>
    # Initial coordinates for the camera in global frame (translation and yaw)
    initial_translation = [ -2.414609, -9.853471, 0]  # x, y, z (from input)
    initial_yaw = 2.432570  # yaw (from input)

    # Get the transform from camera to footprint
    trans, yaw = get_camera_to_footprint_transform(camera_frame, footprint_frame)

    if trans is not None:
        # Calculate the global position of camera1_link in base frame by applying the transformation
        global_translation = {
            'x': initial_translation[0] + trans[0],
            'y': initial_translation[1] + trans[1],
            'z': initial_translation[2] + trans[2]
        }
        
        # Apply yaw from the camera's local frame to footprint frame
        global_orientation = tf.transformations.quaternion_from_euler(0, 0, initial_yaw + yaw)  # assuming no roll/pitch

        # Save the result to a text file in the desired format
        camera_folders = {'jb_0': {'image_raw': '/path/to/camera_folder'}}
        file_name = "camera3_position_orientation"  # You can choose a more meaningful file name

        text_path = f"{file_name}.txt"
        with open(text_path, 'w') as f:
            f.write(f"Position: {global_translation['x']}, {global_translation['y']}, {global_translation['z']}\n")
            f.write(f"Orientation: {global_orientation[0]}, {global_orientation[1]}, {global_orientation[2]}, {global_orientation[3]}\n")

        rospy.loginfo(f"Global coordinates saved to {text_path}")
    else:
        rospy.logwarn("Could not calculate the global coordinates for Camera1.")

if __name__ == '__main__':
    try:
        save_camera_global_coordinates()
    except rospy.ROSInterruptException:
        pass
