from ..new_robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
import rospkg

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "pr2" + '.urdf'

pr2_description = RobotDescription("pr2", "base_footprint", "base_link", "torso_lift_link",
                                   filename)

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right_arm", "torso_lift_link", "r_wrist_roll_link",
                                      pr2_description.urdf_object)
right_arm.add_static_joint_states("park", {'r_shoulder_pan_joint': -1.712,
                                           'r_shoulder_lift_joint': -0.256,
                                           'r_upper_arm_roll_joint': -1.463,
                                           'r_upper_arm_joint': -2.12,
                                           'r_elbow_flex_joint': 1.766,
                                           'r_forearm_roll_joint': -0.07,
                                           'r_forearm_joint': 0.051})
pr2_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "r_gripper_palm_link", "r_gripper_tool_frame",
                                       pr2_description.urdf_object)
right_gripper.add_static_joint_states("open", {'r_gripper_l_finger_joint': 0.548,
                                               'r_gripper_r_finger_joint': 0.548})
right_gripper.add_static_joint_states("close", {'r_gripper_l_finger_joint': 0.0,
                                                'r_gripper_r_finger_joint': 0.0})
right_arm.end_effector = right_gripper

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left_arm", "torso_lift_link", "l_wrist_roll_link",
                                     pr2_description.urdf_object)
left_arm.add_static_joint_states("park", {'l_shoulder_pan_joint': 1.712,
                                          'l_shoulder_lift_joint': -0.264,
                                          'l_upper_arm_roll_joint': 1.38,
                                          'l_upper_arm_joint': -2.12,
                                          'l_elbow_flex_joint': 16.996,
                                          'l_forearm_roll_joint': -0.073,
                                          'l_forearm_joint': 0.0})
pr2_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "l_gripper_palm_link", "l_gripper_tool_frame",
                                      pr2_description.urdf_object)
left_gripper.add_static_joint_states("open", {'l_gripper_l_finger_joint': 0.548,
                                              'l_gripper_r_finger_joint': 0.548})
left_gripper.add_static_joint_states("close", {'l_gripper_l_finger_joint': 0.0,
                                               'l_gripper_r_finger_joint': 0.0})
left_arm.end_effector = left_gripper

################################## Camera ##################################
camera = CameraDescription("kinect_camera", "head_mount_kinect_rgb_optical_frame", 1.27,
                           1.60, 0.99483, 0.75049)

################################## Neck ##################################
pr2_description.add_kinematic_chain("neck", "head_pan_link", "head_tilt_link")

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(pr2_description)
