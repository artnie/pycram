
@dataclass
class GeneratedActionPerformable(TransportActionPerformable):
    @with_tree
    def perform(self) -> None:
        code = gen.generate_perform_for_action("TransportActionPerformable")
        for expr in code:
            print(expr)
            try:
                exec(expr)
            except Exception as e:
                print(e)

# BEGIN Condition-enriched action plan
#
# class PickUp(ActionDesignatorDescription):
#     """
#     Let the robot pick up an object. The robot moves to the desired object, reaches for it and and picks it up.
#     """
#
#     object: ObjectDesignatorDescription
#     """
#     Object designator describing the object that should be picked up
#     """
#
#     arm: Arms
#     """
#     The arm that should be used for pick up
#     """
#
#     grasp: Grasp
#     """
#     The grasp that should be used. For example, 'left' or 'right'
#     """
#
#     @with_tree
#     def ground(self) -> PickUp:
#         if self.object.Object.world_object is None:
#             raise ValueError("The object designator must be grounded before the pick up action can be grounded.")
#         self.arm = self.arm if not self.arm is None else World.robot.get_free_gripper()
#         self.grasp = self.grasp if not self.grasp is None else ManipulationInterface.get_grasp_for(self)
#         return self
#
#     @with_tree
#     def perform(self) -> None:
#         robot = World.robot
#         environment = World.world
#         all_objects = World.objects
#
#         collision_free_conditions = [
#             AvoidCollision(environment),
#             AvoidCollision(obj for obj in all_objects if obj not in robot.get_held_objects())
#         ]
#
#         # Move to the target location
#         move_goals = [RobotInReachFor(self.object)].append(collision_free_conditions)
#         move_target_location = Location(move_goals)
#         move = Primitive.MoveBase(location=move_target_location)
#         Monitor(action=move)
#         move.perform()
#
#         # Open the gripper
#         open_gripper = Primitive.OpenGripper(gripper=self.arm, goal=GripperState.OPEN)
#         open_gripper.perform()
#
#         # Reach for the object
#         reach_goals = [GripperAround(self.object, self.arm)].append(collision_free_conditions)
#         reach = Primitive.MoveGripper(gripper=self.arm, location=Location(reach_goals))
#         Monitor(action=reach)
#         reach.perform()
#
#         # Close the gripper
#         close_gripper_goals = [
#             InCollision(self.object, self.arm),
#             AvoidCollision(environment),
#             not JointState(self.arm, state=GripperState.CLOSE)
#         ]
#         close_gripper = Primitive.CloseGripper(gripper=self.arm,
#                                                goal=close_gripper_goals,
#                                                strength=GripperState.get_gripping_strength_for(self.object))
#         Monitor(action=close_gripper)
#         close_gripper.perform()
#
#         # Lift the object
#         lift_goals = [LiftUp(self.object, self.arm)].append(collision_free_conditions)

#       ------
#
#         # Retrieve object and robot from designators
#         object = self.object_designator.world_object
#         # Get grasp orientation and target pose
#         grasp = RobotDescription.current_robot_description.grasps[self.grasp]
#         # oTm = Object Pose in Frame map
#         oTm = object.get_pose()
#         # Transform the object pose to the object frame, basically the origin of the object frame
#         mTo = object.local_transformer.transform_to_object_frame(oTm, object)
#         # Adjust the pose according to the special knowledge of the object designator
#         adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
#         # Transform the adjusted pose to the map frame
#         adjusted_oTm = object.local_transformer.transform_pose(adjusted_pose, "map")
#         # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
#
#         adjusted_oTm.multiply_quaternions(grasp)
#
#         # prepose depending on the gripper (its annoying we have to put pr2_1 here tbh
#         # gripper_frame = "pr2_1/l_gripper_tool_frame" if self.arm == "left" else "pr2_1/r_gripper_tool_frame"
#         gripper_frame = robot.get_link_tf_frame(
#             RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame())
#         # First rotate the gripper, so the further calculations makes sense
#         tmp_for_rotate_pose = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
#         tmp_for_rotate_pose.pose.position.x = 0
#         tmp_for_rotate_pose.pose.position.y = 0
#         tmp_for_rotate_pose.pose.position.z = -0.1
#         gripper_rotate_pose = object.local_transformer.transform_pose(tmp_for_rotate_pose, "map")
#
#         # Perform Gripper Rotate
#         # BulletWorld.current_bullet_world.add_vis_axis(gripper_rotate_pose)
#         # MoveTCPMotion(gripper_rotate_pose, self.arm).resolve().perform()
#
#         oTg = object.local_transformer.transform_pose(adjusted_oTm, gripper_frame)
#         oTg.pose.position.x -= 0.1  # in x since this is how the gripper is oriented
#         prepose = object.local_transformer.transform_pose(oTg, "map")
#
#         # Perform the motion with the prepose and open gripper
#         World.current_world.add_vis_axis(prepose)
#         MoveTCPMotion(prepose, self.arm, allow_gripper_collision=True).perform()
#         MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()
#
#         # Perform the motion with the adjusted pose -> actual grasp and close gripper
#         World.current_world.add_vis_axis(adjusted_oTm)
#         MoveTCPMotion(adjusted_oTm, self.arm, allow_gripper_collision=True).perform()
#         adjusted_oTm.pose.position.z += 0.03
#         MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm).perform()
#         tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
#         robot.attach(object, tool_frame)
#
#         # Lift object
#         World.current_world.add_vis_axis(adjusted_oTm)
#         MoveTCPMotion(adjusted_oTm, self.arm, allow_gripper_collision=True).perform()
#
#         # Remove the vis axis from the world
#         World.current_world.remove_vis_axis()
