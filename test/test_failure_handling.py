import time
import unittest
import pycram.plan_failures
from pycram.failure_handling import WithRetries
import pycram.bullet_world
from pycram.designators import action_designator, object_designator
from pycram.process_module import ProcessModule
from pycram.process_module import simulated_robot
from pycram.robot_descriptions import robot_description
from pycram.pose import Pose

from pycram.designators.location_designator import CostmapLocation, Location
from pycram.designators.object_designator import *
from pycram.designators.action_designator import *

from pycram.bullet_world_reasoning import contact

import pycram.task
import anytree

from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.joint_state_publisher import JointStatePublisher



# def raise_collision(self) -> None:
#     raise pycram.plan_failures.NavigationGoalInCollision


class FailureHandlingTestCase(unittest.TestCase):
    world: BulletWorld
    milk: Object
    bowl: Object
    robot: Object
    kitchen: Object

    # setupclass - beim erstellen der testklasse
    # setup - vor jedem testcase
    # testing...
    # teardown - zwischen tests, resets
    # teardownclass - full teardown

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()
        # cls.world.exit()

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.world = BulletWorld("GUI")
        cls.robot = Object(robot_description.name, "pr2", robot_description.name + ".urdf")
        cls.kitchen = Object("kitchen", "environment", "kitchen.urdf")
        cls.milk = Object("milk", "milk", "milk.stl", pose=Pose([1.35, 0.75, 0.95]))

        # rospy.spin()
        # broadcaster = TFBroadcaster()
        # joint_publisher = JointStatePublisher("joint_states", 0.1)

        # cls.bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([1.35, 0.62, 0.95]))
        ProcessModule.execution_delay = False
        # cls.world.register_collision_callback(objectA=cls.robot, objectB=cls.kitchen,
        #                                       callback_collision=raise_collision)

    def test_failed_grasp(self) -> None:
        milk_description = ObjectDesignatorDescription(names=["milk"])

        @with_tree
        def plan():
            location_description = CostmapLocation(milk_description.resolve())
            # location_description = Location(pose=Pose([1.5, 0.0, 0.0], [0, 0, 0, 1]))


            # pickup_pose = location_description.resolve()
            # pickup_pose = next(location_description).resolve()
            # navigate_action = iter(NavigateAction(target_locations=location_description))
            # navigate_action = iter(NavigateAction(target_location_description=location_description))
            # pick_up_action = iter(PickUpAction(object_designator_description=milk_description, arms=["left"],
            #                                    grasps=["right"]))
            # navigate_action.resolve().perform()
            # Retry(pick_up_action).perform()

            # navigate_action = iter(NavigateAction(target_location_description=location_description))
            # navigate_action = NavigateAction(target_location_description=location_description)
            # navigate_action.ground().perform()


            # loc: LocationDesignatorDescription.Location = iter(location_description)
            # print(loc)
            # print(next(loc))
            # print(next(loc))

            navigate_action = NavigateAction(target_location_description=location_description)

            # navigate_action.ground().perform()
            # time.sleep(1)
            # self.assertTrue(contact(self.robot, self.kitchen))

            # with self.assertRaises(pycram.plan_failures.PlanFailure):
            #     WithRetries(navigate_action, retries=1).perform()

            WithRetries(navigate_action, retries=5).perform()



            # try:
            #     print("robot contacts before:... " + str(self.robot.contact_points_simulated()))
            #     print(str(navigate_action.ground().perform()))
            #     # self.world.simulate(1)
            #     for i in range(10):
            #         time.sleep(0.01)
            #         print(str(i) + " " + str(contact(self.robot, self.kitchen)))
            # except NavigationGoalInCollision:
            #     next(navigate_action).perform()

        with simulated_robot:
            plan()

        # tt = pycram.task.task_tree
        # print(anytree.RenderTree(tt))

        # self.assertEqual(True, False)  # add assertion here


if __name__ == '__main__':
    unittest.main()
