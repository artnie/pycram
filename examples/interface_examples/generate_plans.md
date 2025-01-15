---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---


# Generate Plans

```bash
roslaunch pycram ik_and_description.launch
```

```bash
roslaunch iai_apartment apartment_bringup.launch
```

```bash
rviz
# Marker Array to /pycram/viz_marker
```

Import the necessary modules and initialize the world.

```python
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.ros_utils.tf_broadcaster import TFBroadcaster
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.failures import IKError
import pycrap

world = BulletWorld(WorldMode.DIRECT)
world.allow_publish_debug_poses = True
viz = VizMarkerPublisher(interval=0.25)
tf = TFBroadcaster()

robot_name = "pr2"
extension = ObjectDescription.get_file_extension()
pr2 = Object("pr2", pycrap.Robot, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))
apartment = Object("apartment", pycrap.Apartment, "apartment.urdf")

milk = Object("milk", pycrap.Milk, "milk.stl", pose=Pose([0.5, 2.5, 1], [0, 0, 0, 1]))
milk.color = Color(0,0,1,1)

milk_desig = BelieveObject(names=["milk"])
robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])

# Navigate towards the fridge and find the door handle.

with simulated_robot:
    start_pose = Pose([1.3, 2.7, 0], [0, 0, 1, 0])
    milk_target_pose = Pose([5.34, 3.55, 0.8])

    NavigateAction([start_pose]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    
    handle_link_name = "handle_cab3_door_top"
    handle_designator = ObjectPart(names=[handle_link_name], part_of=apartment_desig.resolve())

# Open the fridge door with failure handling.

with simulated_robot:
    try:
        OpenAction(object_designator_description=handle_designator.resolve(), arms=[Arms.RIGHT]).resolve().perform()
    except Exception as e:
        # Check if the exception is an instance of IKError
        if isinstance(e, IKError):
            print("Inverse Kinematics Error occurred:", e)
            ParkArmsAction([Arms.RIGHT]).resolve().perform()
        
            OpenAction(object_designator_description=handle_designator, arms=[Arms.LEFT]).resolve().perform()
            # Handle the IK error specifically (e.g., retry with a different approach)
        else:
            print("An unexpected error occurred:", e)
            # Handle other types of errors   
    LookAtAction(targets=[milk_desig.resolve().pose]).resolve().perform()
    DetectAction(milk_desig).resolve().perform()
    print(milk_desig)

# Pick up the milk from the fridge.
    
with simulated_robot:
    PickUpAction(milk_desig, [Arms.LEFT], [Grasp.BACK]).resolve().perform()
    ParkArmsActionPerformable(Arms.BOTH).perform()

# Find a suitable base pose to put the milk to the target position.
    
with simulated_robot:
    try:
        place_loc = CostmapLocation(
            target=milk_target_pose,
            reachable_for=robot_desig.resolve(),
            reachable_arm=Arms.LEFT,
            grasps=[Grasp.BACK],
        ).resolve()
        NavigateAction([place_loc.pose]).resolve().perform()
        PlaceAction(milk_desig, [milk_target_pose], [Arms.LEFT]).resolve().perform()
    except StopIteration:
        raise ReachabilityFailure(
            f"No reachable location found for the target location: {milk_target_pose}"
        )
    ParkArmsActionPerformable(Arms.BOTH).perform()

# Close the fridge door.

with simulated_robot:
    close_fridge_base_location = AccessingLocation(handle_desig=handle_designator.resolve(), robot_desig=robot_desig.resolve()).resolve()
    NavigateAction([close_fridge_base_location.pose]).resolve().perform()
    CloseAction(handle_designator, [Arms.LEFT]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
```
