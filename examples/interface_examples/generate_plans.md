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
```
```python
with simulated_robot:
    start_pose = Pose([1.3, 2.7, 0], [0, 0, 1, 0])
    milk_target_pose = Pose([5.34, 3.55, 0.8])

    NavigateAction([start_pose]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    
    handle_link_name = "handle_cab3_door_top"
    handle_designator = ObjectPart(names=[handle_link_name], part_of=apartment_desig.resolve())
```

```python
from pycram.failures import IKError

with simulated_robot:
    OpenAction(object_designator_description=handle_designator.resolve(), arms=[Arms.LEFT]).resolve().perform()
```

---
END OF PROGRESS
---
```python
from pycram.failures import IKError

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
    object_designator = DetectAction(milk_desig).resolve().perform()
    print(object_designator)
```

To move the robot we need to create a description and resolve it to an actual Designator. The description of navigation
only needs a list of possible poses.

```python
from pycram.designators.action_designator import NavigateAction
from pycram.datastructures.pose import Pose

pose = Pose([1.3, 2, 0], [0, 0, 0, 1])

# This is the Designator Description
navigate_description = NavigateAction(target_locations=[pose])

# This is the performable Designator
navigate_designator = navigate_description.resolve()
```

What we now did was: create the pose where we want to move the robot, create a description describing a navigation with
a list of possible poses (in this case the list contains only one pose) and create an action designator from the
description. The action designator contains the pose picked from the list of possible poses and can be performed.

```python
from pycram.process_module import simulated_robot

with simulated_robot:
    navigate_designator.perform()
```

Every designator that is performed needs to be in an environment that specifies where to perform the designator either
on the real robot or the simulated one. This environment is called {meth}`~pycram.process_module.simulated_robot`  similar there is also
a {meth}`~pycram.process_module.real_robot` environment.

There are also decorators which do the same thing but for whole methods, they are called {meth}`~pycram.process_module.with_real_robot` 
and {meth}`~pycram.process_module.with_simulated_robot`.

## Move Torso

This action designator moves the torso up or down, specifically it sets the torso joint to a given value.

We start again by creating a description and resolving it to a designator. Afterwards, the designator is performed in
a {meth}`~pycram.process_module.simulated_robot` environment.

```python
from pycram.designators.action_designator import MoveTorsoAction
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import TorsoState

torso_pose = TorsoState.HIGH

torso_desig = MoveTorsoAction([torso_pose]).resolve()

with simulated_robot:
    torso_desig.perform()
```

## Set Gripper

As the name implies, this action designator is used to open or close the gripper.

The procedure is similar to the last time, but this time we will shorten it a bit.

```python
from pycram.designators.action_designator import SetGripperAction
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import GripperState, Arms

gripper = Arms.RIGHT
motion = GripperState.OPEN

with simulated_robot:
    SetGripperAction(grippers=[gripper], motions=[motion]).resolve().perform()
```

## Park Arms

Park arms is used to move one or both arms into the default parking position.

```python
from pycram.designators.action_designator import ParkArmsAction
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()
```

## Pick Up and Place

Since these two are dependent on each other, meaning you can only place something when you picked it up beforehand, they
will be shown together.

These action designators use object designators, which will not be further explained in this tutorial so please check
the example on object designators for more details.

To start we need an environment in which we can pick up and place things as well as an object to pick up.

```python
world.reset_world()
```

```python
from pycram.designators.action_designator import PickUpAction, PlaceAction, ParkArmsAction, MoveTorsoAction,NavigateAction
from pycram.designators.object_designator import BelieveObject
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms, Grasp, TorsoState
from pycram.datastructures.pose import Pose

milk_desig = BelieveObject(names=["milk"])
arm = Arms.RIGHT

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

    NavigateAction([Pose([1.8, 2, 0.0],
                         [0.0, 0.0, 0., 1])]).resolve().perform()

    PickUpAction(object_designator_description=milk_desig,
                 arms=[arm],
                 grasps=[Grasp.RIGHT]).resolve().perform()

    PlaceAction(object_designator_description=milk_desig,
                target_locations=[Pose([2.4, 1.8, 1], 
                                       [0, 0, 0, 1])],
                arms=[arm]).resolve().perform()
```

## Look At

Look at lets the robot look at a specific point, for example if it should look at an object for detecting.

```python
world.reset_world()
```

```python
from pycram.designators.action_designator import LookAtAction
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose

target_location = Pose([3, 2, 0.5], [0, 0, 0, 1])
with simulated_robot:
    LookAtAction(targets=[target_location]).resolve().perform()
```

## Detect

Detect is used to detect objects in the field of vision (FOV) of the robot. We will use the milk used in the pick
up/place example, if you didn't execute that example you can spawn the milk with the following cell. The detect
designator will return a resolved instance of an ObjectDesignatorDescription.

```python
world.reset_world()
```

```python
from pycram.designators.action_designator import DetectAction, LookAtAction, ParkArmsAction, NavigateAction
from pycram.designators.object_designator import BelieveObject
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose

milk_desig = BelieveObject(names=["milk"])

with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    NavigateAction([Pose([1.7, 2, 0], [0, 0, 0, 1])]).resolve().perform()

    LookAtAction(targets=[milk_desig.resolve().pose]).resolve().perform()

    obj_desig = DetectAction(milk_desig).resolve().perform()

    print(obj_desig)
```

## Transporting

Transporting can transport an object from its current position to another target position. It is similar to the Pick and
Place plan used in the Pick-up and Place example. Since we need an Object which we can transport we spawn a milk, you
don't need to do this if you already have spawned it in a previous example.

```python
world.reset_world()
```

```python
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import Arms, TorsoState

milk_desig = BelieveObject(names=["milk"])

description = TransportAction(milk_desig,
                              [Pose([2.4, 1.8, 1], 
                                       [0, 0, 0, 1])],
                              [Arms.LEFT])
with simulated_robot:
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    description.resolve().perform()
```

## Opening

Opening allows the robot to open a drawer, the drawer is identified by an ObjectPart designator which describes the
handle of the drawer that should be grasped.

For the moment this designator works only in the apartment environment, therefore we remove the kitchen and spawn the
apartment.

```python
world.reset_world()
```

```python
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import Arms, TorsoState
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose

apartment_desig = BelieveObject(names=["apartment"]).resolve()
handle_deisg = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig)

with simulated_robot:
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    NavigateAction([Pose([1.7474915981292725, 2.6873629093170166, 0.0],
                         [-0.0, 0.0, 0.5253598267689507, -0.850880163370435])]).resolve().perform()
    OpenAction(handle_deisg, [Arms.RIGHT]).resolve().perform()
```

## Closing

Closing lets the robot close an open drawer, like opening the drawer is identified by an ObjectPart designator
describing the handle to be grasped.

This action designator only works in the apartment environment for the moment, therefore we remove the kitchen and spawn
the apartment. Additionally, we open the drawer such that we can close it with the action designator.

```python
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose

apartment_desig = BelieveObject(names=["apartment"]).resolve()
handle_deisg = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig)

with simulated_robot:
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    NavigateAction([Pose([1.7474915981292725, 2.8073629093170166, 0.0],
                         [-0.0, 0.0, 0.5253598267689507, -0.850880163370435])]).resolve().perform()
    CloseAction(handle_deisg, [Arms.RIGHT]).resolve().perform()
```
