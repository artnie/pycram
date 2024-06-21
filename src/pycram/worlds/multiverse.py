import logging
import os
from time import time, sleep

from typing_extensions import List, Dict, Optional, Tuple

from ..datastructures.dataclasses import AxisAlignedBoundingBox, Color, ContactPointsList, ContactPoint
from ..datastructures.enums import WorldMode, JointType, ObjectType
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..description import Link, Joint, LinkDescription
from ..world_concepts.constraints import Constraint
from ..world_concepts.multiverse_socket import MultiverseSocket, SocketAddress
from ..world_concepts.world_object import Object


def get_resource_paths(dirname: str) -> List[str]:
    resources_paths = ["../robots", "../worlds", "../objects"]
    resources_paths = [
        os.path.join(dirname, resources_path.replace('../', '')) if not os.path.isabs(
            resources_path) else resources_path
        for resources_path in resources_paths
    ]

    def add_directories(path: str) -> None:
        with os.scandir(path) as entries:
            for entry in entries:
                if entry.is_dir():
                    resources_paths.append(entry.path)
                    add_directories(entry.path)

    resources_path_copy = resources_paths.copy()
    for resources_path in resources_path_copy:
        add_directories(resources_path)

    return resources_paths


def find_multiverse_resources_path() -> Optional[str]:
    """
    Find the path to the Multiverse resources directory.
    """
    # Get the path to the Multiverse installation
    multiverse_path = find_multiverse_path()

    # Check if the path to the Multiverse installation was found
    if multiverse_path:
        # Construct the path to the resources directory
        resources_path = os.path.join(multiverse_path, 'resources')

        # Check if the resources directory exists
        if os.path.exists(resources_path):
            return resources_path

    return None


def find_multiverse_path() -> Optional[str]:
    """
    Find the path to the Multiverse installation.
    """
    # Get the value of PYTHONPATH environment variable
    pythonpath = os.getenv('PYTHONPATH')

    # Check if PYTHONPATH is set
    if pythonpath:
        # Split the PYTHONPATH into individual paths using the platform-specific path separator
        paths = pythonpath.split(os.pathsep)

        # Iterate through each path and check if 'Multiverse' is in it
        for path in paths:
            if 'multiverse' in path:
                multiverse_path = path.split('multiverse')[0]
                return multiverse_path + 'multiverse'

    return None


class Multiverse(MultiverseSocket, World):
    """
    This class implements an interface between Multiverse and PyCRAM.
    """

    _joint_type_to_position_name: Dict[JointType, str] = {
        JointType.REVOLUTE: "joint_rvalue",
        JointType.PRISMATIC: "joint_tvalue",
    }
    """
    A dictionary to map JointType to the corresponding multiverse attribute name.
    """

    added_multiverse_resources: bool = False
    """
    A flag to check if the multiverse resources have been added.
    """

    def __init__(self, simulation: str, mode: Optional[WorldMode] = WorldMode.DIRECT,
                 is_prospection: Optional[bool] = False,
                 simulation_frequency: Optional[float] = 60.0,
                 client_addr: Optional[SocketAddress] = SocketAddress(port="7000")):
        """
        Initialize the Multiverse Socket and the PyCram World.
        param mode: The mode of the world (DIRECT or GUI).
        param is_prospection: Whether the world is prospection or not.
        param simulation_frequency: The frequency of the simulation.
        param client_addr: The address of the multiverse client.
        """
        MultiverseSocket.__init__(self, client_addr)
        World.__init__(self, mode, is_prospection, simulation_frequency)
        self.simulation: str = simulation
        self._make_sure_multiverse_resources_are_added()
        self.set_attached_objects_poses = False
        self.handle_spawning = False
        self.update_poses_on_get = True
        self.last_object_id: int = -1
        self.last_constraint_id: int = -1
        self.constraints: Dict[int, Constraint] = {}
        self.object_name_to_id: Dict[str, int] = {}
        self.object_id_to_name: Dict[int, str] = {}
        self.time_start = time()
        self.run()
        self.floor = self._spawn_floor()

    def _init_world(self, mode: WorldMode):
        pass

    def _make_sure_multiverse_resources_are_added(self):
        """
        Add the multiverse resources to the pycram world resources.
        """
        if not self.added_multiverse_resources:
            dirname = find_multiverse_resources_path()
            resources_paths = get_resource_paths(dirname)
            for resource_path in resources_paths:
                self.add_resource_path(resource_path)
            self.added_multiverse_resources = True

    def _spawn_floor(self):
        """
        Spawn the plane in the simulator.
        """
        floor = Object("floor", ObjectType.ENVIRONMENT, "plane.urdf",
                   world=self)
        sleep(0.5)
        return floor

    def get_joint_position_name(self, joint: Joint) -> str:
        if joint.type not in self._joint_type_to_position_name:
            logging.warning(f"Invalid joint type: {joint.type}")
            return "joint_rvalue"
        return self._joint_type_to_position_name[joint.type]

    def load_object_and_get_id(self, name: Optional[str] = None,
                               pose: Optional[Pose] = None) -> int:
        """
        Spawn the object in the simulator and return the object id. Object name has to be unique and has to be same as
        the name of the object in the description file.
        param name: The name of the object to be loaded.
        param pose: The pose of the object.
        """
        if pose is None:
            pose = Pose()

        self._reset_body_pose(name, pose)

        self.last_object_id += 1

        self.object_name_to_id[name] = self.last_object_id
        self.object_id_to_name[self.last_object_id] = name

        return self.last_object_id

    def _init_spawn_object(self, name: str) -> None:
        """
        Initialize the object spawning process.
        """
        self._init_setter()
        self.request_meta_data["send"][name] = ["position", "quaternion"]
        self.send_and_receive_meta_data()

    def get_object_joint_names(self, obj: Object) -> List[str]:

        return [joint.name for joint in obj.description.joints
                if joint.type in [JointType.REVOLUTE, JointType.PRISMATIC]]

    def get_object_link_names(self, obj: Object) -> List[str]:
        return [link.name for link in obj.description.links]

    def _init_getter(self):
        self.request_meta_data["receive"] = {}
        self.request_meta_data["send"] = {}
        self.request_meta_data["meta_data"]["simulation_name"] = self._meta_data.simulation_name

    def get_joint_position(self, joint: Joint) -> float:
        self.check_object_exists_and_issue_warning_if_not(joint.object)
        self._init_getter()
        attribute = self.get_joint_position_name(joint)
        self.request_meta_data["receive"][joint.name] = [attribute]
        self.send_and_receive_meta_data()
        receive_data = self.response_meta_data["receive"][joint.name][attribute]
        if len(receive_data) != 1:
            logging.error(f"Invalid joint position data: {receive_data}")
            raise ValueError
        return receive_data[0]

    def _init_setter(self):
        self.request_meta_data["send"] = {}
        self.request_meta_data["receive"] = {}
        self.set_simulation_in_request_meta_data()

    def set_simulation_in_request_meta_data(self):
        self.request_meta_data["meta_data"]["simulation_name"] = self.simulation

    def reset_joint_position(self, joint: Joint, joint_position: float) -> None:
        self._init_setter()
        attribute = self.get_joint_position_name(joint)
        self.request_meta_data["send"][joint.name] = [attribute]
        self.send_and_receive_meta_data()
        self.send_data = [time() - self.time_start, joint_position]
        self.send_and_receive_data()

    def get_link_pose(self, link: Link) -> Pose:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        return self._get_body_pose(link.name)

    def get_object_pose(self, obj: Object) -> Pose:
        self.check_object_exists_and_issue_warning_if_not(obj)
        return self._get_body_pose(obj.name)

    def _get_body_pose(self, body_name: str) -> Pose:
        self._init_getter()
        self.request_meta_data["receive"][body_name] = ["position", "quaternion"]
        self.send_and_receive_meta_data()
        self.send_data = [time() - self.time_start]
        self.send_and_receive_data()
        if len(self.receive_data) != 8:
            logging.error(f"Invalid body pose data: {self.receive_data}")
            raise ValueError
        wxyz = self.receive_data[4:]
        xyzw = [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]
        return Pose(self.receive_data[1:4], xyzw)

    def reset_object_base_pose(self, obj: Object, pose: Pose):
        self.check_object_exists_and_issue_warning_if_not(obj)
        self._reset_body_pose(obj.name, pose)

    def multiverse_reset_world(self):
        self._init_setter()
        self.send_and_receive_meta_data()
        self.send_data = [0]
        self.send_and_receive_data()

    def _reset_body_pose(self, body_name: str, pose: Pose) -> None:
        """
        Reset the pose of a body in the simulator.
        param body_name: The name of the body.
        param pose: The pose of the body.
        """
        self._init_setter()
        self.request_meta_data["send"][body_name] = ["position", "quaternion"]
        self.send_and_receive_meta_data()
        xyzw = pose.orientation_as_list()
        wxyz = [xyzw[3], *xyzw[:3]]
        self.send_data = [time() - self.time_start, *pose.position_as_list(), *wxyz]
        self.send_and_receive_data()

    def get_all_objects_data_from_server(self) -> Dict[str, Dict]:
        """
        Get all objects data from the multiverse server.
        """
        self._init_getter()
        self.request_meta_data["receive"][""] = [""]
        self.send_and_receive_meta_data()
        return self.response_meta_data["receive"]

    def disconnect_from_physics_server(self) -> None:
        self.stop()

    def join_threads(self) -> None:
        pass

    def remove_object_from_simulator(self, obj: Object) -> None:
        self.set_simulation_in_request_meta_data()
        self.request_meta_data["send"][obj.name] = []
        self.request_meta_data["receive"][obj.name] = []
        self.send_and_receive_meta_data()
        self.send_data = [time() - self.time_start]
        self.send_and_receive_data()

    def add_constraint(self, constraint: Constraint) -> int:

        if constraint.type != JointType.FIXED:
            logging.error("Only fixed constraints are supported in Multiverse")
            raise ValueError

        self._request_attach(constraint)

        self.last_constraint_id += 1

        self.constraints[self.last_constraint_id] = constraint

        return self.last_constraint_id

    def _request_attach(self, constraint: Constraint) -> None:
        """
        Request to attach the child link to the parent link.
        param constraint: The constraint.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        self._add_api_request("attach", child_link_name,
                              parent_link_name, self._get_attachment_pose_as_string(constraint))
        self._send_api_request()

    def get_constraint_link_names(self, constraint: Constraint) -> Tuple[str, str]:
        """
        Get the link names of the constraint.
        param constraint: The constraint.
        return: The link names of the constraint.
        """
        return self.get_parent_link_name(constraint), self.get_constraint_child_link_name(constraint)

    def get_parent_link_name(self, constraint: Constraint) -> str:
        """
        Get the parent link name of the constraint.
        param constraint: The constraint.
        return: The parent link name of the constraint.
        """
        return self.get_link_name_for_constraint(constraint.parent_link)

    def get_constraint_child_link_name(self, constraint: Constraint) -> str:
        """
        Get the child link name of the constraint.
        param constraint: The constraint.
        return: The child link name of the constraint.
        """
        return self.get_link_name_for_constraint(constraint.child_link)

    def remove_constraint(self, constraint_id) -> None:
        constraint = self.constraints.pop(constraint_id)
        self._request_detach(constraint)

    def _request_detach(self, constraint: Constraint) -> None:
        """
        Request to detach the child link from the parent link.
        param constraint: The constraint.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        self._add_api_request("detach", child_link_name, parent_link_name)
        self._send_api_request()

    @staticmethod
    def get_link_name_for_constraint(link: Link) -> str:
        """
        Get the link name from link object, if the link belongs to a one link object, return the object name.
        param link: The link.
        return: The link name.
        """
        return link.name if not link.is_only_link else link.object.name

    def _get_attachment_pose_as_string(self, constraint: Constraint) -> str:
        """
        Get the attachment pose as a string.
        param constraint: The constraint.
        return: The attachment pose as a string.
        """
        self.check_object_exists_and_issue_warning_if_not(constraint.parent_link.object)
        self.check_object_exists_and_issue_warning_if_not(constraint.child_link.object)
        pose = constraint.child_link.get_pose_wrt_link(constraint.parent_link)
        return self._pose_to_string(pose)

    @staticmethod
    def _pose_to_string(pose: Pose) -> str:
        """
        Convert the pose to a string.
        param pose: The pose.
        return: The pose as a string.
        """
        return f"{pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.w} {pose.orientation.x} " \
               f"{pose.orientation.y} {pose.orientation.z}"

    def perform_collision_detection(self) -> None:
        logging.warning("perform_collision_detection is not implemented in Multiverse")

    def get_object_contact_points(self, obj: Object) -> ContactPointsList:
        self.check_object_exists_and_issue_warning_if_not(obj)
        contact_bodies = self._request_objects_in_contact(obj)
        contact_points = ContactPointsList([])
        body_link = None
        for body in contact_bodies:
            if body == "world":
                continue
            body_object = self.get_object_by_name(body)
            if body_object is None:
                for obj in self.objects:
                    for link in obj.links.values():
                        if link.name == body:
                            body_link = link
                            break
            else:
                body_link = body_object.root_link
            if body_link is None:
                logging.error(f"Body link not found: {body}")
                raise ValueError
            contact_points.append(ContactPoint(obj.root_link, body_link))
        return contact_points

    def _request_objects_in_contact(self, obj: Object) -> List[str]:
        self._init_api_callback()
        self._add_api_request("get_contact_bodies", obj.name)
        self._send_api_request()
        return self._get_contact_bodies_from_response()

    def _get_contact_bodies_from_response(self) -> List[str]:
        return self._get_api_response("get_contact_bodies")

    def _get_api_response(self, api_name: str) -> List[str]:
        return self.response_meta_data["api_callbacks_response"][self.simulation][0][api_name]

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> ContactPointsList:
        self.check_object_exists_and_issue_warning_if_not(obj1)
        self.check_object_exists_and_issue_warning_if_not(obj2)
        logging.warning("get_contact_points_between_two_objects is not implemented in Multiverse")
        return ContactPointsList([])

    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        logging.error("ray_test is not implemented in Multiverse")
        raise NotImplementedError

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        logging.error("ray_test_batch is not implemented in Multiverse")
        raise NotImplementedError

    def step(self):
        logging.warning("step is not implemented in Multiverse")

    def save_physics_simulator_state(self) -> int:
        logging.warning("save_physics_simulator_state is not implemented in Multiverse")
        return 0

    def remove_physics_simulator_state(self, state_id: int) -> None:
        logging.warning("remove_physics_simulator_state is not implemented in Multiverse")

    def restore_physics_simulator_state(self, state_id: int) -> None:
        logging.error("restore_physics_simulator_state is not implemented in Multiverse")
        raise NotImplementedError

    def set_link_color(self, link: Link, rgba_color: Color):
        self.check_object_exists_and_issue_warning_if_not(link.object)
        logging.warning("set_link_color is not implemented in Multiverse")

    def get_link_color(self, link: Link) -> Color:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        logging.warning("get_link_color is not implemented in Multiverse")
        return Color()

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        self.check_object_exists_and_issue_warning_if_not(obj)
        logging.warning("get_colors_of_object_links is not implemented in Multiverse")
        return {}

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        self.check_object_exists_and_issue_warning_if_not(obj)
        logging.error("get_object_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        logging.error("get_link_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def set_realtime(self, real_time: bool) -> None:
        logging.warning("set_realtime is not implemented in Multiverse")

    def set_gravity(self, gravity_vector: List[float]) -> None:
        logging.warning("set_gravity is not implemented in Multiverse")

    def check_object_exists_and_issue_warning_if_not(self, obj: Object):
        if obj not in self.objects:
            logging.warning(f"Object {obj.name} does not exist in the simulator")

    def _init_api_callback(self):
        """
        Initialize the API callback in the request metadata.
        """
        self._reset_send_data()
        self._reset_receive_data()
        self._reset_api_callback()
        self.set_simulation_in_request_meta_data()

    def _reset_send_data(self):
        """
        Reset the send data in the request metadata.
        """
        self.request_meta_data["send"] = {}

    def _reset_receive_data(self):
        """
        Reset the receive data in the request metadata.
        """
        self.request_meta_data["receive"] = {}

    def _reset_api_callback(self):
        """
        Reset the API callback in the request metadata.
        """
        self.request_meta_data["api_callbacks"] = {self.simulation: []}

    def _add_api_request(self, api_name: str, *params):
        """
        Add an API request to the request metadata.
        param api_name: The name of the API.
        param params: The parameters of the API.
        """
        if "api_callbacks" not in self.request_meta_data:
            self._init_api_callback()
        self.request_meta_data["api_callbacks"][self.simulation].append({api_name: list(params)})

    def _send_api_request(self):
        """
        Send the API request to the server.
        """
        if "api_callbacks" not in self.request_meta_data:
            logging.error("No API request to send")
            raise ValueError
        self.send_and_receive_meta_data()
        self.request_meta_data.pop("api_callbacks")
