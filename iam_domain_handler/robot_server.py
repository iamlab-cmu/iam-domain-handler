import time
import rospy
import numpy as np
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import convert_rigid_transform_to_array

from domain_handler_msgs.srv import RunSkill, GetSkillTraj 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from domain_handler_msgs.msg import GetTrajectory
from std_msgs.msg import Header

from iam_skills import *
from .state_client import StateClient
from .memory_client import MemoryClient
from .action_registry_client import ActionRegistryClient
from .utils import EE_RigidTransform_from_state, joints_from_state, create_formated_skill_dict

class RobotServer:

    def __init__(self, skills_dict):
        rospy.init_node('robot_server')
        self._skills_dict = skills_dict

        # Only support gripper skill or skills that stream ee or joint trajs for now
        for skill in self._skills_dict.values():
            assert isinstance(skill, BaseStreamTrajSkill) or isinstance(skill, BaseGripperSkill) or isinstance(skill, BaseForceSkill)
        
        self._fa = FrankaArm()
        self._state_client = StateClient()
        self._memory_client = MemoryClient()
        self._action_registry_client = ActionRegistryClient()

        self._traj_sensor_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)

        self._run_skill_srv = rospy.Service('run_skill', RunSkill, self._run_skill_srv_handler)
        self._get_skill_traj_srv = rospy.Service('get_skill_traj', GetSkillTraj, self._get_skill_traj_srv_handler)

        self._set_state_trajectory_pub = rospy.Publisher('/set_state_trajectory', GetTrajectory, queue_size=1000)

        rospy.loginfo('Running Robot Server...')
        rospy.spin()

    def _run_policy_on_robot(self, skill_id, init_state, policy, param, skill):
        if policy.cmd_type == CmdType.GRIPPER:
            gripper_cmd = policy(init_state)
            if gripper_cmd['target_width'] is not None:
                self._fa.goto_gripper(gripper_cmd['target_width'])
            elif gripper_cmd['close_gripper'] is not None:
                self._fa.close_gripper()
            elif gripper_cmd['open_gripper'] is not None:
                self._fa.open_gripper()
            return 1
        elif policy.cmd_type == CmdType.FORCE:
            end_effector_position = []
            time_since_skill_started = []
            joints = []
            start_time = time.time()
            last_time = None

            rate = rospy.Rate(1 / policy.dt)
            t_step = 0
            fa.run_guide_mode(duration=policy.duration, block=False)

            while True:
                state = self._state_client.get_state()
                t_step += 1

                if skill.termination_condition_satisfied(state, param, policy, t_step) > 0.5 \
                or self._action_registry_client.get_action_status(skill_id) == 'cancelled':
                    break

                if policy.record:
                    pose_array = convert_rigid_transform_to_array(EE_RigidTransform_from_state(state))
                    end_effector_position.append(pose_array)
                    joints.append(joints_from_state(state))
                    time_since_skill_started.append(time.time() - start_time)

                rate.sleep()

            if policy.record:
                skill_dict = create_formated_skill_dict(joints, end_effector_position, time_since_skill_started)
                self._memory_client.set_memory_objects({'recorded_trajectory' : skill_dict})
            
            return 1
        else:
            if policy.cmd_type == CmdType.EE:
                init_pose = EE_RigidTransform_from_state(init_state)
                self._fa.goto_pose(
                    init_pose, 
                    duration=policy.horizon, 
                    dynamic=True, 
                    buffer_time=3
                )
            elif policy.cmd_type == CmdType.JOINT:
                init_joints = joints_from_state(init_state)
                self._fa.goto_joints(
                    init_joints, 
                    duration=policy.horizon, 
                    dynamic=True, 
                    buffer_time=3
                )
            
            rate = rospy.Rate(1 / policy.dt)
            t_step = 0
            init_time = rospy.Time.now().to_time()
            while True:
                state = self._state_client.get_state()
                t_step += 1
                if skill.termination_condition_satisfied(state, param, policy, t_step) > 0.5 \
                or self._action_registry_client.get_action_status(skill_id) == 'cancelled':
                    break

                traj_item = policy(state)
                timestamp = rospy.Time.now().to_time() - init_time

                if policy.cmd_type == CmdType.EE:
                    traj_gen_proto_msg = PosePositionSensorMessage(
                        id=t_step, 
                        timestamp=timestamp, 
                        position=traj_item.translation, 
                        quaternion=traj_item.quaternion
                    )
                    ros_msg = make_sensor_group_msg(
                        trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                            traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION
                        )
                    )
                elif policy.cmd_type == CmdType.JOINT:
                    traj_gen_proto_msg = JointPositionSensorMessage(
                        id=t_step, 
                        timestamp=timestamp, 
                        joints=traj_item
                    )
                    ros_msg = make_sensor_group_msg(
                        trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                            traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION
                        )
                    )    
                
                self._traj_sensor_pub.publish(ros_msg)
                rate.sleep()

            term_proto_msg = ShouldTerminateSensorMessage(
                timestamp=rospy.Time.now().to_time() - init_time, 
                should_terminate=True
            )
            ros_msg = make_sensor_group_msg(
                termination_handler_sensor_msg=sensor_proto2ros_msg(
                    term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE
                )
            )
            self._traj_sensor_pub.publish(ros_msg)

            return t_step

    
    def _get_skill_traj_srv_handler(self, req): 
        skill = self._skills_dict[req.skill_name]
        assert isinstance(skill, BaseStreamTrajSkill)

        init_state = self._state_client.get_state()
        policy = skill.make_policy(init_state, req.skill_param)
        assert isinstance(policy, StreamTrajPolicy)

        pts = []
        for traj in policy._traj:
            pt = JointTrajectoryPoint()
            pt.positions = traj
            pts.append(pt)

        pts_msg = JointTrajectory()
        pts_msg.points = pts
        pts_traj_msg = GetTrajectory()
        pts_traj_msg.trajectory = pts_msg

        self._set_state_trajectory_pub.publish(pts_traj_msg)
        trajectory_done = False
        while not trajectory_done:
            cur_state = self._state_client.get_state()
            if not (cur_state.has_prop('skill_trajectory_done') and cur_state.has_prop('skill_trajectory')):
                continue 
            if cur_state['skill_trajectory_done'][0] > 0:
                rospy.loginfo('Successfully getting trajectory from robot server...')
                trajectory_done = True
        return 'success'
    
    def _run_skill_srv_handler(self, req):
        skill_info = self._action_registry_client.get_action_info(req.skill_id)
        self._action_registry_client.set_action_status(req.skill_id, 'running')

        skill = self._skills_dict[skill_info.action_name]
        
        init_state = self._state_client.get_state()
        policy = skill.make_policy(init_state, skill_info.action_param)

        t_step = self._run_policy_on_robot(req.skill_id, init_state, policy, skill_info.action_param, skill)

        end_state = self._state_client.get_state()

        if skill.skill_execution_successful(init_state, end_state, skill_info.action_param, policy, t_step) > 0.5:
            skill_status = 'success'
        else:
            skill_status = 'failure'

        self._action_registry_client.set_action_status(req.skill_id, skill_status)
        return skill_status