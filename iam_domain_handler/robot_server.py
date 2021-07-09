import rospy
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from domain_handler_msgs.srv import RunSkill

from iam_skills import CmdType, BaseStreamTrajSkill, BaseGripperSkill
from .state_client import StateClient
from .skill_registry_client import SkillRegistryClient
from .utils import EE_RigidTransform_from_state, joints_from_state


class RobotServer:

    def __init__(self, skills_dict):
        self._skills_dict = skills_dict

        # Only support gripper skill or skills that stream ee or joint trajs for now
        for skill in self._skills_dict.values():
            assert isinstance(skill, BaseStreamTrajSkill) or isinstance(skill, BaseGripperSkill)

        self._fa = FrankaArm(old_gripper=True)
        self._state_client = StateClient()
        self._skill_registry_client = SkillRegistryClient()

        self._traj_sensor_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)

        self._run_skill_srv = rospy.Service('run_skill', RunSkill, self._run_skill_srv_handler)

        rospy.loginfo('Running Robot Server...')
        rospy.spin()

    def _run_policy_on_robot(self, init_state, policy, param, skill):
        if policy.cmd_type == CmdType.GRIPPER:
            gripper_cmd = policy(init_state)
            if gripper_cmd['target_width'] is not None:
                self._fa.goto_gripper(gripper_cmd['target_width'])
            elif gripper_cmd['close_gripper'] is not None:
                self._fa.close_gripper()
            elif gripper_cmd['open_gripper'] is not None:
                self._fa.open_gripper()
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
                if skill.termination_condition_satisfied(state, param, policy, t_step) > 0.5:
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

    def _run_skill_srv_handler(self, req):
        skill_info = self._skill_registry_client.get_skill_info(req.skill_id)
        self._skill_registry_client.set_skill_status(req.skill_id, 'running')

        skill = self._skills_dict[skill_info.skill_name]
        
        init_state = self._state_client.get_state()
        policy = skill.make_policy(init_state, skill_info.skill_param)

        t_step = self._run_policy_on_robot(init_state, policy, skill_info.skill_param, skill)

        end_state = self._state_client.get_state()

        if skill.skill_execution_successful(init_state, end_state, skill_info.skill_param, policy, t_step) > 0.5:
            skill_status = 'success'
        else:
            skill_status = 'failure'
        self._skill_registry_client.set_skill_status(req.skill_id, skill_status)
        return skill_status