#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 无人机自主送货控制节点 (Delivery Mission Node)

基于有限状态机 (FSM) 实现 PX4 四旋翼无人机的自主送货任务:
  INIT -> SET_OFFBOARD_ARM -> TAKEOFF -> GO_TO_A -> HOVER_A -> GO_TO_B -> AUTO_LAND -> DONE

    任务路线: 起飞点 (0,0) -> 取货点 A (4,4) -> 送货点 B (15,12) -> 降落

环境依赖:
    - ROS Noetic (Ubuntu 20.04)
    - PX4-Autopilot + Gazebo Classic 11
    - mavros, mavros_msgs, geometry_msgs

代码边界:
    本节点仅负责读取状态 + 计算并发布目标期望位置 (Setpoints);
    模式切换与电机解锁由 MAVROS 桥接，底层控制由 PX4 飞控自行完成。

作者: Orang3Whale
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


# =============================================================================
# 异常定义
# =============================================================================

class MissionAbortException(Exception):
    """任务中止异常 —— 当无人机状态异常时抛出，终止状态机主循环。"""
    pass


# =============================================================================
# 主类
# =============================================================================

class PX4DeliveryDrone:
    """
    PX4 无人机自主送货控制节点。

    通过 MAVROS 与 PX4 飞控通信，在 Gazebo 仿真环境中完成多航点送货任务。
    使用面向对象架构封装状态、回调与服务代理，便于后续扩展为多机集群。
    """

    def __init__(self):
        # ------------------------------------------------------------------
        # 1. 初始化 ROS 节点
        # ------------------------------------------------------------------
        rospy.init_node('px4_delivery_mission_node', anonymous=True)

        # ------------------------------------------------------------------
        # 2. 命名空间参数配置 (便于后续拓展为多机集群)
        # ------------------------------------------------------------------
        # 单机 SITL 默认无前缀 (MAVROS 直连 /mavros)；多机时设为 "/uav1" 等
        self.ns = rospy.get_param("~uav_namespace", "")

        # ------------------------------------------------------------------
        # 3. 任务核心参数定义
        # ------------------------------------------------------------------
        self.takeoff_alt = rospy.get_param("~takeoff_altitude", 2.5)     # 起飞高度 (m)
        self.pos_tolerance = rospy.get_param("~position_tolerance", 0.3) # 目标点到达判定阈值 (m)
        self.hover_count_target = rospy.get_param("~hover_seconds", 5.0) # 悬停时长 (s)

        # A 点坐标 (取货点)
        self.point_A = {
            "x": rospy.get_param("~point_A/x", 4.0),
            "y": rospy.get_param("~point_A/y", 4.0),
            "z": rospy.get_param("~point_A/z", 2.5),
        }
        # B 点坐标 (送货点)
        self.point_B = {
            "x": rospy.get_param("~point_B/x", 15.0),
            "y": rospy.get_param("~point_B/y", 12.0),
            "z": rospy.get_param("~point_B/z", 2.5),
        }

        # ------------------------------------------------------------------
        # 4. 内部状态变量
        # ------------------------------------------------------------------
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()

        # 有限状态机当前状态
        self.current_mission_state = "INIT"

        # HOVER 阶段计数器
        self.hover_counter = 0
        # 悬停目标循环次数 = 悬停秒数 * 发布频率 (20 Hz)
        self.hover_target_loops = int(self.hover_count_target * 20)

        # ------------------------------------------------------------------
        # 5. 订阅者 (Subscribers)
        # ------------------------------------------------------------------
        self.state_sub = rospy.Subscriber(
            f"{self.ns}/mavros/state", State, self.state_cb
        )
        self.pose_sub = rospy.Subscriber(
            f"{self.ns}/mavros/local_position/pose", PoseStamped, self.pose_cb
        )

        # ------------------------------------------------------------------
        # 6. 发布者 (Publishers)
        # ------------------------------------------------------------------
        self.local_pos_pub = rospy.Publisher(
            f"{self.ns}/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10,
        )

        # ------------------------------------------------------------------
        # 7. 服务客户端 (Service Clients) - 解锁 & 切模式
        # ------------------------------------------------------------------
        arming_srv = f"{self.ns}/mavros/cmd/arming"
        set_mode_srv = f"{self.ns}/mavros/set_mode"
        rospy.loginfo(f"等待 MAVROS 服务就绪: {arming_srv}, {set_mode_srv}")
        rospy.wait_for_service(arming_srv)
        rospy.wait_for_service(set_mode_srv)
        self.arming_client = rospy.ServiceProxy(arming_srv, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(set_mode_srv, SetMode)
        rospy.loginfo("MAVROS 服务已就绪。")

        # ------------------------------------------------------------------
        # 8. 频率控制
        #    PX4 OFFBOARD 模式要求目标点发布频率 >= 2 Hz，实战建议 >= 20 Hz
        # ------------------------------------------------------------------
        self.rate = rospy.Rate(20)

    # =====================================================================
    # 回调函数 (Callbacks)
    # =====================================================================

    def state_cb(self, msg):
        """飞控状态回调: 记录 armed / connected / mode 等关键字段。"""
        self.current_state = msg

    def pose_cb(self, msg):
        """本地位置回调: 记录 ENU 坐标系下当前三维位姿。"""
        self.current_pose = msg

    # =====================================================================
    # 工具方法
    # =====================================================================

    def _build_setpoint(self, x, y, z):
        """
        构造 PoseStamped 期望位置消息。

        Args:
            x, y, z (float): ENU 坐标系下的目标位置 (m)。

        Returns:
            PoseStamped: 可直接发布到 setpoint_position/local 的消息。
        """
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        # 姿态留空 —— PX4 位置控制器自行解算
        sp.pose.orientation.w = 1.0
        return sp

    def is_arrived(self, target):
        """
        欧氏距离判定是否到达目标点。

        Args:
            target (dict): {"x": float, "y": float, "z": float}

        Returns:
            bool: 当前位姿与目标点三维距离是否在容差范围内。
        """
        cp = self.current_pose.pose.position
        dx = cp.x - target["x"]
        dy = cp.y - target["y"]
        dz = cp.z - target["z"]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance < self.pos_tolerance

    def _check_failsafe(self):
        """
        检查飞控连接及模式状态，异常时抛出 MissionAbortException。

        Raises:
            MissionAbortException: 飞控断连或意外退出 OFFBOARD 模式。
        """
        if not self.current_state.connected:
            rospy.logerr("飞控断开连接！中止任务。")
            raise MissionAbortException("FCU disconnected")

        # OFFBOARD 阶段若被外部干预切走模式也应中止
        offboard_states = [
            "SET_OFFBOARD_ARM", "TAKEOFF", "GO_TO_A",
            "HOVER_A", "GO_TO_B",
        ]
        if (self.current_mission_state in offboard_states
                and self.current_state.mode != "OFFBOARD"):
            rospy.logerr(
                f"非预期模式切换: 当前 {self.current_state.mode}，"
                f"期望 OFFBOARD。中止任务。"
            )
            raise MissionAbortException("Unexpected mode change")

    # =====================================================================
    # 通信建立 & 安全切模式
    # =====================================================================

    def prepare_communication(self):
        """
        建立 OFFBOARD 通信通道并解锁电机。

        PX4 安全机制要求: 在切入 OFFBOARD 前必须已有高频 setpoint 流，
        否则飞控会拒绝接管。因此先连续发送 100 次当前位置点 (或零点)，
        然后再调用切模式与解锁服务。
        """
        rospy.loginfo("正在建立 OFFBOARD 通信通道 (预发 setpoint 流)...")
        pre_sp = self._build_setpoint(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z,
        )

        for i in range(100):
            pre_sp.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(pre_sp)
            self.rate.sleep()

        rospy.loginfo("预发 setpoint 完成，正在请求 OFFBOARD 模式...")

        # 切换到 OFFBOARD 模式
        mode_req = SetModeRequest(custom_mode="OFFBOARD")
        mode_res = self.set_mode_client.call(mode_req)
        if mode_res.mode_sent:
            rospy.loginfo("OFFBOARD 模式切换请求已发送。")
        else:
            rospy.logerr("OFFBOARD 模式切换失败！")
            return False

        rospy.sleep(1.0)  # 等待模式生效

        # 解锁电机
        arm_req = CommandBoolRequest(value=True)
        arm_res = self.arming_client.call(arm_req)
        if arm_res.success:
            rospy.loginfo("电机已解锁。")
        else:
            rospy.logerr("电机解锁失败！")
            return False

        return True

    # =====================================================================
    # 状态机主循环
    # =====================================================================

    def run_mission(self):
        """
        有限状态机主循环 —— 按顺序驱动送货任务。

        状态流转:
            INIT -> SET_OFFBOARD_ARM -> TAKEOFF -> GO_TO_A
            -> HOVER_A -> GO_TO_B -> AUTO_LAND -> DONE
        """
        rospy.loginfo("====== 送货任务启动 ======")

        while not rospy.is_shutdown():

            # ---- 全局安全检查 -------------------------------------------
            try:
                self._check_failsafe()
            except MissionAbortException as e:
                rospy.logerr(f"任务中止: {e}")
                return

            # ---- 状态机分发 ---------------------------------------------
            if self.current_mission_state == "INIT":
                self._handle_init()

            elif self.current_mission_state == "SET_OFFBOARD_ARM":
                self._handle_set_offboard_arm()

            elif self.current_mission_state == "TAKEOFF":
                self._handle_takeoff()

            elif self.current_mission_state == "GO_TO_A":
                self._handle_go_to_A()

            elif self.current_mission_state == "HOVER_A":
                self._handle_hover_A()

            elif self.current_mission_state == "GO_TO_B":
                self._handle_go_to_B()

            elif self.current_mission_state == "AUTO_LAND":
                self._handle_auto_land()
                break  # 进入自动降落后终止循环

            elif self.current_mission_state == "DONE":
                rospy.loginfo("任务完成，节点退出。")
                break

            # ---- 发布当前目标点 & 维持频率 -----------------------------
            if self.current_mission_state not in ("INIT", "AUTO_LAND", "DONE"):
                self.target_pose.header.stamp = rospy.Time.now()
                self.local_pos_pub.publish(self.target_pose)

            self.rate.sleep()

    # =====================================================================
    # 各状态处理函数
    # =====================================================================

    def _handle_init(self):
        """
        INIT 状态:
            1. 调用 prepare_communication 建立通道并解锁
            2. 若成功则转入 SET_OFFBOARD_ARM，否则转入 DONE
        """
        rospy.loginfo("[INIT] 初始化通信通道...")
        if self.prepare_communication():
            self.current_mission_state = "SET_OFFBOARD_ARM"
            rospy.loginfo("[INIT] -> SET_OFFBOARD_ARM")
        else:
            rospy.logerr("[INIT] 通信建立失败，终止任务。")
            self.current_mission_state = "DONE"

    def _handle_set_offboard_arm(self):
        """
        SET_OFFBOARD_ARM 状态:
            等待 OFFBOARD 模式确认 & 电机解锁确认，随后进入 TAKEOFF。
            将目标点设在当前位置以维持悬停过渡。
        """
        if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
            rospy.loginfo("[SET_OFFBOARD_ARM] OFFBOARD 模式已确认，电机已解锁。")
            # 将目标点初始化为当前位置 (XY=0, Z=takeoff_alt 由 TAKEOFF 接管)
            self.target_pose = self._build_setpoint(
                0.0, 0.0, self.takeoff_alt
            )
            self.current_mission_state = "TAKEOFF"
            rospy.loginfo("[SET_OFFBOARD_ARM] -> TAKEOFF")
        else:
            rospy.loginfo_throttle(
                2.0,
                f"[SET_OFFBOARD_ARM] 等待中... mode={self.current_state.mode}, "
                f"armed={self.current_state.armed}",
            )

    def _handle_takeoff(self):
        """
        TAKEOFF 状态:
            在原点 (0, 0) 垂直起飞至 takeoff_alt 高度。
            到达高度阈值后转入 GO_TO_A。
        """
        self.target_pose = self._build_setpoint(0.0, 0.0, self.takeoff_alt)

        if self.is_arrived({"x": 0.0, "y": 0.0, "z": self.takeoff_alt}):
            rospy.loginfo(
                f"[TAKEOFF] 到达目标高度 {self.takeoff_alt:.1f} m，前往取货点 A。"
            )
            self.current_mission_state = "GO_TO_A"
            rospy.loginfo("[TAKEOFF] -> GO_TO_A")

    def _handle_go_to_A(self):
        """
        GO_TO_A 状态:
            发布 A 点坐标，到达后转入 HOVER_A。

            (拓展接口): 可在此处添加前方相机 / 雷达避障检测逻辑。
        """
        self.target_pose = self._build_setpoint(
            self.point_A["x"], self.point_A["y"], self.point_A["z"]
        )

        if self.is_arrived(self.point_A):
            rospy.loginfo("[GO_TO_A] 已到达取货点 A，开始悬停等待。")
            self.hover_counter = 0
            self.current_mission_state = "HOVER_A"
            rospy.loginfo("[GO_TO_A] -> HOVER_A")

    def _handle_hover_A(self):
        """
        HOVER_A 状态:
            在 A 点上方悬停指定时长 (默认 5 秒)，模拟取货过程。

            (拓展接口): 可在此处触发下游相机识别降落标志，
                        或进行视觉精调 / 机械臂抓取。
        """
        self.target_pose = self._build_setpoint(
            self.point_A["x"], self.point_A["y"], self.point_A["z"]
        )

        self.hover_counter += 1
        if self.hover_counter >= self.hover_target_loops:
            rospy.loginfo(
                f"[HOVER_A] 悬停完成 ({self.hover_count_target:.0f} s)，"
                f"前往送货点 B。"
            )
            self.current_mission_state = "GO_TO_B"
            rospy.loginfo("[HOVER_A] -> GO_TO_B")
        else:
            remaining = self.hover_count_target - self.hover_counter / 20.0
            rospy.loginfo_throttle(
                1.0,
                f"[HOVER_A] 悬停中... 剩余 {remaining:.1f} s",
            )

    def _handle_go_to_B(self):
        """
        GO_TO_B 状态:
            发布 B 点坐标，到达后转入 AUTO_LAND。
        """
        self.target_pose = self._build_setpoint(
            self.point_B["x"], self.point_B["y"], self.point_B["z"]
        )

        if self.is_arrived(self.point_B):
            rospy.loginfo("[GO_TO_B] 已到达送货点 B，开始自动降落。")
            self.current_mission_state = "AUTO_LAND"
            rospy.loginfo("[GO_TO_B] -> AUTO_LAND")

    def _handle_auto_land(self):
        """
        AUTO_LAND 状态:
            停止发布位置 setpoint (PX4 超时会自动触发降落逻辑)，
            并显式调用 set_mode 服务切换到 AUTO.LAND 模式。
            飞控自动完成着陆检测与上锁，脚本随后退出。
        """
        rospy.loginfo("[AUTO_LAND] 发送 AUTO.LAND 模式切换请求...")
        land_req = SetModeRequest(custom_mode="AUTO.LAND")
        land_res = self.set_mode_client.call(land_req)
        if land_res.mode_sent:
            rospy.loginfo("[AUTO_LAND] AUTO.LAND 模式已发送，等待飞控自动着陆...")
        else:
            rospy.logerr("[AUTO_LAND] AUTO.LAND 模式切换失败！")

        self.current_mission_state = "DONE"


# =============================================================================
# 主入口
# =============================================================================

if __name__ == "__main__":
    try:
        drone = PX4DeliveryDrone()
        drone.run_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被 ROS 中断，安全退出。")
    except Exception as e:
        rospy.logerr(f"未捕获异常: {e}")
