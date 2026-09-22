"""mock_nav2：离线回归用的假 Nav2 NavigateToPose 动作服务器。

仅用于仿真/离线回归，不接真实 Nav2。行为可脚本化（按第 N 个目标顺序生效，不足时重复
最后一个）：
- success  接受后 delay 秒 succeed
- abort    接受后 delay 秒 abort（模拟不可达）
- reject   目标一到达即 REJECT（accept 阶段拒绝）
- never    接受后永不主动完成（配合超时/取消/定位失效测试），直到被取消或 stop()
- cancel_stall   接受取消但永不置终态（模拟“取消已接受但终态迟迟不返回”，客户端 cancel 超时→fault）
- cancel_externally  接受后 delay 秒自行 CANCELED（模拟外部实体取消，客户端未主动请求）
- die      接受后立刻销毁 action server（模拟动作服务中途消失；客户端 result/cancel
            未来挂起、结局未知，走 per-goal timeout → fault 兜底）

rclpy Humble 语义：NavigateToPose 的 result 是 std_msgs/Empty，成功/失败/取消通过
GoalStatus 表达，且终端状态（succeed/abort/canceled）必须发生在 EXECUTING 之后，因此
采用「execute 回调」在后台等待 delay 后置终态（阻塞在 execute 回调里，跑在线程池），
取消在 execute 回调里轮询 ``goal_handle.is_cancel_requested`` 处理。

参数：
- action              动作名，默认 /navigate_to_pose
- behavior            逗号分隔行为序列，默认 'success'
- delay               完成前等待秒数，默认 0.5
- accept_delay        accept 阶段前额外等待秒数（用于制造“等待接受期间”窗口），默认 0.0
- reject_cancel       取消回调返回 REJECT（用于测取消被拒 -> 故障），默认 False
- no_cancel_response  取消回调收到取消请求后不回 ACCEPT/REJECT（模拟“取消请求无响应”，
                      客户端 cancel_async 未来永不完成，只能靠 cancel_timeout 兜底），默认 False
- recoveries          反馈中报告的恢复次数，默认 0
- pose_error          反馈位姿相对目标的偏移(米)，用于测位置误差，默认 0.0
- shutdown_after      服务 N 个目标后自行退出（0=永不），默认 0
- localization_topic  若非空，按 localization_rate 发布 Bool True（模拟定位健康），默认 ''
- localization_rate   定位发布频率 Hz，默认 10.0

服务端事件跟踪：``self.events`` 按发生顺序记录 ('event', goal#) 元组，供回归断言
“取消请求确实到达服务端、目标确实被服务端置为终态”，而不只看任务器本地报告状态。
事件：goal_received / accepted / rejected / cancel_received / succeeded / aborted /
canceled / died。
"""
import threading
import time

import rclpy
import rclpy._rclpy_pybind11 as _rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, CancelResponse
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


class MockNav2(Node):
    def __init__(self, overrides=None):
        super().__init__('mock_nav2')
        self.declare_parameter('action', '/navigate_to_pose')
        self.declare_parameter('behavior', 'success')
        self.declare_parameter('delay', 0.5)
        self.declare_parameter('accept_delay', 0.0)
        self.declare_parameter('reject_cancel', False)
        self.declare_parameter('no_cancel_response', False)
        self.declare_parameter('recoveries', 0)
        self.declare_parameter('pose_error', 0.0)
        self.declare_parameter('shutdown_after', 0)
        self.declare_parameter('localization_topic', '')
        self.declare_parameter('localization_rate', 10.0)
        self.declare_parameter('events_file', '')

        if overrides:
            from rclpy.parameter import Parameter
            self.set_parameters([
                Parameter(k, value=v) for k, v in overrides.items()])

        self._action = self.get_parameter('action').value
        self._behavior = [b.strip() for b in
                          self.get_parameter('behavior').value.split(',') if b.strip()]
        self._delay = float(self.get_parameter('delay').value)
        self._accept_delay = float(self.get_parameter('accept_delay').value)
        self._reject_cancel = bool(self.get_parameter('reject_cancel').value)
        self._no_cancel_response = bool(self.get_parameter('no_cancel_response').value)
        self._recoveries = int(self.get_parameter('recoveries').value)
        self._pose_error = float(self.get_parameter('pose_error').value)
        self._shutdown_after = int(self.get_parameter('shutdown_after').value)
        self._loc_topic = self.get_parameter('localization_topic').value or ''
        self._loc_rate = float(self.get_parameter('localization_rate').value)
        self._events_file = self.get_parameter('events_file').value or ''

        self._count = 0
        self._pending_behavior = 'success'
        self.events = []                 # 服务端事件跟踪（回归断言用）
        self._stop = threading.Event()

        self._server = ActionServer(
            self, NavigateToPose, self._action,
            self._execute_cb,
            goal_callback=self._goal_cb,
            handle_accepted_callback=self._accepted_cb,
            cancel_callback=self._cancel_cb,
        )

        self._loc_pub = None
        if self._loc_topic:
            self._loc_pub = self.create_publisher(Bool, self._loc_topic, 10)
            self.create_timer(1.0 / self._loc_rate, self._publish_loc)

        self.get_logger().info('mock_nav2 ready: action=%s behavior=%s' %
                               (self._action, self._behavior))

    def stop(self):
        """请求后台执行回调尽快退出（配合 never 行为 + 拒绝取消等场景的清理）。"""
        self._stop.set()

    def _record(self, event):
        self.events.append((event, self._count))
        # events_file：把服务端事件即时落盘，供独立进程的退出/信号测试观察“取消请求
        # 确实到达服务端、目标确实被服务端置为终态”（本进程 events 列表他人读不到）。
        if self._events_file:
            import json
            try:
                with open(self._events_file, 'w', encoding='utf-8') as f:
                    json.dump(self.events, f)
            except Exception:
                pass

    # ---- 行为选择 ----
    def _behavior_for(self):
        if not self._behavior:
            return 'success'
        return self._behavior[min(self._count, len(self._behavior) - 1)]

    # ---- 动作服务器回调 ----
    def _goal_cb(self, goal_request):
        b = self._behavior_for()
        self._count += 1
        self._pending_behavior = b
        # accept_delay：在返回 ACCEPT 前阻塞，制造客户端“等待接受期间”窗口。
        if self._accept_delay > 0:
            time.sleep(self._accept_delay)
        self._record('goal_received')
        self.get_logger().info('mock: goal #%d behavior=%s' % (self._count, b))
        if b == 'reject':
            self._record('rejected')
            return GoalResponse.REJECT
        self._record('accepted')
        return GoalResponse.ACCEPT

    def _accepted_cb(self, goal_handle):
        self._publish_feedback(goal_handle)
        if self._shutdown_after and self._count >= self._shutdown_after:
            self.get_logger().warn('mock: served %d goals, shutting down' % self._count)
            self.destroy_node()
            return
        # 进入 EXECUTING；置终态在 _execute_cb 里异步完成
        goal_handle.execute()

    def _execute_cb(self, goal_handle):
        # 同步阻塞在 execute 回调里（MultiThreadedExecutor 跑在线程池，阻塞单线程无碍），
        # 用 time.sleep 模拟导航耗时并轮询取消。
        b = self._pending_behavior
        if b == 'never':
            # 永不主动完成，直到被取消或外部 stop()。
            while not goal_handle.is_cancel_requested and not self._stop.is_set():
                time.sleep(0.05)
            if goal_handle.is_cancel_requested:
                self._record('canceled')
                goal_handle.canceled()
            else:
                self._record('aborted')
                goal_handle.abort()
            return NavigateToPose.Result()

        if b == 'cancel_stall':
            # 接受取消但永不置终态（_cancel_cb 已返回 ACCEPT），模拟“取消已接受但终态
            # 迟迟不返回”；直到外部 stop() 才兜底置 aborted，避免清理挂起。
            while not self._stop.is_set():
                time.sleep(0.05)
            self._record('aborted')
            goal_handle.abort()
            return NavigateToPose.Result()

        if b == 'cancel_externally':
            # 外部实体取消：客户端未主动请求，服务端收到 cancel request 后自行置 CANCELED。
            # rclpy 状态机不允许 EXECUTING 直接跳 CANCELED（会抛 invalid transition），
            # 必须先经 CANCEL_GOAL 事件进入 CANCELING（等价于 _execute_cancel_request 的
            # 处理），再置 CANCELED。
            time.sleep(self._delay)
            self._record('canceled')
            goal_handle._update_state(_rclpy.GoalEvent.CANCEL_GOAL)
            goal_handle.canceled()
            return NavigateToPose.Result()

        if b == 'die':
            # 动作服务执行中消失：直接销毁 action server（而非 destroy_node 干净退出）。
            # destroy_node 会先干净地 abort 在途目标（客户端收到 ABORTED，目标确实已结束）；
            # 而销毁 action server 会让客户端 result/cancel 未来挂起（结局未知），真正模拟
            # “服务中途消失”后客户端无法确认结局的情形，走 per-goal timeout → fault 兜底。
            self._record('died')
            self.get_logger().warn('mock: goal #%d server disappears' % self._count)
            self._server.destroy()
            return NavigateToPose.Result()

        time.sleep(self._delay)
        if goal_handle.is_cancel_requested:
            self._record('canceled')
            goal_handle.canceled()
            return NavigateToPose.Result()

        if b == 'success':
            self._record('succeeded')
            goal_handle.succeed()
        else:  # abort
            self._record('aborted')
            goal_handle.abort()
        return NavigateToPose.Result()

    def _cancel_cb(self, goal_handle):
        self._record('cancel_received')
        self.get_logger().info('mock: cancel received')
        if self._reject_cancel:
            return CancelResponse.REJECT
        if self._no_cancel_response:
            # 模拟“取消请求无响应”：收到取消请求但不回 ACCEPT/REJECT，客户端 cancel_async
            # 未来永不完成，只能靠 cancel_timeout 兜底进入 fault。stop() 时解除阻塞，
            # 避免清理挂起（此时测试已取走 events，不会污染断言）。
            while not self._stop.is_set():
                time.sleep(0.05)
            return CancelResponse.ACCEPT
        return CancelResponse.ACCEPT

    # ---- 反馈 ----
    def _publish_feedback(self, goal_handle):
        fb = NavigateToPose.Feedback()
        req = goal_handle.request
        fb.current_pose.header.frame_id = req.pose.header.frame_id
        fb.current_pose.header.stamp = self.get_clock().now().to_msg()
        fb.current_pose.pose.position.x = req.pose.pose.position.x + self._pose_error
        fb.current_pose.pose.position.y = req.pose.pose.position.y
        fb.current_pose.pose.orientation = req.pose.pose.orientation
        fb.navigation_time.sec = 0
        fb.navigation_time.nanosec = 0
        fb.number_of_recoveries = self._recoveries
        fb.distance_remaining = 0.0
        goal_handle.publish_feedback(fb)

    def _publish_loc(self):
        if self._loc_pub is not None:
            self._loc_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = MockNav2()
    # 必须用多线程 executor：never/cancel_stall 等行为的 execute 回调会阻塞等待取消，
    # 单线程全局 executor 会因此无法再处理取消服务请求（取消永远到不了服务端）。
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
