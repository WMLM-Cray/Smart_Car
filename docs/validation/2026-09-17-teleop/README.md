# 手柄包判据3b补丁+油门回中门复验证据(2026-09-17)

## 背景

主线任务书(2026-09-17)第二节要求对队友手柄修复版按六条判据独立复验,其中判据3:
"超时后按住使能的新消息**先于 on_tick 到达**,仍输出零"。

代码考古结论:`origin/racecar-teleop-standalone@ce0bdc9` 的 `state.py` 与旧提交
`01a3374` **逐字相同**——队友三轮修复只改了 launch autorepeat 与文档,状态机未动
(其任务红线即"不改 state.py")。若判据3字面场景在状态机层面成立,则该漏洞
**至今仍在**,与"01a3374 已确认漏洞"的既有记录一致。

## 漏洞机理(黑盒复现)

- 断流 0.99s(>input_timeout 0.3s)期间**一次 on_tick 都未执行**(事件循环阻塞/系统挂起的等效时序);
- 恢复的"仍按住使能"消息到达:`on_joy` 无条件刷新 `_last_input_time`,`_armed` 保持 True;
- 下一个 `on_tick`:时间间隔 0.01s < 0.3s,不判超时;`_armed=True` → **输出 0.5 m/s 继续运动**;
- 全程节点从未观察到超时 → "漏掉超时撤销使能"。

正常 20Hz tick 运行时不触发(tick 在断流 0.3s 后即锁定,判据 3a 覆盖),因此队友的
e2e(注入与 tick 同事件循环)与 46 单测均未暴露。

## 补丁(1 处,`racecar_teleop/state.py` 的 `on_joy`)

在刷新时间戳**之前**检查相邻 Joy 消息间隔:

```python
if self._last_input_time is not None and \
        (now - self._last_input_time) > self.params.input_timeout:
    self._timeout_occurred = True
    self._armed = False        # 视为已观察超时:锁定,恢复需松开+重按使能
```

效果:断流 >0.3s 后,恢复消息自身触发锁定(不依赖 tick 是否跑过);
joy 20Hz 正常流(间隔 0.05s)行为不变。速度→PWM 转换位置、话题、其余逻辑零改动。

## 证据文件

| 文件 | 内容 | 退出码 |
|---|---|---|
| `verify_task_spec_原版对照.txt` | ce0bdc9 原版:8/9,**仅 3b FAIL**(输出 0.5 m/s, armed=True) | 1 |
| `verify_task_spec_补丁后.txt` | 补丁后:**9/9 全 PASS** | 0 |
| `pytest_46_补丁后.txt` | 队友 46 单测补丁后回归:全过 | 0 |
| `verify_task_spec_油门门.txt` | 油门回中门(判据7,2026-09-17 晚追加):**10/10 全 PASS** | 0 |
| `pytest_49_油门门.txt` | 适配后 46+**新增 3**(首帧满杆锁/推杆重按门/倒车向门):**49 全过** | 0 |
| `dryrun_chain_S7油门门.txt` | 全链 S1-S8(S7 改述=油门门实链):**8/8 PASS**(WSL) | 0 |

## 油门回中门(判据7,2026-09-17 晚,主线追加需求)

用户场景:重连/超时锁定恢复时,操作者**仍推着油门摇杆**,此时按"松开重按使能"
流程一按使能,车会以当前杆偏转突然起步。修复=在"松开重按使能"之外,另要求
**观测到一帧油门轴在死区内**(取绝对值,倒车向同样受门;转向轴不受门——仅转向
无纵向运动)才允许杆值生效。门在启动/超时/消息间隔超时/非法输入任一锁定时重置。

- 实现:`state.py` 新增 `_throttle_centered` 标志,`_lock()` 统一重置双门;
  `on_joy` 观测回死区即置位;`on_tick` 输出闸门 `not armed or not centered → (0,0)`。
- 适配:复验器 arm 帧/全链 S7 解锁帧统一改"松使能+杆回中"同帧(真实松手动作);
  T4/T10 检查表操作步骤同步(不先回中而直接重按使能=预期恒零,非故障)。

## 复现命令(仓库根)

```
# 补丁后(当前 HEAD)
python tools/teleop/verify_teleop_task_spec.py
# 原版对照(从 8384d8b 检出原包到任意目录后)
git show 8384d8b:hwcart_sim/src/racecar_teleop > /tmp/orig_pkg_dir(或 git worktree)
python tools/teleop/verify_teleop_task_spec.py <原包目录>
```

## 边界(如实)

- 本复验为纯逻辑层(状态机)黑盒;`teleop_node.py` 时钟接线为代码审查结论
  (两处 `time.monotonic()` 同源、未启用 use_sim_time),未做进程级验证;
- 判据6 的行为验证在纯逻辑层完成(只看时间差);"仿真暂停不冻结"依赖节点不用
  sim time 的审查结论;
- 实体手柄、joy_node 真硬件、物理断开/重连行为:**仍未实测**(与队友交接说明边界一致)。
