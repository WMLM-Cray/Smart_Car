# 临时精简版

分支：codex/temporary-pruned-version。不合并。
完整整理版：66814ff；整理前 tag：before-directory-cleanup-20260926。

删除清单见 PRUNED_PATHS.txt。移除旧监视台、旧双输出策略及导出工具、多速度实验、重复地图预览、重建中间稿和雷达示例录包。
Gazebo 启动脚本改用现有低速转向策略 candidate_060，默认 0.6 m/s；未进行 Gazebo 动态验证。

保留当前 road 训练、low_speed 实车策略及对照、共享仿真和测试、Gazebo/导航工具、标定证据、厂家备份、训练断点。PPO 入口仍被 preview 和共享回归测试使用，保留依赖。
本地 work/tmp 属于忽略或未提交文件，另行备份后移出工作区。
