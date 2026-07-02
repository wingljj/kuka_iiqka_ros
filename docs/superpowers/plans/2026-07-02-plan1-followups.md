# Plan 1 终审跟进事项(Plan 2~6 输入)

来源:feature/soft-force-control-core 整分支终审(2026-07-02,结论 APPROVE,52/52 测试通过、零警告)。

## Plan 3(soft_robot_controllers)必须处理

1. **AutoReTare 边沿触发(重要)**:`shouldTare` 是无状态谓词,机器人停在参考姿态且残差小于死区时条件每周期恒真;控制器若每周期调用 `absorbResidual` 会连续吸收噪声导致偏置漂移。必须实现单次触发(tare 后置冷却/挂起标志,离开参考姿态后复位)。
2. **修正合成顺序**:FORCE_COMPLIANCE 与 orientation motion 两路 `CartesianCorrection` 先求和、再进 `SafetyLimiter`,per-cycle clamp 才对合成量生效。写入控制器 update() 伪代码。
3. **滤波带宽随 profile 切换**:`ForceTorqueFilter` 的 cutoff 仅构造期可设;DRAG/PRECISION 需不同带宽时重建对象并 `reset()`,或扩展 `setCutoff()`(扩展须连带 reset 语义)。
4. **硬截断为严格大于语义**:wrench 恰等于 500N/50Nm 不触发;若安全评审要求 ≥,需显式调整并写入参数文档。
5. **备忘**:`MotionStatus::INACTIVE` 不区分"未启动"与"被 cancel",action server 回报 PREEMPTED 需上层记账;`OrientationMotionCore` 逐轴 Euler wrap 误差在 B≈±90° 时与测地误差偏离,当前小角度回正用途可接受。

## Plan 5/6(标定流程与 UI)

6. **PayloadEstimator 无秩/条件数检查**:姿态样本退化时 QR 仍给有限解,质量仅由 r2_force/r2_torque 反映;标定流程须强制姿态多样性并以 R² 阈值把关(对齐规格 §9 fit-quality index)。

## 低优先

7. **M_PI 非 ISO 标准**:`rotation.h`、`force_torque_filter.cpp` 依赖 glibc 扩展;若切换编译器/严格模式,改为 `constexpr double kPi`。
