"""
MuJoCo 模型调试工具箱
=====================
用法:  python3 mujoco_debug.py
       python3 -i mujoco_debug.py  # 交互模式，跑完后可以继续输命令

依赖:  pip install mujoco
"""

import mujoco
import numpy as np

# ============================================================
# 1. 加载模型
# ============================================================
def load_model(xml_path="geniarm_v1.1.2.xml"):
    """加载 MuJoCo 模型，返回 (model, data)"""
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    return m, d

m, d = load_model()

print("=== 模型基本信息 ===")
print(f"  DOF 数量 (nv):     {m.nv}")
print(f"  关节数量 (nq):     {m.nq}  (含 quaternion 的维度)")
print(f"  body 数量:         {m.nbody}")
print(f"  geom 数量:         {m.ngeom}")
print(f"  执行器数量:        {m.nu}")
print(f"  接触点数量(当前):  {d.ncon}")


# ============================================================
# 2. 查看关节信息
# ============================================================
def print_joint_info(m, d):
    """打印所有关节的名称、类型、限位、当前值"""
    print("\n=== 关节信息 ===")
    # m.jnt_qposadr: 每个关节在 qpos 数组中的起始索引
    for i in range(m.njnt):
        name = m.joint(i).name
        jnt_type = ["FREE", "BALL", "SLIDE", "HINGE"][m.jnt_type[i]]
        qpos_adr = m.jnt_qposadr[i]          # 在 qpos 中的位置
        qpos_val = d.qpos[qpos_adr]           # 当前值
        if m.jnt_limited[i]:                   # 是否有限位
            lo, hi = m.jnt_range[i]            # 限位范围
            range_str = f"[{lo:.4f}, {hi:.4f}]"
        else:
            range_str = "无限制"
        print(f"  [{i}] {name:25s}  type={jnt_type:6s}  "
              f"range={range_str:16s}  qpos={qpos_val:.4f}")

print_joint_info(m, d)


# ============================================================
# 3. 查看执行器信息
# ============================================================
def print_actuator_info(m):
    """打印所有执行器的名称、关联关节、控制参数"""
    print("\n=== 执行器信息 ===")
    for i in range(m.nu):
        name = m.actuator(i).name
        joint_name = m.joint(m.actuator_trnid[i, 0]).name  # 关联的关节
        ctrl_lo, ctrl_hi = m.actuator_ctrlrange[i]
        force_lo, force_hi = m.actuator_forcerange[i]
        kp = m.actuator_gainprm[i, 0]
        kv = -m.actuator_biasprm[i, 2]   # kv 存在 biasprm[2] 中
        print(f"  [{i}] {name:30s}  joint={joint_name:20s}  "
              f"ctrl=[{ctrl_lo:.3f},{ctrl_hi:.3f}]  "
              f"force=[{force_lo:.1f},{force_hi:.1f}]  "
              f"kp={kp:.0f}  kv={kv:.0f}")

print_actuator_info(m)


# ============================================================
# 4. 设置控制并步进仿真
# ============================================================
def step_and_print(m, d, steps=1000, label=""):
    """
    步进仿真并打印状态
    
    参数:
        m, d: 模型和数据
        steps: 步数 (timestep=0.001s, 所以1000步=1秒)
        label: 输出标签
    """
    for _ in range(steps):
        mujoco.mj_step(m, d)
    
    if label:
        print(f"\n=== {label} (after {steps} steps = {steps*0.001:.1f}s) ===")
    print(f"  qpos: {np.array2string(d.qpos, precision=6, suppress_small=True)}")
    print(f"  qvel: {np.array2string(d.qvel, precision=6, suppress_small=True)}")
    print(f"  接触数: {d.ncon}")


# ============================================================
# 5. 查看力/力矩分解
# ============================================================
def print_force_breakdown(m, d, dof_range=None):
    """
    查看各力分量
    
    qfrc_actuator  : 执行器输出力
    qfrc_bias      : 重力 + 科里奥利 + 离心力
    qfrc_passive   : 被动阻尼/摩擦
    qfrc_constraint: 约束力 (碰撞、关节限位、equality)
    qfrc_gravcomp  : 重力补偿 (未启用时为0)
    """
    mujoco.mj_forward(m, d)  # 重新计算力 (mj_step也会算，但mj_forward只算力不积分)
    
    if dof_range is None:
        dof_range = range(min(6, m.nv))  # 默认只看前6个DOF
    
    print("\n=== 力分量分解 ===")
    print(f"{'DOF':>4s}  {'执行器':>10s}  {'重力/惯性':>10s}  "
          f"{'被动':>10s}  {'约束':>10s}  {'净力':>10s}  {'加速度':>10s}")
    for i in dof_range:
        act = d.qfrc_actuator[i]
        bias = d.qfrc_bias[i]
        pas = d.qfrc_passive[i]
        con = d.qfrc_constraint[i]
        net = act + bias + pas + con
        acc = d.qacc[i]
        print(f"  {i:2d}  {act:10.4f}  {bias:10.4f}  {pas:10.4f}  "
              f"{con:10.4f}  {net:10.4f}  {acc:10.4f}")


# ============================================================
# 6. 查看接触信息
# ============================================================
def print_contacts(m, d):
    """打印所有接触点的详细信息"""
    mujoco.mj_forward(m, d)
    print(f"\n=== 接触信息 (共 {d.ncon} 个) ===")
    for i in range(d.ncon):
        c = d.contact[i]
        g1_name = m.geom(c.geom1).name or f"unnamed_{c.geom1}"
        g2_name = m.geom(c.geom2).name or f"unnamed_{c.geom2}"
        body1 = m.body(m.geom_bodyid[c.geom1]).name
        body2 = m.body(m.geom_bodyid[c.geom2]).name
        print(f"  [{i}] {body1:20s}({g1_name})  <->  "
              f"{body2:20s}({g2_name})")
        print(f"       穿透深度 dist={c.dist:.6f}m  位置={np.round(c.pos, 6)}")
        print(f"       法向={np.round(c.frame[:3], 6)}")


# ============================================================
# 7. 执行器阶跃响应测试
# ============================================================
def step_response_test(m, d, joint_indices, targets, steps=2000):
    """
    阶跃响应测试: 对指定关节设置目标位置，观察跟踪效果
    
    参数:
        m, d: 模型和数据
        joint_indices: 要控制的关节索引列表
        targets: 目标位置列表 (与 joint_indices 一一对应)
        steps: 每个目标步进的步数
    """
    print(f"\n{'='*60}")
    print("阶跃响应测试")
    print(f"{'='*60}")
    
    # 重置到零位
    d.qpos[:] = 0
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    
    for idx, target in zip(joint_indices, targets):
        d.ctrl[idx] = target
        joint_name = m.joint(m.actuator_trnid[idx, 0]).name
        
        # 采样: 初始 + 中间几个点 + 最终
        sample_steps = [0, int(steps*0.05), int(steps*0.1), 
                       int(steps*0.2), int(steps*0.5), steps]
        sample_steps = sorted(set(sample_steps))
        
        print(f"\n  关节 {joint_name:20s}  目标={target:.4f}")
        last_step = 0
        for s in sample_steps:
            for _ in range(s - last_step):
                mujoco.mj_step(m, d)
            last_step = s
            pos = d.qpos[m.jnt_qposadr[m.actuator_trnid[idx, 0]]]
            err = target - pos
            force = d.qfrc_actuator[idx]
            print(f"    step={s:5d} ({s*0.001:.3f}s):  "
                  f"pos={pos:.6f}  err={err:.6f}  force={force:.4f}")
        
        # 再步进一些看稳态
        for _ in range(steps - last_step):
            mujoco.mj_step(m, d)
        pos = d.qpos[m.jnt_qposadr[m.actuator_trnid[idx, 0]]]
        err = target - pos
        print(f"    └─ 稳态: pos={pos:.6f}  err={err:.6f}  "
              f"({abs(err/(target+1e-10))*100:.2f}%)")


# ============================================================
# 8. 修改参数后对比测试
# ============================================================
def compare_actuator_params(xml_path, actuator_idx, joint_idx, 
                            param_sets, ctrl_value, steps=2000):
    """
    对比不同执行器参数的控制效果
    
    参数:
        xml_path: 模型文件路径
        actuator_idx: 执行器索引
        joint_idx: 关节在 qpos 中的索引
        param_sets: 参数列表, 每项为 (label, kp, kv)
        ctrl_value: 控制目标值
        steps: 步进数
    """
    print(f"\n{'='*60}")
    print("参数对比测试")
    print(f"{'='*60}")
    
    for label, kp, kv in param_sets:
        # 每个参数集重新加载模型
        m2 = mujoco.MjModel.from_xml_path(xml_path)
        d2 = mujoco.MjData(m2)
        
        # 修改参数
        m2.actuator_gainprm[actuator_idx, 0] = kp
        m2.actuator_biasprm[actuator_idx, 2] = -kv
        
        # 施加控制
        d2.ctrl[actuator_idx] = ctrl_value
        
        # 采样记录
        positions = []
        for s in range(steps):
            mujoco.mj_step(m2, d2)
            if s % 100 == 0:
                positions.append((s, d2.qpos[joint_idx].copy()))
        
        final_pos = d2.qpos[joint_idx].copy()
        final_err = abs(ctrl_value - final_pos)
        print(f"\n  [{label}] kp={kp:6.0f}  kv={kv:6.1f}")
        print(f"    最终 pos={final_pos:.6f}  err={final_err:.6f}")
        for s, pos in positions:
            print(f"    step={s:5d}: pos={pos:.6f}")


# ============================================================
# 9. Python 一行速查
# ============================================================
def quick_help():
    """MuJoCo 常用调试命令速查"""
    print("""
========== MuJoCo Python 速查 ==========

【加载】
  m = mujoco.MjModel.from_xml_path("model.xml")
  d = mujoco.MjData(m)

【步进】
  mujoco.mj_step(m, d)           # 步进1步
  mujoco.mj_forward(m, d)        # 只算力，不积分 (用于查看当前状态)

【读状态】
  d.qpos       # 关节位置
  d.qvel       # 关节速度
  d.ctrl       # 控制输入 (可读写)
  d.time       # 仿真时间

【读力】
  d.qfrc_actuator    # 执行器力
  d.qfrc_bias        # 重力+科里奥利
  d.qfrc_passive     # 阻尼/摩擦
  d.qfrc_constraint  # 约束力
  d.qacc             # 加速度

【写控制】
  d.ctrl[0] = 1.57    # 设置第1个执行器的目标

【关节信息】
  m.jnt_type[i]       # 关节类型 (0=free, 1=ball, 2=slide, 3=hinge)
  m.jnt_range[i]      # 关节限位 [lo, hi]
  m.jnt_qposadr[i]    # 关节在 qpos 中的起始索引

【执行器信息】
  m.actuator_trnid[i]  # 执行器关联的 (joint_id, ...)
  m.actuator_ctrlrange[i]
  m.actuator_forcerange[i]
  m.actuator_gainprm[i, 0]  = kp
  m.actuator_biasprm[i, 2]  = -kv

【碰撞】
  d.ncon           # 接触点数量
  d.contact[i]     # 第i个接触
  
【交互模式】
  python3 -i mujoco_debug.py
  # 然后可以输任何命令继续调试
""")


# ============================================================
# 10. 实用小工具
# ============================================================
def reset(m, d):
    """重置仿真到初始状态"""
    d.qpos[:] = 0
    d.qvel[:] = 0
    d.ctrl[:] = 0
    mujoco.mj_forward(m, d)
    print("已重置到零位")

def run_until_stable(m, d, max_steps=10000, tol=1e-6):
    """步进到关节速度趋近于0 (稳态)"""
    for i in range(max_steps):
        mujoco.mj_step(m, d)
        if np.max(np.abs(d.qvel)) < tol:
            print(f"  在 step={i} 达到稳态, qvel<{tol}")
            return i
    print(f"  未能在 {max_steps} 步内达到稳态")
    return max_steps

def set_and_run(m, d, ctrl_values, steps=1000):
    """设置控制并运行"""
    d.ctrl[:len(ctrl_values)] = ctrl_values
    step_and_print(m, d, steps, f"ctrl={ctrl_values}")


# ============================================================
# 使用示例 (取消注释即可运行)
# ============================================================
if __name__ == "__main__":
    print("\n\n========== 使用示例 ==========")
    
    # ---- 例1: 驱动 J1 到 1.0 rad ----
    m, d = load_model()
    d.ctrl[0] = 1.0  # J1 目标位置
    step_and_print(m, d, 2000, "J1 → 1.0 rad")
    
    # ---- 例2: 查看力分解 ----
    print_force_breakdown(m, d, range(3))
    
    # ---- 例3: 查看接触 ----
    print_contacts(m, d)
    
    # ---- 例4: 夹爪阶跃测试 ----
    reset(m, d)
    step_response_test(m, d, [6], [0.045], steps=1000)
    
    # ---- 打印速查表 ----
    quick_help()
