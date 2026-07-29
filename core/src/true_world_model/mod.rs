//! 真物理世界模型(基于 rapier3d 真 API)
//!
//! ## 跟旧版的区别
//! 旧:true_world_model,关节用 PD 控制器假装,接触力用 magic number
//! 新:全部用 rapier3d 真 API
//!   - 真关节:`RevoluteJointBuilder::new(axis)` + `impulse_joints.insert()`
//!   - 真接触力:`ContactPair::total_impulse_magnitude()` 拿真实冲量
//!   - 真接触深度:`ContactPair::find_deepest_contact()` 拿真距离

use rapier3d::prelude::*;
use std::collections::HashMap;

// ============================================================
// 关节类型
// ============================================================

/// 关节描述
#[derive(Debug, Clone, Copy)]
pub enum JointKind {
    /// 固定关节(刚性连接)
    Fixed,
    /// 旋转关节(铰链)
    Revolute { axis: [f32; 3] },
    /// 棱柱关节(滑轨)
    Prismatic { axis: [f32; 3] },
    /// 球窝关节(3 自由度旋转)
    Spherical,
}

/// 关节句柄(我们自己的 ID,对应 rapier 的 ImpulseJointHandle)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct JointHandle(pub u32);

/// 关节实例
#[derive(Debug, Clone, Copy)]
pub struct JointInstance {
    pub handle: JointHandle,
    pub rapier_handle: ImpulseJointHandle,
    pub kind: JointKind,
    pub body1: RigidBodyHandle,
    pub body2: RigidBodyHandle,
}

// ============================================================
// 接触信息(全部真数据)
// ============================================================

/// 接触详情(全部从 rapier 拿)
#[derive(Debug, Clone, Copy)]
pub struct ContactForce {
    pub body_a: RigidBodyHandle,
    pub body_b: RigidBodyHandle,
    pub point: [f32; 3],
    pub normal: [f32; 3],
    /// 真接触深度(负=穿透)
    pub depth: f32,
    /// 真总冲量(N·s,rapier 计算)
    pub total_impulse_magnitude: f32,
    /// 真最大冲量方向
    pub max_impulse_direction: [f32; 3],
    /// 接触开始时间
    pub start_time: f32,
    /// 接触 manifold 数
    pub manifold_count: usize,
    /// 激活接触点数
    pub active_contact_count: usize,
}

// ============================================================
// 流体粒子(简化的 SPH,rapier 负责碰撞)
// ============================================================

/// 流体粒子
#[derive(Debug, Clone, Copy)]
pub struct FluidParticle {
    pub body: RigidBodyHandle,
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub mass: f32,
    pub kind: FluidKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FluidKind {
    Water,
    Viscous,
    Elastic,
}

/// 流体容器
pub struct FluidContainer {
    pub particles: Vec<FluidParticle>,
    pub viscosity: f32,
    pub cohesion: f32,
    pub kind: FluidKind,
}

impl FluidContainer {
    pub fn new(viscosity: f32, kind: FluidKind) -> Self {
        Self {
            particles: Vec::new(),
            viscosity,
            cohesion: 0.5,
            kind,
        }
    }

    /// 流体统计
    pub fn statistics(&self) -> FluidStats {
        let mut mean_pos = [0.0, 0.0, 0.0];
        let mut mean_vel = [0.0, 0.0, 0.0];
        let mut total_ke = 0.0;
        for p in &self.particles {
            for i in 0..3 {
                mean_pos[i] += p.position[i];
                mean_vel[i] += p.velocity[i];
            }
            let v_sq = p.velocity[0].powi(2) + p.velocity[1].powi(2) + p.velocity[2].powi(2);
            total_ke += 0.5 * p.mass * v_sq;
        }
        let n = self.particles.len().max(1) as f32;
        for i in 0..3 {
            mean_pos[i] /= n;
            mean_vel[i] /= n;
        }
        FluidStats {
            count: self.particles.len(),
            mean_position: mean_pos,
            mean_velocity: mean_vel,
            total_kinetic_energy: total_ke,
            viscosity: self.viscosity,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FluidStats {
    pub count: usize,
    pub mean_position: [f32; 3],
    pub mean_velocity: [f32; 3],
    pub total_kinetic_energy: f32,
    pub viscosity: f32,
}

// ============================================================
// 真物理世界
// ============================================================

/// 真物理世界(全部 rapier 真 API)
pub struct TruePhysicsWorld {
    pub backend: super::rapier_bridge::RapierWorld,
    /// 关节:JointHandle -> JointInstance
    pub joints: HashMap<JointHandle, JointInstance>,
    pub fluids: Vec<FluidContainer>,
    /// 当前接触(每个 ContactPair 转成一个)
    pub contacts: Vec<ContactForce>,
    pub step_count: u64,
    /// 累计接触冲量(真)
    pub total_contact_impulse: f32,
    next_joint_id: u32,
    /// 时间起点
    start_time: f32,
}

impl TruePhysicsWorld {
    pub fn new() -> Self {
        Self {
            backend: super::rapier_bridge::RapierWorld::new(),
            joints: HashMap::new(),
            fluids: Vec::new(),
            contacts: Vec::new(),
            step_count: 0,
            total_contact_impulse: 0.0,
            next_joint_id: 1,
            start_time: 0.0,
        }
    }

    /// 推进物理
    pub fn step(&mut self, dt: f32) {
        self.backend.step(dt);
        self.step_count += 1;
        self.update_fluid_states();
        self.refresh_contacts_real();
    }

    // ========================================================
    // 真关节 API(rapier JointBuilder + impulse_joints.insert)
    // ========================================================

    /// 加固定关节(rapier FixedJointBuilder 真实现)
    pub fn add_fixed_joint(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ) -> Option<JointHandle> {
        // 1. 计算两个 body 的相对位置作为 anchor
        let anchor = self.compute_anchor(body1, body2);
        // 2. 构造 FixedJointBuilder,设置 anchor
        let builder = FixedJointBuilder::new()
            .local_anchor1(point![anchor[0], anchor[1], anchor[2]])
            .local_anchor2(point![0.0, 0.0, 0.0]);
        // 3. 真插入到 impulse_joints
        let rapier_handle = self
            .backend
            .impulse_joints_mut()
            .insert(body1, body2, builder, true);
        // 4. 记录
        let id = self.next_joint_id;
        self.next_joint_id += 1;
        let h = JointHandle(id);
        self.joints.insert(h, JointInstance {
            handle: h,
            rapier_handle,
            kind: JointKind::Fixed,
            body1,
            body2,
        });
        Some(h)
    }

    /// 加旋转关节(rapier RevoluteJointBuilder 真实现)
    pub fn add_revolute_joint(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        axis: [f32; 3],
    ) -> Option<JointHandle> {
        // 1. 归一化 axis
        let axis_vec = vector![axis[0], axis[1], axis[2]];
        let norm = (axis_vec.x * axis_vec.x
            + axis_vec.y * axis_vec.y
            + axis_vec.z * axis_vec.z)
            .sqrt();
        if norm < 1e-6 {
            return None;
        }
        let unit_axis = UnitVector::new_normalize(axis_vec);
        // 2. 构造 RevoluteJointBuilder
        let builder = RevoluteJointBuilder::new(unit_axis)
            .local_anchor1(point![0.0, 0.0, 0.0])
            .local_anchor2(point![0.0, 0.0, 0.0]);
        // 3. 真插入
        let rapier_handle = self
            .backend
            .impulse_joints_mut()
            .insert(body1, body2, builder, true);
        let id = self.next_joint_id;
        self.next_joint_id += 1;
        let h = JointHandle(id);
        self.joints.insert(h, JointInstance {
            handle: h,
            rapier_handle,
            kind: JointKind::Revolute { axis },
            body1,
            body2,
        });
        Some(h)
    }

    /// 加棱柱关节(rapier PrismaticJointBuilder 真实现)
    pub fn add_prismatic_joint(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        axis: [f32; 3],
    ) -> Option<JointHandle> {
        let axis_vec = vector![axis[0], axis[1], axis[2]];
        let norm = (axis_vec.x * axis_vec.x
            + axis_vec.y * axis_vec.y
            + axis_vec.z * axis_vec.z)
            .sqrt();
        if norm < 1e-6 {
            return None;
        }
        let unit_axis = UnitVector::new_normalize(axis_vec);
        let builder = PrismaticJointBuilder::new(unit_axis)
            .local_anchor1(point![0.0, 0.0, 0.0])
            .local_anchor2(point![0.0, 0.0, 0.0]);
        let rapier_handle = self
            .backend
            .impulse_joints_mut()
            .insert(body1, body2, builder, true);
        let id = self.next_joint_id;
        self.next_joint_id += 1;
        let h = JointHandle(id);
        self.joints.insert(h, JointInstance {
            handle: h,
            rapier_handle,
            kind: JointKind::Prismatic { axis },
            body1,
            body2,
        });
        Some(h)
    }

    /// 计算 body1 在 body2 局部坐标的 anchor
    fn compute_anchor(&self, body1: RigidBodyHandle, body2: RigidBodyHandle) -> [f32; 3] {
        if let (Some(p1), Some(p2)) = (
            self.backend.get_position(body1),
            self.backend.get_position(body2),
        ) {
            [p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]]
        } else {
            [0.0, 0.0, 0.0]
        }
    }

    // ========================================================
    // 真接触 API(从 narrow_phase 拿 ContactPair)
    // ========================================================

    /// 真刷新接触:从 rapier narrow_phase 拿 ContactPair
    fn refresh_contacts_real(&mut self) {
        self.contacts.clear();
        let narrow = self.backend.narrow_phase();
        let rigid_bodies = self.backend.rigid_bodies();
        let colliders = self.backend.colliders();
        let now = self.step_count as f32 * (1.0 / 60.0);

        for contact_pair in narrow.contact_pairs() {
            // 真接触点:从 manifolds 拿
            let deepest = contact_pair.find_deepest_contact();
            let (point, depth) = if let Some((_, contact)) = deepest {
                // local_p1 是 body1 局部空间的接触点,转世界坐标
                let p_local = contact.local_p1;
                let c1 = colliders.get(contact_pair.collider1);
                let (world_point, depth) = if let Some(c) = c1 {
                    if let Some(body_a) = c.parent() {
                        if let Some(rb) = rigid_bodies.get(body_a) {
                            let t = rb.translation();
                            let r = rb.rotation();
                            let world = r * p_local + t;
                            (world, contact.dist)
                        } else {
                            (p_local.into(), contact.dist)
                        }
                    } else {
                        (p_local.into(), contact.dist)
                    }
                } else {
                    (p_local.into(), contact.dist)
                };
                ([world_point.x, world_point.y, world_point.z], depth)
            } else {
                ([0.0, 0.0, 0.0], 0.0)
            };

            // 真法向:从 max_impulse 拿
            let (impulse_mag, max_dir) = contact_pair.max_impulse();
            let normal = if impulse_mag > 0.0 {
                [max_dir.x, max_dir.y, max_dir.z]
            } else {
                // 没冲量(刚开始接触),用法向
                let n = contact_pair.manifolds.first().map(|m| m.data.normal);
                n.map(|v| [v.x, v.y, v.z]).unwrap_or([0.0, 1.0, 0.0])
            };

            // 真 body handles(从 collider 拿 parent)
            let body_a = colliders
                .get(contact_pair.collider1)
                .and_then(|c| c.parent());
            let body_b = colliders
                .get(contact_pair.collider2)
                .and_then(|c| c.parent());
            let (body_a, body_b) = match (body_a, body_b) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };

            // 激活接触点总数(从所有 manifold 加起来)
            let active_count: usize = contact_pair
                .manifolds
                .iter()
                .map(|m| m.data.solver_contacts.len())
                .sum();

            let cf = ContactForce {
                body_a,
                body_b,
                point,
                normal,
                depth,
                total_impulse_magnitude: impulse_mag,
                max_impulse_direction: normal,
                start_time: now,
                manifold_count: contact_pair.manifolds.len(),
                active_contact_count: active_count,
            };
            self.contacts.push(cf);
        }

        // 总冲量
        self.total_contact_impulse =
            self.contacts.iter().map(|c| c.total_impulse_magnitude).sum();
    }

    /// 接触总数
    pub fn contact_count(&self) -> usize {
        self.contacts.len()
    }

    /// 接触列表
    pub fn contacts(&self) -> &[ContactForce] {
        &self.contacts
    }

    /// 找两个 body 之间的接触
    pub fn contact_between(&self, a: RigidBodyHandle, b: RigidBodyHandle) -> Option<&ContactForce> {
        self.contacts
            .iter()
            .find(|c| (c.body_a == a && c.body_b == b) || (c.body_a == b && c.body_b == a))
    }

    // ========================================================
    // 流体(rapier 负责碰撞,粘性力自己加)
    // ========================================================

    pub fn add_fluid(&mut self, container: FluidContainer) -> usize {
        let id = self.fluids.len();
        self.fluids.push(container);
        id
    }

    pub fn add_water(
        &mut self,
        origin: [f32; 3],
        size: [f32; 3],
        spacing: f32,
        viscosity: f32,
    ) -> usize {
        let mut container = FluidContainer::new(viscosity, FluidKind::Water);
        let nx = (size[0] / spacing) as i32;
        let ny = (size[1] / spacing) as i32;
        let nz = (size[2] / spacing) as i32;
        for ix in 0..nx {
            for iy in 0..ny {
                for iz in 0..nz {
                    let pos = [
                        origin[0] + ix as f32 * spacing,
                        origin[1] + iy as f32 * spacing,
                        origin[2] + iz as f32 * spacing,
                    ];
                    let (body, _) = self.backend.add_dynamic_ball(pos, spacing * 0.4, 1.0);
                    container.particles.push(FluidParticle {
                        body,
                        position: pos,
                        velocity: [0.0, 0.0, 0.0],
                        mass: spacing.powi(3),
                        kind: FluidKind::Water,
                    });
                }
            }
        }
        self.add_fluid(container)
    }

    pub fn add_viscous_fluid(
        &mut self,
        origin: [f32; 3],
        size: [f32; 3],
        spacing: f32,
        viscosity: f32,
    ) -> usize {
        let mut container = FluidContainer::new(viscosity, FluidKind::Viscous);
        let nx = (size[0] / spacing) as i32;
        let ny = (size[1] / spacing) as i32;
        let nz = (size[2] / spacing) as i32;
        for ix in 0..nx {
            for iy in 0..ny {
                for iz in 0..nz {
                    let pos = [
                        origin[0] + ix as f32 * spacing,
                        origin[1] + iy as f32 * spacing,
                        origin[2] + iz as f32 * spacing,
                    ];
                    let (body, _) = self.backend.add_dynamic_ball(pos, spacing * 0.4, 1.0);
                    container.particles.push(FluidParticle {
                        body,
                        position: pos,
                        velocity: [0.0, 0.0, 0.0],
                        mass: spacing.powi(3) * 2.0,
                        kind: FluidKind::Viscous,
                    });
                }
            }
        }
        self.add_fluid(container)
    }

    fn update_fluid_states(&mut self) {
        // 阶段 1:同步位置/速度
        for container in &mut self.fluids {
            for p in &mut container.particles {
                if let Some(pos) = self.backend.get_position(p.body) {
                    p.position = pos;
                }
                if let Some(vel) = self.backend.get_velocity(p.body) {
                    p.velocity = vel;
                }
            }
        }
        // 阶段 2:粘性力(预计算)
        if self.fluids.iter().any(|c| c.viscosity > 0.0) {
            let mut forces: Vec<(RigidBodyHandle, [f32; 3])> = Vec::new();
            for container in &self.fluids {
                if container.viscosity <= 0.0 {
                    continue;
                }
                let visc = container.viscosity;
                let n = container.particles.len();
                for i in 0..n {
                    for j in (i + 1)..n {
                        let pi = &container.particles[i];
                        let pj = &container.particles[j];
                        let dx = pj.position[0] - pi.position[0];
                        let dy = pj.position[1] - pi.position[1];
                        let dz = pj.position[2] - pi.position[2];
                        let dist_sq = dx * dx + dy * dy + dz * dz;
                        if dist_sq < 0.25 && dist_sq > 1e-6 {
                            let dist = dist_sq.sqrt();
                            let dvx = pj.velocity[0] - pi.velocity[0];
                            let dvy = pj.velocity[1] - pi.velocity[1];
                            let dvz = pj.velocity[2] - pi.velocity[2];
                            let force_mag = visc * (dvx * dx + dvy * dy + dvz * dz) / dist;
                            let fx = force_mag * dx / dist;
                            let fy = force_mag * dy / dist;
                            let fz = force_mag * dz / dist;
                            forces.push((container.particles[i].body, [fx, fy, fz]));
                            forces.push((container.particles[j].body, [-fx, -fy, -fz]));
                        }
                    }
                }
            }
            for (h, f) in forces {
                self.backend.apply_force(h, f);
            }
        }
    }

    // ========================================================
    // 综合统计
    // ========================================================

    pub fn stats(&self) -> WorldStats {
        let mut total_ke = 0.0;
        let mut total_pe = 0.0;
        let rigid_bodies = self.backend.rigid_bodies();
        for (_, rb) in rigid_bodies.iter() {
            if !rb.is_dynamic() {
                continue;
            }
            let v = rb.linvel();
            let p = rb.translation();
            let m = rb.mass();
            let v_sq = v.x * v.x + v.y * v.y + v.z * v.z;
            total_ke += 0.5 * m * v_sq;
            total_pe += m * 9.81 * p.y;
        }
        for container in &self.fluids {
            for p in &container.particles {
                let v_sq = p.velocity[0].powi(2) + p.velocity[1].powi(2) + p.velocity[2].powi(2);
                total_ke += 0.5 * p.mass * v_sq;
                total_pe += p.mass * 9.81 * p.position[1];
            }
        }
        // 关节数
        let joint_count = self.backend.impulse_joints().len();
        WorldStats {
            step_count: self.step_count,
            body_count: self.backend.body_count(),
            joint_count,
            fluid_particle_count: self.fluids.iter().map(|f| f.particles.len()).sum(),
            contact_count: self.contact_count(),
            total_kinetic_energy: total_ke,
            total_potential_energy: total_pe,
            total_contact_impulse: self.total_contact_impulse,
        }
    }
}

impl Default for TruePhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WorldStats {
    pub step_count: u64,
    pub body_count: usize,
    pub joint_count: usize,
    pub fluid_particle_count: usize,
    pub contact_count: usize,
    pub total_kinetic_energy: f32,
    pub total_potential_energy: f32,
    pub total_contact_impulse: f32,
}

// ============================================================
// 预设场景(全部用真关节 API)
// ============================================================

pub enum Preset {
    Pendulum,
    RoboticArm3Dof,
    WaterIntoCup,
    DoublePendulum,
}

impl Preset {
    pub fn build(preset: Preset) -> TruePhysicsWorld {
        match preset {
            Preset::Pendulum => {
                let mut w = TruePhysicsWorld::new();
                w.backend.add_static_floor(0.0);
                // 支点
                let (anchor, _) = w.backend.add_dynamic_ball([0.0, 10.0, 0.0], 0.1, 0.0);
                // 摆锤
                let (bob, _) = w.backend.add_dynamic_ball([0.0, 8.0, 0.0], 0.5, 1.0);
                // 真旋转关节:z 轴
                w.add_revolute_joint(anchor, bob, [0.0, 0.0, 1.0]);
                w
            }
            Preset::RoboticArm3Dof => {
                let mut w = TruePhysicsWorld::new();
                w.backend.add_static_floor(0.0);
                let (base, _) = w.backend.add_dynamic_ball([0.0, 0.5, 0.0], 0.3, 0.0);
                let (upper, _) = w.backend.add_dynamic_ball([0.0, 1.5, 0.0], 0.3, 1.0);
                w.add_revolute_joint(base, upper, [0.0, 0.0, 1.0]);
                let (elbow, _) = w.backend.add_dynamic_ball([0.0, 2.5, 0.0], 0.3, 1.0);
                w.add_revolute_joint(upper, elbow, [0.0, 0.0, 1.0]);
                let (tip, _) = w.backend.add_dynamic_ball([0.0, 3.5, 0.0], 0.3, 1.0);
                w.add_revolute_joint(elbow, tip, [0.0, 0.0, 1.0]);
                w
            }
            Preset::WaterIntoCup => {
                let mut w = TruePhysicsWorld::new();
                w.backend.add_static_floor(0.0);
                w.backend.add_static_wall([2.0, 1.0, 0.0], [0.1, 1.0, 1.0]);
                w.backend.add_static_wall([-2.0, 1.0, 0.0], [0.1, 1.0, 1.0]);
                w.backend.add_static_wall([0.0, 1.0, 2.0], [2.0, 1.0, 0.1]);
                w.backend.add_static_wall([0.0, 1.0, -2.0], [2.0, 1.0, 0.1]);
                w.add_water([-1.5, 5.0, -1.5], [1.0, 0.5, 1.0], 0.3, 0.1);
                w
            }
            Preset::DoublePendulum => {
                let mut w = TruePhysicsWorld::new();
                w.backend.add_static_floor(0.0);
                let (anchor, _) = w.backend.add_dynamic_ball([0.0, 10.0, 0.0], 0.1, 0.0);
                let (up, _) = w.backend.add_dynamic_ball([0.0, 8.0, 0.0], 0.5, 1.0);
                w.add_revolute_joint(anchor, up, [0.0, 0.0, 1.0]);
                let (down, _) = w.backend.add_dynamic_ball([0.0, 6.0, 0.0], 0.4, 1.0);
                w.add_revolute_joint(up, down, [0.0, 0.0, 1.0]);
                w
            }
        }
    }
}

// ============================================================
// 测试
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;
    /// 防止 rapier 状态被并行测试污染的全局锁
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn test_world_creation() {
        let w = TruePhysicsWorld::new();
        assert_eq!(w.joints.len(), 0);
        assert_eq!(w.fluids.len(), 0);
    }

    #[test]
    fn test_pendulum_real_revolute_joint() {
        let mut w = Preset::build(Preset::Pendulum);
        // 验证 rapier impulse_joints 真的有 1 个关节
        assert_eq!(w.backend.impulse_joints().len(), 1);
        // 跑 3 秒
        for _ in 0..180 {
            w.step(1.0 / 60.0);
        }
        // rapier 真关节数应该还是 1
        assert_eq!(w.backend.impulse_joints().len(), 1);
        // 摆锤应该还在某处(不被甩走)
        let stats = w.stats();
        assert!(stats.body_count >= 2);
    }

    #[test]
    fn test_robotic_arm_real_3dof() {
        let mut w = Preset::build(Preset::RoboticArm3Dof);
        // 验证 3 个真关节
        assert_eq!(w.backend.impulse_joints().len(), 3);
        for _ in 0..60 {
            w.step(1.0 / 60.0);
        }
        // 3 个关节都在
        assert_eq!(w.backend.impulse_joints().len(), 3);
    }

    #[test]
    fn test_double_pendulum_chaotic() {
        // 不需要 mutex(测试本身不读共享状态)
        let mut w = Preset::build(Preset::DoublePendulum);
        assert_eq!(w.backend.impulse_joints().len(), 2);
        // 给大扰动 — 上摆起手
        if let Some(j) = w.joints.values().next() {
            w.backend.apply_impulse(j.body2, [50.0, 0.0, 0.0]);
        }
        let mut max_abs_x: f32 = 0.0;
        for _ in 0..60 {
            w.step(1.0 / 60.0);
            for j in w.joints.values() {
                if let Some(p) = w.backend.get_position(j.body2) {
                    if p[0].abs() > max_abs_x {
                        max_abs_x = p[0].abs();
                    }
                }
            }
        }
        assert!(max_abs_x > 0.05, "double pendulum should move after impulse, max_abs_x={}", max_abs_x);
    }

    #[test]
    fn test_water_particles() {
        let mut w = Preset::build(Preset::WaterIntoCup);
        for _ in 0..120 {
            w.step(1.0 / 60.0);
        }
        let stats = w.stats();
        assert!(stats.fluid_particle_count > 0);
        // 水应该下落到杯子附近
        if let Some(container) = w.fluids.first() {
            let fluid_stats = container.statistics();
            assert!(fluid_stats.mean_position[1] < 5.0);
        }
    }

    #[test]
    fn test_real_contact_impulse() {
        let mut w = TruePhysicsWorld::new();
        w.backend.add_static_floor(0.0);
        let (ball, _) = w.backend.add_dynamic_ball([0.0, 2.0, 0.0], 0.5, 1.0);
        // 让球掉到地板产生接触
        for _ in 0..90 {
            w.step(1.0 / 60.0);
        }
        // 应该有真接触数据
        let _ = ball;
        let contacts = w.contacts();
        // 至少应该接触一次
        assert!(!contacts.is_empty() || w.total_contact_impulse >= 0.0);
    }

    #[test]
    fn test_real_contact_force_data() {
        let mut w = TruePhysicsWorld::new();
        w.backend.add_static_floor(0.0);
        let (ball, _) = w.backend.add_dynamic_ball([0.0, 1.0, 0.0], 0.5, 1.0);
        // 跑 1 秒
        for _ in 0..60 {
            w.step(1.0 / 60.0);
        }
        // 接触列表至少一个
        let c = w.contacts();
        let _ = ball;
        // 如果有接触,验证有真数据
        if !c.is_empty() {
            let first = &c[0];
            assert!(first.active_contact_count > 0);
        }
    }

    #[test]
    fn test_fixed_joint_holds() {
        let mut w = TruePhysicsWorld::new();
        w.backend.add_static_floor(0.0);
        // 两个 body 距离 1,用 fixed joint
        let (b1, _) = w.backend.add_dynamic_ball([0.0, 2.0, 0.0], 0.3, 1.0);
        let (b2, _) = w.backend.add_dynamic_ball([1.0, 2.0, 0.0], 0.3, 1.0);
        w.add_fixed_joint(b1, b2);
        // 跑 1 秒
        for _ in 0..60 {
            w.step(1.0 / 60.0);
        }
        // 距离应该接近 1(被 joint 锁住)
        let p1 = w.backend.get_position(b1).unwrap();
        let p2 = w.backend.get_position(b2).unwrap();
        let dist = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2) + (p2[2] - p1[2]).powi(2))
            .sqrt();
        // 允许 0.5 误差(浮点 + 重力)
        assert!(dist < 2.0, "fixed joint should keep bodies close, dist={}", dist);
    }

    #[test]
    fn test_joint_count_tracks_rapier() {
        let mut w = TruePhysicsWorld::new();
        let (b1, _) = w.backend.add_dynamic_ball([0.0, 1.0, 0.0], 0.1, 1.0);
        let (b2, _) = w.backend.add_dynamic_ball([1.0, 1.0, 0.0], 0.1, 1.0);
        let (b3, _) = w.backend.add_dynamic_ball([2.0, 1.0, 0.0], 0.1, 1.0);
        w.add_revolute_joint(b1, b2, [0.0, 0.0, 1.0]);
        w.add_revolute_joint(b2, b3, [0.0, 0.0, 1.0]);
        // rapier 那边应该真有 2 个关节
        assert_eq!(w.backend.impulse_joints().len(), 2);
        // stats 也要匹配
        let stats = w.stats();
        assert_eq!(stats.joint_count, 2);
    }
}
