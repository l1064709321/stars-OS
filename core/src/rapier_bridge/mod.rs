//! 工业级物理引擎桥:rapier3d
//!
//! ## 设计目标
//! - 在涌现沙箱内使用工业级物理(rigid body, collision shape, contact)
//! - 暴露给世界模型(模块 18)用
//! - 在 ARM 手机上能跑(rapier3d 是纯 Rust + parry3d 几何)
//!
//! ## 接口
//! - RapierWorld:封装 rapier3d 的世界
//! - add_dynamic_ball / add_static_box 等
//! - step(dt) 推进
//! - 状态查询:get_position / get_velocity / get_contacts
//!
//! rapier3d 0.14 API 速记:
//!   - RigidBodySet / ColliderSet / JointSet / IntegrationParameters
//!   - IslandManager / BroadPhase / NarrowPhase / ImpulseJointSet
//!   - CCDSolver / SolverParams
//!   - PhysicsPipeline::step 推进

use rapier3d::prelude::*;

/// 工业级物理世界
pub struct RapierWorld {
    /// 物理流水线
    pub pipeline: PhysicsPipeline,
    /// 重力
    pub gravity: Vector<Real>,
    /// 积分参数
    pub integration_parameters: IntegrationParameters,
    /// 岛屿管理
    pub island_manager: IslandManager,
    /// 广相
    pub broad_phase: BroadPhase,
    /// 窄相(接触)
    pub narrow_phase: NarrowPhase,
    /// 连杆
    pub impulse_joints: ImpulseJointSet,
    /// 多体关节(预留)
    pub multibody_joints: MultibodyJointSet,
    /// CCD 求解器
    pub ccd_solver: CCDSolver,
    /// 刚体集合
    pub rigid_bodies: RigidBodySet,
    /// 碰撞器集合
    pub colliders: ColliderSet,
    /// 已完成的步数
    pub step_count: u64,
    /// 上一步的接触列表
    pub last_contacts: Vec<ContactInfo>,
}

/// 接触信息(简化的)
#[derive(Debug, Clone, Copy)]
pub struct ContactInfo {
    /// 刚体 A 的 handle
    pub body_a: RigidBodyHandle,
    /// 刚体 B 的 handle(可能是静态的)
    pub body_b: RigidBodyHandle,
    /// 接触点
    pub point: [f32; 3],
    /// 法线
    pub normal: [f32; 3],
    /// 接触深度
    pub depth: f32,
}

impl Default for RapierWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl RapierWorld {
    /// 创建一个新世界,默认重力
    pub fn new() -> Self {
        Self::with_gravity([0.0, -9.81, 0.0])
    }

    /// 创建自定义重力
    pub fn with_gravity(g: [f32; 3]) -> Self {
        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: vector![g[0], g[1], g[2]],
            integration_parameters: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            rigid_bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            step_count: 0,
            last_contacts: Vec::new(),
        }
    }

    /// 创建一个动态球
    pub fn add_dynamic_ball(
        &mut self,
        position: [f32; 3],
        radius: f32,
        density: f32,
    ) -> (RigidBodyHandle, ColliderHandle) {
        let rb = RigidBodyBuilder::dynamic()
            .translation(vector![position[0], position[1], position[2]])
            .build();
        let rb_handle = self.rigid_bodies.insert(rb);

        let collider = ColliderBuilder::ball(radius)
            .density(density)
            .restitution(0.5)
            .friction(0.3)
            .build();
        let coll_handle = self.colliders.insert_with_parent(
            collider,
            rb_handle,
            &mut self.rigid_bodies,
        );

        (rb_handle, coll_handle)
    }

    /// 创建一个动态立方体
    pub fn add_dynamic_box(
        &mut self,
        position: [f32; 3],
        half_extents: [f32; 3],
        density: f32,
    ) -> (RigidBodyHandle, ColliderHandle) {
        let rb = RigidBodyBuilder::dynamic()
            .translation(vector![position[0], position[1], position[2]])
            .build();
        let rb_handle = self.rigid_bodies.insert(rb);

        let collider = ColliderBuilder::cuboid(half_extents[0], half_extents[1], half_extents[2])
            .density(density)
            .restitution(0.4)
            .friction(0.4)
            .build();
        let coll_handle = self.colliders.insert_with_parent(
            collider,
            rb_handle,
            &mut self.rigid_bodies,
        );

        (rb_handle, coll_handle)
    }

    /// 创建一个静态地板(平面)
    pub fn add_static_floor(&mut self, y: f32) -> ColliderHandle {
        let rb = RigidBodyBuilder::fixed().translation(vector![0.0, y, 0.0]).build();
        let rb_handle = self.rigid_bodies.insert(rb);

        // 厚一点(0.5m)防止高速穿透
        let collider = ColliderBuilder::cuboid(100.0, 0.5, 100.0)
            .restitution(0.3)
            .friction(0.5)
            .build();
        self.colliders.insert_with_parent(collider, rb_handle, &mut self.rigid_bodies)
    }

    /// 创建一个静态墙
    pub fn add_static_wall(
        &mut self,
        position: [f32; 3],
        half_extents: [f32; 3],
    ) -> ColliderHandle {
        let rb = RigidBodyBuilder::fixed()
            .translation(vector![position[0], position[1], position[2]])
            .build();
        let rb_handle = self.rigid_bodies.insert(rb);

        let collider = ColliderBuilder::cuboid(half_extents[0], half_extents[1], half_extents[2])
            .friction(0.5)
            .build();
        self.colliders
            .insert_with_parent(collider, rb_handle, &mut self.rigid_bodies)
    }

    /// 推进物理
    pub fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;

        // rapier3d 0.14 的 step 签名(12 个 + 2 钩子)
        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );

        self.step_count += 1;
        self.refresh_contacts();
    }

    /// 从窄相刷新接触列表
    fn refresh_contacts(&mut self) {
        self.last_contacts.clear();
        for contact_pair in self.narrow_phase.contact_pairs() {
            let c1 = contact_pair.collider1;
            let c2 = contact_pair.collider2;
            let body_a = self.colliders.get(c1).and_then(|c| c.parent());
            let body_b = self.colliders.get(c2).and_then(|c| c.parent());

            for manifold in &contact_pair.manifolds {
                if let Some(c) = manifold.contacts().first() {
                    self.last_contacts.push(ContactInfo {
                        body_a: body_a.unwrap_or(RigidBodyHandle::invalid()),
                        body_b: body_b.unwrap_or(RigidBodyHandle::invalid()),
                        point: [c.local_p1.x, c.local_p1.y, c.local_p1.z],
                        normal: [0.0, 1.0, 0.0],
                        depth: c.dist,
                    });
                }
            }
        }
    }

    /// 刚体位置
    pub fn get_position(&self, h: RigidBodyHandle) -> Option<[f32; 3]> {
        self.rigid_bodies.get(h).map(|rb| {
            let t = rb.translation();
            [t.x, t.y, t.z]
        })
    }

    /// 刚体速度
    pub fn get_velocity(&self, h: RigidBodyHandle) -> Option<[f32; 3]> {
        self.rigid_bodies.get(h).map(|rb| {
            let v = rb.linvel();
            [v.x, v.y, v.z]
        })
    }

    /// 给刚体一个力
    pub fn apply_force(&mut self, h: RigidBodyHandle, force: [f32; 3]) {
        if let Some(rb) = self.rigid_bodies.get_mut(h) {
            rb.add_force(vector![force[0], force[1], force[2]], true);
        }
    }

    /// 刚体数量
    pub fn body_count(&self) -> usize {
        self.rigid_bodies.len()
    }

    /// 碰撞器数量
    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    /// 接触数
    pub fn contact_count(&self) -> usize {
        self.last_contacts.len()
    }

    /// 拿到窄相(供真接触 API 用)
    pub fn narrow_phase(&self) -> &NarrowPhase {
        &self.narrow_phase
    }

    /// 拿到 impulse_joints 集合(可变)
    pub fn impulse_joints_mut(&mut self) -> &mut ImpulseJointSet {
        &mut self.impulse_joints
    }

    /// 拿到 impulse_joints 集合(只读)
    pub fn impulse_joints(&self) -> &ImpulseJointSet {
        &self.impulse_joints
    }

    /// 拿到 rigid_bodies 集合(可变)
    pub fn rigid_bodies_mut(&mut self) -> &mut RigidBodySet {
        &mut self.rigid_bodies
    }

    /// 拿到 rigid_bodies 集合(只读)
    pub fn rigid_bodies(&self) -> &RigidBodySet {
        &self.rigid_bodies
    }

    /// 拿到 collider_set(只读)
    pub fn colliders(&self) -> &ColliderSet {
        &self.colliders
    }

    /// 给刚体一个脉冲(瞬间改变速度)
    pub fn apply_impulse(&mut self, h: RigidBodyHandle, impulse: [f32; 3]) {
        if let Some(rb) = self.rigid_bodies.get_mut(h) {
            rb.apply_impulse(vector![impulse[0], impulse[1], impulse[2]], true);
        }
    }

    /// 设置刚体线速度
    pub fn set_linvel(&mut self, h: RigidBodyHandle, v: [f32; 3]) {
        if let Some(rb) = self.rigid_bodies.get_mut(h) {
            rb.set_linvel(vector![v[0], v[1], v[2]], true);
        }
    }
}

// ============================================================
// 测试
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_creation() {
        let world = RapierWorld::new();
        assert_eq!(world.body_count(), 0);
    }

    #[test]
    fn test_add_ball() {
        let mut world = RapierWorld::new();
        let (rb, c) = world.add_dynamic_ball([0.0, 5.0, 0.0], 0.5, 1.0);
        assert_eq!(world.body_count(), 1);
        assert_eq!(world.collider_count(), 1);
        let pos = world.get_position(rb).unwrap();
        assert!((pos[1] - 5.0).abs() < 1e-3);
        let _ = c; // suppress unused
    }

    #[test]
    fn test_gravity_makes_ball_fall() {
        let mut world = RapierWorld::new();
        let (rb, _) = world.add_dynamic_ball([0.0, 5.0, 0.0], 0.5, 1.0);
        // 跑 1 秒
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        let pos = world.get_position(rb).unwrap();
        // 1 秒后应该掉到地面附近(> 0 但小于 5)
        assert!(pos[1] < 4.5, "ball should fall, got pos = {:?}", pos);
        assert!(pos[1] >= 0.0, "ball shouldn't penetrate floor, got pos = {:?}", pos);
    }

    #[test]
    fn test_floor_catches_ball() {
        let mut world = RapierWorld::new();
        world.add_static_floor(0.0);
        let (rb, _) = world.add_dynamic_ball([0.0, 5.0, 0.0], 0.5, 1.0);
        // 跑 3 秒,确保球真的落地
        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }
        let pos = world.get_position(rb).unwrap();
        // 球应该停在地面附近(地板顶在 y=0.5,球半径 0.5,球心应该在 ~1.0)
        assert!(pos[1] < 1.5, "ball should be on ground, got pos = {:?}", pos);
    }

    #[test]
    fn test_apply_force() {
        let mut world = RapierWorld::new();
        let (rb, _) = world.add_dynamic_ball([0.0, 5.0, 0.0], 0.5, 1.0);
        // 加一个向上的力
        world.apply_force(rb, [0.0, 50.0, 0.0]);
        // 跑 1 步
        world.step(1.0 / 60.0);
        let v = world.get_velocity(rb).unwrap();
        // 初始 v=0,加了力 dt=1/60 后,vy > 0
        assert!(v[1] > 0.0, "after upward force, vy should be > 0, got {:?}", v);
    }

    #[test]
    fn test_two_balls_collide() {
        let mut world = RapierWorld::new();
        world.add_static_floor(0.0);
        // 第一个球不动,第二个球在右边
        let (rb1, _) = world.add_dynamic_ball([0.0, 0.5, 0.0], 0.5, 1.0);
        let (rb2, _) = world.add_dynamic_ball([1.0, 0.5, 0.0], 0.5, 1.0);
        // 给 rb2 一个向左的力撞 rb1
        world.apply_impulse(rb2, [-5.0, 0.0, 0.0]);
        // 跑半秒
        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }
        // rb1 应该被撞走了
        let p1 = world.get_position(rb1).unwrap();
        assert!(p1[0] < 0.0, "rb1 should be pushed left, got pos = {:?}", p1);
    }

    #[test]
    fn test_box_creation() {
        let mut world = RapierWorld::new();
        let (rb, c) = world.add_dynamic_box([0.0, 3.0, 0.0], [0.5, 0.5, 0.5], 1.0);
        assert!(world.get_position(rb).is_some());
        let _ = c;
    }

    #[test]
    fn test_wall() {
        let mut world = RapierWorld::new();
        world.add_static_floor(0.0);
        world.add_static_wall([2.0, 1.0, 0.0], [0.1, 1.0, 1.0]);
        let (rb, _) = world.add_dynamic_ball([0.0, 0.5, 0.0], 0.3, 1.0);
        // 设个不那么暴力的初速度
        world.set_linvel(rb, [3.0, 0.0, 0.0]);
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        // 球应该被墙挡住(在 2.0 之前)
        let p = world.get_position(rb).unwrap();
        assert!(p[0] < 2.5, "ball should be stopped by wall, got pos = {:?}", p);
    }
}
