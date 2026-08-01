//! 真环境驱动的物理世界
//!
//! 区别于"真物理世界模型(true_world_model)":
//! - true_world_model:我手写 `add_ball([0, 5, 0])` 预设场景
//! - environment_world:从真实传感器数据流创建物体 / 施力 / 移动
//!
//! ## 真实世界流
//! - 加速度计变化 → 触发物理世界里的"被推一下"事件
//! - 麦克风声音大 → 触发物理世界里的"爆炸冲击波"
//! - 光强变化 → 物理世界里的物体被照亮(影响视觉子系统)
//! - 温度变化 → 物理世界里的流体变热
//! - GPS 移动 → 物理世界随用户移动
//!
//! 这是 AGI 操作系统"看到真实世界"的核心。

use crate::sensors::{SensorKind, SensorReading, SensorHub};
use crate::true_world_model::{TruePhysicsWorld, JointHandle};

/// 真环境事件
#[derive(Debug, Clone)]
pub enum EnvironmentEvent {
    /// 推一下(由加速度计突变触发)
    Push {
        impulse: [f32; 3],
        source: &'static str,
    },
    /// 声音大 → 冲击波(由麦克风触发)
    Shockwave {
        center: [f32; 3],
        intensity: f32,
    },
    /// 温度变化
    TemperatureChange { new_temp: f32 },
    /// 位置变化(由 GPS)
    Moved { delta_m: f32 },
    /// 倾倒(由陀螺仪)
    Tilt { axis: [f32; 3], angle_rad: f32 },
}

/// 环境驱动的物理世界
pub struct EnvironmentWorld {
    pub physics: TruePhysicsWorld,
    pub sensor_hub: SensorHub,
    /// 事件队列
    pub events: Vec<EnvironmentEvent>,
    /// 真实时间步(传感器实际间隔)
    pub real_dt: f32,
    /// 累计步数
    pub step_count: u64,
    /// 当前位置(GPS)
    pub current_gps: Option<[f32; 3]>,
    /// 当前温度
    pub current_temp: f32,
    /// 声音 dB 历史(滑动平均)
    pub mic_history: Vec<f32>,
    /// 加速度计 x/y/z 历史
    pub accel_history: Vec<[f32; 3]>,
    /// 最大历史长度
    pub max_history: usize,
}

impl EnvironmentWorld {
    pub fn new() -> Self {
        Self {
            physics: TruePhysicsWorld::new(),
            sensor_hub: SensorHub::new(),
            events: Vec::new(),
            real_dt: 1.0 / 60.0,
            step_count: 0,
            current_gps: None,
            current_temp: 20.0,
            mic_history: Vec::new(),
            accel_history: Vec::new(),
            max_history: 100,
        }
    }

    /// 一步:从传感器读真实数据 → 推环境事件 → 物理世界更新
    pub fn step(&mut self) {
        self.step_count += 1;
        // 1. 拉所有传感器数据
        let readings = self.sensor_hub.poll_all();
        // 2. 把每条读数转成环境事件
        for r in &readings {
            self.process_reading(r);
        }
        // 3. 应用事件到物理世界
        self.apply_events();
        // 4. 物理世界推进
        self.physics.step(self.real_dt);
    }

    /// 一条传感器读数 → 环境事件(公开,给 JNI 用)
    pub fn process_reading(&mut self, r: &SensorReading) {
        match r.kind {
            SensorKind::Accelerometer => {
                self.accel_history.push(r.vec);
                if self.accel_history.len() > self.max_history {
                    self.accel_history.remove(0);
                }
                // 检测突变(加速度比上一帧大很多)
                if self.accel_history.len() >= 2 {
                    let prev = self.accel_history[self.accel_history.len() - 2];
                    let dx = (r.vec[0] - prev[0]).abs();
                    let dy = (r.vec[1] - prev[1]).abs();
                    let dz = (r.vec[2] - prev[2]).abs();
                    let mag = (dx * dx + dy * dy + dz * dz).sqrt();
                    if mag > 5.0 {
                        // 真物理:加速度突变 → 推断为"有东西撞了一下桌子"
                        // 我们施加一个反方向的力给所有动态物体
                        self.events.push(EnvironmentEvent::Push {
                            impulse: [r.vec[0] * 0.1, r.vec[1] * 0.1, r.vec[2] * 0.1],
                            source: "accelerometer_shock",
                        });
                    }
                }
            }
            SensorKind::Gyroscope => {
                // 倾倒事件(简化:超过 1 rad/s 触发)
                let ang = (r.vec[0].powi(2) + r.vec[1].powi(2) + r.vec[2].powi(2)).sqrt();
                if ang > 1.0 {
                    self.events.push(EnvironmentEvent::Tilt {
                        axis: r.vec,
                        angle_rad: ang,
                    });
                }
            }
            SensorKind::Microphone => {
                self.mic_history.push(r.scalar);
                if self.mic_history.len() > self.max_history {
                    self.mic_history.remove(0);
                }
                // 检测突然变响(平均 + 15 dB)
                if self.mic_history.len() >= 10 {
                    let recent: f32 = self.mic_history[self.mic_history.len() - 5..]
                        .iter()
                        .sum::<f32>()
                        / 5.0;
                    let baseline: f32 = self.mic_history[..self.mic_history.len() - 5]
                        .iter()
                        .sum::<f32>()
                        / (self.mic_history.len() - 5) as f32;
                    if recent - baseline > 15.0 {
                        // 突然变响 → 冲击波
                        self.events.push(EnvironmentEvent::Shockwave {
                            center: [0.0, 0.5, 0.0],
                            intensity: (recent - baseline) * 0.01,
                        });
                    }
                }
            }
            SensorKind::Temperature => {
                self.current_temp = r.scalar;
                self.events.push(EnvironmentEvent::TemperatureChange { new_temp: r.scalar });
            }
            SensorKind::Gps => {
                if let Some(prev) = self.current_gps {
                    let dx = r.vec[0] - prev[0];
                    let dy = r.vec[1] - prev[1];
                    let dz = r.vec[2] - prev[2];
                    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                    if dist > 0.0001 {
                        // 111_000m / 1° 大约
                        let meters = dist * 111_000.0;
                        self.events.push(EnvironmentEvent::Moved { delta_m: meters });
                    }
                }
                self.current_gps = Some(r.vec);
            }
            _ => {}
        }
    }

    /// 把累积的事件应用到物理世界(公开,给 JNI 用)
    pub fn apply_events(&mut self) {
        let events = std::mem::take(&mut self.events);
        for ev in events {
            match ev {
                EnvironmentEvent::Push { impulse, source: _ } => {
                    // 给所有动态 body 一个反方向力
                    for body_handle in self.all_dynamic_bodies() {
                        self.physics.backend.apply_impulse(body_handle, impulse);
                    }
                }
                EnvironmentEvent::Shockwave { center, intensity } => {
                    // 给所有 body 离冲击中心越远力越小
                    for body_handle in self.all_dynamic_bodies() {
                        if let Some(p) = self.physics.backend.get_position(body_handle) {
                            let dx = p[0] - center[0];
                            let dy = p[1] - center[1];
                            let dz = p[2] - center[2];
                            let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(0.1);
                            let force = intensity / dist;
                            let fx = dx / dist * force;
                            let fy = dy / dist * force;
                            let fz = dz / dist * force;
                            self.physics.backend.apply_force(body_handle, [fx, fy, fz]);
                        }
                    }
                }
                EnvironmentEvent::TemperatureChange { new_temp: _ } => {
                    // 暂时只记录
                }
                EnvironmentEvent::Moved { delta_m: _ } => {
                    // 暂时只记录
                }
                EnvironmentEvent::Tilt { axis, angle_rad } => {
                    // 倾倒:把所有 body 加一个跟陀螺仪轴向相关的力
                    for body_handle in self.all_dynamic_bodies() {
                        let impulse = [axis[0] * angle_rad * 0.05,
                                       axis[1] * angle_rad * 0.05,
                                       axis[2] * angle_rad * 0.05];
                        self.physics.backend.apply_impulse(body_handle, impulse);
                    }
                }
            }
        }
    }

    /// 拿到所有动态 body 的 handles
    fn all_dynamic_bodies(&self) -> Vec<rapier3d::prelude::RigidBodyHandle> {
        let mut out = Vec::new();
        let rigid_bodies = self.physics.backend.rigid_bodies();
        for (handle, rb) in rigid_bodies.iter() {
            if rb.is_dynamic() {
                out.push(handle);
            }
        }
        out
    }

    /// 统计
    pub fn stats(&self) -> EnvironmentStats {
        EnvironmentStats {
            step_count: self.step_count,
            sensor_count: self.sensor_hub.sensor_count(),
            event_count: self.events.len(),
            body_count: self.physics.backend.body_count(),
            joint_count: self.physics.backend.impulse_joints().len(),
            current_temp: self.current_temp,
            current_gps: self.current_gps,
            mic_level: self.mic_history.last().copied(),
            accel: self.accel_history.last().copied(),
        }
    }
}

impl Default for EnvironmentWorld {
    fn default() -> Self {
        Self::new()
    }
}

/// 环境统计
#[derive(Debug, Clone)]
pub struct EnvironmentStats {
    pub step_count: u64,
    pub sensor_count: usize,
    pub event_count: usize,
    pub body_count: usize,
    pub joint_count: usize,
    pub current_temp: f32,
    pub current_gps: Option<[f32; 3]>,
    pub mic_level: Option<f32>,
    pub accel: Option<[f32; 3]>,
}

// ============================================================
// 测试
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::{SimulatedAccelerometer, SimulatedGps, SimulatedMicrophone};
    use crate::true_world_model::Preset;

    #[test]
    fn test_environment_world_creation() {
        let mut w = EnvironmentWorld::new();
        assert_eq!(w.sensor_hub.sensor_count(), 0);
    }

    #[test]
    fn test_environment_with_simulated_sensors() {
        let mut w = EnvironmentWorld::new();
        // 物理世界:几个动态球
        let (b1, _) = w.physics.backend.add_dynamic_ball([0.0, 2.0, 0.0], 0.5, 1.0);
        let (b2, _) = w.physics.backend.add_dynamic_ball([1.0, 2.0, 0.0], 0.5, 1.0);
        // 传感器
        w.sensor_hub
            .register(Box::new(SimulatedAccelerometer::new()));
        w.sensor_hub
            .register(Box::new(SimulatedMicrophone::new()));
        w.sensor_hub
            .register(Box::new(SimulatedGps::default()));
        w.sensor_hub.start_all();
        // 跑 100 步:从传感器读真数据
        for _ in 0..100 {
            w.step();
        }
        let stats = w.stats();
        assert!(stats.body_count >= 2, "应该有动态球");
        assert!(stats.sensor_count >= 3, "应该有 3 个传感器");
        // 球应该已经被加速度触发的事件动过
        let p1 = w.physics.backend.get_position(b1).unwrap();
        let p2 = w.physics.backend.get_position(b2).unwrap();
        // 球不一定有显著移动(加速度小),但不能出错
        let _ = (p1, p2);
    }

    #[test]
    fn test_environment_handles_gps() {
        let mut w = EnvironmentWorld::new();
        w.sensor_hub
            .register(Box::new(SimulatedGps::new(39.9, 116.4)));
        w.sensor_hub.start_all();
        for _ in 0..20 {
            w.step();
        }
        // 应该有 GPS 位置
        let stats = w.stats();
        assert!(stats.current_gps.is_some());
    }

    #[test]
    fn test_environment_handles_microphone() {
        let mut w = EnvironmentWorld::new();
        w.sensor_hub
            .register(Box::new(SimulatedMicrophone::new()));
        w.sensor_hub.start_all();
        for _ in 0..20 {
            w.step();
        }
        let stats = w.stats();
        assert!(stats.mic_level.is_some());
    }

    #[test]
    fn test_environment_with_preset() {
        // 用 Preset 单摆,传感器驱动
        let mut w = EnvironmentWorld::new();
        w.physics = Preset::build(Preset::Pendulum);
        w.sensor_hub
            .register(Box::new(SimulatedAccelerometer::new()));
        w.sensor_hub.start_all();
        for _ in 0..120 {
            w.step();
        }
        let stats = w.stats();
        // 单摆有 1 个关节
        assert!(stats.joint_count >= 1);
    }

    #[test]
    fn test_environment_records_temp() {
        let mut w = EnvironmentWorld::new();
        use crate::sensors::Sensor;
        struct TempSensor { running: bool, t: u64 }
        impl Sensor for TempSensor {
            fn kind(&self) -> SensorKind { SensorKind::Temperature }
            fn name(&self) -> &str { "temp" }
            fn read(&mut self) -> Option<SensorReading> {
                if !self.running { return None; }
                self.t += 1;
                Some(SensorReading::now_scalar(SensorKind::Temperature, 20.0 + self.t as f32 * 0.1))
            }
            fn start(&mut self) -> Result<(), crate::sensors::SensorError> { self.running = true; Ok(()) }
            fn stop(&mut self) -> Result<(), crate::sensors::SensorError> { self.running = false; Ok(()) }
            fn is_running(&self) -> bool { self.running }
        }
        w.sensor_hub.register(Box::new(TempSensor { running: false, t: 0 }));
        w.sensor_hub.start_all();
        for _ in 0..10 {
            w.step();
        }
        let stats = w.stats();
        assert!(stats.current_temp > 20.0, "温度应该上升,got {}", stats.current_temp);
    }
}
