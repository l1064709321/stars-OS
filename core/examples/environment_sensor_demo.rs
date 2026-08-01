//! 真环境传感器驱动的物理世界 demo
//!
//! 不预设场景,从"虚拟传感器"(模拟真实手机传感器)读数据,推给物理世界。
//! 在真手机上,这个虚拟传感器会被 Android 真传感器替代。

use quantum_core::environment_world::EnvironmentWorld;
use quantum_core::sensors::{SimulatedAccelerometer, SimulatedGps, SimulatedMicrophone};

fn main() {
    println!("=== 真环境传感器驱动的物理世界 demo ===\n");
    println!("模拟真实手机的传感器,推给物理世界。");
    println!("真手机上,SensorEventListener 替代这些 SimulatedSensor。\n");

    let mut world = EnvironmentWorld::new();
    // 加地板
    world.physics.backend.add_static_floor(0.0);
    // 加 3 个动态球
    let (b1, _) = world
        .physics
        .backend
        .add_dynamic_ball([0.0, 3.0, 0.0], 0.3, 1.0);
    let (b2, _) = world
        .physics
        .backend
        .add_dynamic_ball([1.5, 3.0, 0.0], 0.3, 1.0);
    let (b3, _) = world
        .physics
        .backend
        .add_dynamic_ball([-1.5, 3.0, 0.0], 0.3, 1.0);

    // 注册"虚拟传感器"(在真手机上,这些是 Android SensorManager 的真实回调)
    world
        .sensor_hub
        .register(Box::new(SimulatedAccelerometer::new()));
    world
        .sensor_hub
        .register(Box::new(SimulatedMicrophone::new()));
    world
        .sensor_hub
        .register(Box::new(SimulatedGps::default()));
    world.sensor_hub.start_all();

    println!("初始状态: 6 个动态球 + 1 个地板");
    let stats = world.stats();
    println!("  bodies={}, sensors={}\n", stats.body_count, stats.sensor_count);

    // 跑 200 步
    for tick in 0..200 {
        world.step();
        if tick % 40 == 0 {
            let stats = world.stats();
            let p1 = world.physics.backend.get_position(b1).unwrap();
            let p2 = world.physics.backend.get_position(b2).unwrap();
            let p3 = world.physics.backend.get_position(b3).unwrap();
            println!(
                "t={:>3} | b1=({:.2},{:.2},{:.2}) b2=({:.2},{:.2},{:.2}) b3=({:.2},{:.2},{:.2}) | accel_hist={} mic_hist={} | last_mic={:?}",
                tick,
                p1[0], p1[1], p1[2],
                p2[0], p2[1], p2[2],
                p3[0], p3[1], p3[2],
                stats.accel.as_ref().map(|a| a.len()).unwrap_or(0),
                stats.mic_level.is_some() as usize,
                stats.mic_level
            );
        }
    }
    println!();

    let final_stats = world.stats();
    println!("=== 最终状态 ===");
    println!("步数: {}", final_stats.step_count);
    println!("事件数: {}", final_stats.event_count);
    println!("传感器数据: accel_history={}, mic_history={}",
        world.accel_history.len(),
        world.mic_history.len()
    );
    println!("当前 GPS: {:?}", final_stats.current_gps);
    println!("当前温度: {:.1}℃", final_stats.current_temp);
    println!();
    println!("=== 完成 ===");
    println!();
    println!("这个 demo 跑的是沙箱里\"模拟的传感器\"。");
    println!("真手机上,SensorEventListener 会推真实数据:");
    println!("  - 摇手机 → 物理世界物体被推");
    println!("  - 拍手 → 物理世界产生冲击波");
    println!("  - 走到街上 → GPS 变化被记录");
    println!("  - 房间温度变化 → 温度被记录");
}
