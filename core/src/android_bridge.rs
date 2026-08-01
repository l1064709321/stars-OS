//! Android NDK 桥
//!
//! 这个文件给 C JNI 调用,Java side 调 C,C 调这里。
//!
//! 编译命令:
//!   cargo build --release --target aarch64-linux-android

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_float, c_int, c_long, c_ulong};
use std::sync::Mutex;

use crate::environment_world::EnvironmentWorld;
use crate::sensors::{SensorKind, SensorReading};

/// 全局引擎表(裸指针 → Box<EnvironmentWorld>)
/// 用 Mutex 保护,Android 主线程 + 渲染线程可能并发。
/// 裸指针不是 Send,但我们手动保证单线程访问 Mutex。
struct EngineTable(Vec<*mut EnvironmentWorld>);
// SAFETY: 我们保证所有访问都在 Mutex 内,单线程
unsafe impl Send for EngineTable {}
unsafe impl Sync for EngineTable {}

static ENGINES: Mutex<EngineTable> = Mutex::new(EngineTable(Vec::new()));

/// 拿到一个引擎的可变引用
fn with_engine<F, R>(ptr: usize, f: F) -> Option<R>
where
    F: FnOnce(&mut EnvironmentWorld) -> R,
{
    let mut table = ENGINES.lock().ok()?;
    let raw = *table.0.get(ptr)?;
    if raw.is_null() {
        return None;
    }
    unsafe { Some(f(&mut *raw)) }
}

/// 创建物理世界
#[no_mangle]
pub extern "C" fn starsos_create() -> c_ulong {
    let mut world = Box::new(EnvironmentWorld::new());
    // 默认场景:3 个动态球 + 地板(用户可以自己加)
    world.physics.backend.add_static_floor(0.0);
    world
        .physics
        .backend
        .add_dynamic_ball([0.0, 2.0, 0.0], 0.3, 1.0);
    world
        .physics
        .backend
        .add_dynamic_ball([1.0, 2.0, 0.0], 0.3, 1.0);
    world
        .physics
        .backend
        .add_dynamic_ball([-1.0, 2.0, 0.0], 0.3, 1.0);

    let raw = Box::into_raw(world);
    let mut table = ENGINES.lock().unwrap();
    let ptr = table.0.len();
    table.0.push(raw);
    ptr as c_ulong
}

/// 销毁
#[no_mangle]
pub extern "C" fn starsos_destroy(ptr: c_ulong) {
    if let Ok(mut table) = ENGINES.lock() {
        if let Some(slot) = table.0.get_mut(ptr as usize) {
            if !slot.is_null() {
                unsafe {
                    let _ = Box::from_raw(*slot);
                }
                *slot = std::ptr::null_mut();
            }
        }
    }
}

/// 推一条传感器数据
#[no_mangle]
pub extern "C" fn starsos_push_sensor(
    ptr: c_ulong,
    kind: c_int,
    x: c_float,
    y: c_float,
    z: c_float,
    ts_nanos: c_long,
) {
    let sensor_kind = match kind {
        0 => SensorKind::Accelerometer,
        1 => SensorKind::Gyroscope,
        2 => SensorKind::Magnetometer,
        3 => SensorKind::AmbientLight,
        4 => SensorKind::Barometer,
        5 => SensorKind::Temperature,
        6 => SensorKind::Proximity,
        _ => return,
    };
    let reading = SensorReading {
        kind: sensor_kind,
        timestamp_ns: ts_nanos as u128,
        vec: [x, y, z],
        scalar: 0.0,
        raw: None,
    };
    with_engine(ptr as usize, |engine| {
        engine.process_reading(&reading);
        engine.apply_events();
    });
}

/// 跑一步物理
#[no_mangle]
pub extern "C" fn starsos_step(ptr: c_ulong, dt: c_float) {
    with_engine(ptr as usize, |engine| {
        engine.real_dt = dt;
        engine.physics.step(dt);
        engine.step_count += 1;
    });
}

/// 拿世界统计
#[no_mangle]
pub extern "C" fn starsos_get_stats(ptr: c_ulong) -> *const c_char {
    let stats = with_engine(ptr as usize, |engine| {
        let s = engine.stats();
        format!(
            "step={} sensors={} bodies={} joints={} events={} temp={:.1}",
            s.step_count, s.sensor_count, s.body_count, s.joint_count, s.event_count, s.current_temp
        )
    });
    match stats {
        Some(s) => {
            let cstr = CString::new(s).unwrap();
            cstr.into_raw()
        }
        None => std::ptr::null(),
    }
}

/// 拿世界状态
#[no_mangle]
pub extern "C" fn starsos_get_state(ptr: c_ulong) -> *const c_char {
    let result = with_engine(ptr as usize, |engine| {
        let rigid_bodies = engine.physics.backend.rigid_bodies();
        let mut s = String::from("{\n  bodies: [\n");
        for (i, (handle, rb)) in rigid_bodies.iter().enumerate() {
            if i > 0 {
                s.push_str(",\n");
            }
            let t = rb.translation();
            let v = rb.linvel();
            let is_dyn = rb.is_dynamic();
            s.push_str(&format!(
                "    {{id: {:?}, pos: [{:.2}, {:.2}, {:.2}], vel: [{:.2}, {:.2}, {:.2}], dynamic: {}}}",
                handle, t.x, t.y, t.z, v.x, v.y, v.z, is_dyn
            ));
        }
        s.push_str("\n  ]\n}");
        s
    });
    match result {
        Some(s) => {
            let cstr = CString::new(s).unwrap();
            cstr.into_raw()
        }
        None => std::ptr::null(),
    }
}

/// 拿传感器统计
#[no_mangle]
pub extern "C" fn starsos_get_sensor_stats(ptr: c_ulong) -> *const c_char {
    let result = with_engine(ptr as usize, |engine| {
        format!(
            "accel_history={} mic_history={} last_gps={:?} last_mic={:?}",
            engine.accel_history.len(),
            engine.mic_history.len(),
            engine.current_gps,
            engine.mic_history.last()
        )
    });
    match result {
        Some(s) => {
            let cstr = CString::new(s).unwrap();
            cstr.into_raw()
        }
        None => std::ptr::null(),
    }
}

/// 释放 C 字符串
#[no_mangle]
pub extern "C" fn starsos_free_string(s: *mut c_char) {
    if s.is_null() {
        return;
    }
    unsafe {
        let _ = CString::from_raw(s);
    }
}

/// 内部使用
#[allow(dead_code)]
pub fn _dummy() {
    let _ = CStr::from_bytes_with_nul(b"hello\0");
}
