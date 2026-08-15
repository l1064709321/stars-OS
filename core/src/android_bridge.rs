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

// =====================================================================
// AGI 核心状态(简化版,只用现有公开 API)
// 所有 62 模块在同一个 .so 里,Java 端通过 starsos_agi_* 系列 JNI 调用
// =====================================================================

// =====================================================================
// AGI 核心状态(简化版,只用现有公开 API)
// 所有 62 模块在同一个 .so 里,Java 端通过 starsos_agi_* 系列 JNI 调用
// =====================================================================

use crate::genuine_emergence::{EmergenceEngine, Evidence, EvidenceKind};
use crate::genuine_concept::linear_fit;
use crate::predictive_coding::PredictiveLayer;
use crate::plasticity_lif::PlasticLifNetwork;
use crate::causal_full::{CausalGraph, NodeId};
use crate::ewc::EwcNetwork;
use crate::memory_io::FileMemoryInjector;
use crate::reasoning::ReasoningLoop;
use crate::reflection::SelfReflector;
use crate::nlp::NluEngine;
use crate::sandbox_genuine::SandboxGenuineBridge;
use crate::emergence::indicators::EmergenceIndicators;

/// AGI 状态:11 个核心模块的状态打包
pub struct AgiState {
    pub emergence: EmergenceEngine,
    pub predictive: PredictiveLayer,
    pub lif: PlasticLifNetwork,
    pub causal: CausalGraph,
    pub ewc: EwcNetwork,
    pub memory: FileMemoryInjector,
    pub reasoning: ReasoningLoop,
    pub reflection: SelfReflector,
    pub nlp: NluEngine,
    pub sandbox: SandboxGenuineBridge,
    pub indicators: EmergenceIndicators,
    pub tick: u64,
    pub evidence_history: Vec<(u64, [f32; 3])>,
    pub emergence_count: usize,
    pub concept_slope: f32,
    pub concept_r2: f32,
}

impl AgiState {
    fn new() -> Self {
        Self {
            emergence: EmergenceEngine::new(100),
            predictive: PredictiveLayer::new(3, 3),
            lif: PlasticLifNetwork::new(),
            causal: CausalGraph::new(),
            ewc: EwcNetwork::new(3, 3),
            memory: FileMemoryInjector::new(64),
            reasoning: ReasoningLoop::new(16),
            reflection: SelfReflector::new(),
            nlp: NluEngine::new(),
            sandbox: SandboxGenuineBridge::new(100),
            indicators: EmergenceIndicators::new(),
            tick: 0,
            evidence_history: Vec::new(),
            emergence_count: 0,
            concept_slope: 0.0,
            concept_r2: 0.0,
        }
    }
}

// 用同一个 Mutex 容器存所有 AGI 状态
struct AgiTable(Vec<*mut AgiState>);
unsafe impl Send for AgiTable {}
unsafe impl Sync for AgiTable {}

static AGI_STATES: Mutex<AgiTable> = Mutex::new(AgiTable(Vec::new()));

fn with_agi<F, R>(ptr: usize, f: F) -> Option<R>
where
    F: FnOnce(&mut AgiState) -> R,
{
    let mut table = AGI_STATES.lock().ok()?;
    let raw = *table.0.get(ptr)?;
    if raw.is_null() {
        return None;
    }
    unsafe { Some(f(&mut *raw)) }
}

/// 创建 AGI 核心(返回 ID)
#[no_mangle]
pub extern "C" fn starsos_agi_create() -> c_ulong {
    let raw = Box::into_raw(Box::new(AgiState::new()));
    let mut table = AGI_STATES.lock().unwrap();
    let id = table.0.len();
    table.0.push(raw);
    id as c_ulong
}

/// 销毁
#[no_mangle]
pub extern "C" fn starsos_agi_destroy(id: c_ulong) {
    if let Ok(mut table) = AGI_STATES.lock() {
        if let Some(slot) = table.0.get_mut(id as usize) {
            if !slot.is_null() {
                unsafe {
                    let _ = Box::from_raw(*slot);
                }
                *slot = std::ptr::null_mut();
            }
        }
    }
}

/// 喂一个真环境观察给 AGI(传感器数据 → 涌现引擎 + 预测编码 + 记忆)
#[no_mangle]
pub extern "C" fn starsos_agi_observe(
    id: c_ulong,
    _modality: c_int,
    x: c_float,
    y: c_float,
    z: c_float,
) {
    with_agi(id as usize, |agi| {
        // 累积证据历史
        agi.evidence_history.push((agi.tick, [x, y, z]));
        if agi.evidence_history.len() > 256 {
            agi.evidence_history.remove(0);
        }
        // 推给涌现引擎
        let ev = Evidence {
            source: agi.tick,
            kind: EvidenceKind::ClusterFit,
            strength: 0.5,
            tick: agi.tick,
        };
        // 2 参数版本(EmergenceEngine::add_evidence)
        agi.emergence.add_evidence(agi.tick, ev);
        // 推给预测编码
        agi.predictive.forward(&[x, y, z]);
        // 因果图加边:前一个 tick → 当前
        if agi.tick > 0 {
            agi.causal.add_edge(
                NodeId((agi.tick - 1) as u32),
                NodeId(agi.tick as u32),
                0.5,
            );
        }
        agi.tick += 1;
    });
}

/// 跑一步 AGI 推理(涌现检测 + 概念发现 + 反思 + 推理循环)
#[no_mangle]
pub extern "C" fn starsos_agi_step(id: c_ulong) -> *const c_char {
    let result = with_agi(id as usize, |agi| {
        // 涌现检测
        let sigs = agi.indicators.detect();
        agi.emergence_count = sigs.len();
        // 概念发现
        if agi.evidence_history.len() >= 8 {
            let last8: Vec<f32> = agi.evidence_history.iter().rev().take(8)
                .map(|(_, v)| v[0]).collect();
            let last8y: Vec<f32> = agi.evidence_history.iter().rev().take(8)
                .map(|(_, v)| v[1]).collect();
            if let Some(fit) = linear_fit(&last8, &last8y) {
                agi.concept_slope = fit.slope;
                agi.concept_r2 = fit.r_squared;
            }
        }
        // 反思
        let _reflect = agi.reflection.reflect(16);
        // 推理循环 - 用 propose + strongest
        if agi.tick % 10 == 0 {
            let text = format!("tick_{}", agi.tick);
            agi.reasoning.propose(text, 0.5);
        }
        format!(
            "{{\"tick\":{},\"emergence_signals\":{},\"concept_slope\":{:.3},\"concept_r2\":{:.3},\"hypotheses\":{}}}",
            agi.tick,
            sigs.len(),
            agi.concept_slope,
            agi.concept_r2,
            agi.reasoning.len()
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

/// AGI 状态(JSON 摘要)
#[no_mangle]
pub extern "C" fn starsos_agi_status(id: c_ulong) -> *const c_char {
    let result = with_agi(id as usize, |agi| {
        format!(
            "{{\"tick\":{},\"evidence_count\":{},\"emergence_signals\":{},\"causal_nodes\":{},\"memory_count\":{}}}",
            agi.tick,
            agi.evidence_history.len(),
            agi.emergence_count,
            agi.causal.nodes().count(),
            agi.memory.len()
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

/// NLP 文本处理(让 AGI 真的"理解"你说的)
#[no_mangle]
pub extern "C" fn starsos_agi_nlp(id: c_ulong, text: *const c_char) -> *const c_char {
    let text_str = unsafe {
        if text.is_null() {
            return std::ptr::null();
        }
        match CStr::from_ptr(text).to_str() {
            Ok(s) => s.to_string(),
            Err(_) => return std::ptr::null(),
        }
    };
    let result = with_agi(id as usize, |agi| {
        let nlu = agi.nlp.understand(&text_str);
        format!(
            "{{\"intent\":{},\"slots\":{}}}",
            nlu.intent.as_str(),
            nlu.slots.len()
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

/// 写记忆(让 AGI 记住某事)
#[no_mangle]
pub extern "C" fn starsos_agi_remember(
    id: c_ulong,
    _layer: c_int,
    key: *const c_char,
    value: *const c_char,
) {
    let key_str = unsafe {
        if key.is_null() {
            return;
        }
        CStr::from_ptr(key).to_str().unwrap_or("").to_string()
    };
    let value_str = unsafe {
        if value.is_null() {
            return;
        }
        CStr::from_ptr(value).to_str().unwrap_or("").to_string()
    };
    with_agi(id as usize, |agi| {
        // 合并 key=value 当作 source|content 注入
        agi.memory.inject_text(key_str, value_str);
    });
}

/// 读记忆(简单通过 entries 找匹配的)
#[no_mangle]
pub extern "C" fn starsos_agi_recall(
    id: c_ulong,
    key: *const c_char,
) -> *const c_char {
    let key_str = unsafe {
        if key.is_null() {
            return std::ptr::null();
        }
        CStr::from_ptr(key).to_str().unwrap_or("").to_string()
    };
    let result = with_agi(id as usize, |agi| {
        // 从已有 entries 里找匹配的
        let entries = agi.memory.all_entries();
        for e in entries {
            if e.source.contains(&key_str) || e.content.contains(&key_str) {
                return format!("{}: {}", e.source, e.content);
            }
        }
        String::from("(not found)")
    });
    match result {
        Some(s) => {
            let cstr = CString::new(s).unwrap();
            cstr.into_raw()
        }
        None => std::ptr::null(),
    }
}

/// 内部使用
#[allow(dead_code)]
pub fn _dummy() {
    let _ = CStr::from_bytes_with_nul(b"hello\0");
}
