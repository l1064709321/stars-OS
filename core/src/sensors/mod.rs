//! 真实环境传感器接口
//!
//! 这是 AGI 操作系统"感知真实世界"的入口。
//! 真实数据从 Android 设备的传感器 / 麦克风 / 摄像头过来,
//! 通过这个 trait 进来,让物理世界模型能用真环境数据驱动。
//!
//! ## 设计
//! - `Sensor` trait:每个传感器实现
//! - `SensorReading`:统一数据格式(时间戳 + 数据)
//! - `SensorHub`:汇总多个传感器,推流到物理世界

use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

/// 传感器类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorKind {
    /// 加速度计 (m/s²)
    Accelerometer,
    /// 陀螺仪 (rad/s)
    Gyroscope,
    /// 磁力计 (μT)
    Magnetometer,
    /// 环境光 (lux)
    AmbientLight,
    /// 距离 (cm)
    Proximity,
    /// 气压 (hPa)
    Barometer,
    /// 温度 (℃)
    Temperature,
    /// 湿度 (%)
    Humidity,
    /// 麦克风 (dB)
    Microphone,
    /// 摄像头 RGB
    CameraRgb,
    /// 摄像头深度
    CameraDepth,
    /// GPS (lat, lon, alt)
    Gps,
}

/// 一个传感器读数
#[derive(Debug, Clone)]
pub struct SensorReading {
    pub kind: SensorKind,
    /// 纳秒时间戳(从 UNIX_EPOCH 起)
    pub timestamp_ns: u128,
    /// 3 维向量(加速度/陀螺仪/磁力计/光/...)
    pub vec: [f32; 3],
    /// 标量(压力/温度/湿度/...)
    pub scalar: f32,
    /// 原始字节(摄像头/麦克风)
    pub raw: Option<Vec<u8>>,
}

impl SensorReading {
    pub fn now_3d(kind: SensorKind, x: f32, y: f32, z: f32) -> Self {
        Self {
            kind,
            timestamp_ns: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0),
            vec: [x, y, z],
            scalar: 0.0,
            raw: None,
        }
    }

    pub fn now_scalar(kind: SensorKind, v: f32) -> Self {
        Self {
            kind,
            timestamp_ns: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0),
            vec: [0.0; 3],
            scalar: v,
            raw: None,
        }
    }
}

/// 传感器 trait(可移植)
pub trait Sensor: Send {
    fn kind(&self) -> SensorKind;
    fn name(&self) -> &str;
    fn read(&mut self) -> Option<SensorReading>;
    fn start(&mut self) -> Result<(), SensorError>;
    fn stop(&mut self) -> Result<(), SensorError>;
    fn is_running(&self) -> bool;
}

/// 传感器错误
#[derive(Debug)]
pub enum SensorError {
    NotAvailable,
    PermissionDenied,
    IoError(String),
}

impl std::fmt::Display for SensorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SensorError::NotAvailable => write!(f, "sensor not available"),
            SensorError::PermissionDenied => write!(f, "permission denied"),
            SensorError::IoError(e) => write!(f, "io error: {}", e),
        }
    }
}

impl std::error::Error for SensorError {}

// ============================================================
// 模拟传感器(测试用,真传感器在 android_bridge 里)
// ============================================================

/// 模拟加速度计(产生有噪声的重力 + 运动)
pub struct SimulatedAccelerometer {
    running: bool,
    t: f32,
}

impl SimulatedAccelerometer {
    pub fn new() -> Self {
        Self {
            running: false,
            t: 0.0,
        }
    }
}

impl Default for SimulatedAccelerometer {
    fn default() -> Self {
        Self::new()
    }
}

impl Sensor for SimulatedAccelerometer {
    fn kind(&self) -> SensorKind {
        SensorKind::Accelerometer
    }
    fn name(&self) -> &str {
        "sim_accel"
    }
    fn read(&mut self) -> Option<SensorReading> {
        if !self.running {
            return None;
        }
        self.t += 0.01;
        // 重力 + 摆动
        let x = (self.t * 1.5).sin() * 0.3;
        let y = -9.81 + (self.t * 0.8).cos() * 0.1;
        let z = (self.t * 2.1).sin() * 0.2;
        Some(SensorReading::now_3d(SensorKind::Accelerometer, x, y, z))
    }
    fn start(&mut self) -> Result<(), SensorError> {
        self.running = true;
        self.t = 0.0;
        Ok(())
    }
    fn stop(&mut self) -> Result<(), SensorError> {
        self.running = false;
        Ok(())
    }
    fn is_running(&self) -> bool {
        self.running
    }
}

/// 模拟 GPS(随机游走)
pub struct SimulatedGps {
    running: bool,
    lat: f64,
    lon: f64,
    alt: f64,
    t: u64,
}

impl SimulatedGps {
    pub fn new(start_lat: f64, start_lon: f64) -> Self {
        Self {
            running: false,
            lat: start_lat,
            lon: start_lon,
            alt: 0.0,
            t: 0,
        }
    }
}

impl Default for SimulatedGps {
    fn default() -> Self {
        Self::new(39.9042, 116.4074) // 北京
    }
}

impl Sensor for SimulatedGps {
    fn kind(&self) -> SensorKind {
        SensorKind::Gps
    }
    fn name(&self) -> &str {
        "sim_gps"
    }
    fn read(&mut self) -> Option<SensorReading> {
        if !self.running {
            return None;
        }
        self.t += 1;
        // 简单随机游走
        let hash = self.t.wrapping_mul(0x9E3779B97F4A7C15);
        self.lat += ((hash & 0xFFFF) as f64 / 65535.0 - 0.5) * 0.0001;
        self.lon += (((hash >> 16) & 0xFFFF) as f64 / 65535.0 - 0.5) * 0.0001;
        self.alt += 0.0;
        Some(SensorReading {
            kind: SensorKind::Gps,
            timestamp_ns: self.t as u128 * 1_000_000_000,
            vec: [self.lat as f32, self.lon as f32, self.alt as f32],
            scalar: 0.0,
            raw: None,
        })
    }
    fn start(&mut self) -> Result<(), SensorError> {
        self.running = true;
        Ok(())
    }
    fn stop(&mut self) -> Result<(), SensorError> {
        self.running = false;
        Ok(())
    }
    fn is_running(&self) -> bool {
        self.running
    }
}

/// 模拟麦克风(产生有噪声的音频电平)
pub struct SimulatedMicrophone {
    running: bool,
    t: u64,
    level_db: f32,
}

impl SimulatedMicrophone {
    pub fn new() -> Self {
        Self {
            running: false,
            t: 0,
            level_db: -60.0,
        }
    }
}

impl Default for SimulatedMicrophone {
    fn default() -> Self {
        Self::new()
    }
}

impl Sensor for SimulatedMicrophone {
    fn kind(&self) -> SensorKind {
        SensorKind::Microphone
    }
    fn name(&self) -> &str {
        "sim_mic"
    }
    fn read(&mut self) -> Option<SensorReading> {
        if !self.running {
            return None;
        }
        self.t += 1;
        // 模拟环境噪声 50-70 dB,偶尔有声音 80-100 dB
        let hash = self.t.wrapping_mul(0x123456789ABCDEF0);
        let r = ((hash >> 8) & 0xFF) as f32 / 255.0;
        self.level_db = 50.0 + r * 30.0;
        Some(SensorReading {
            kind: SensorKind::Microphone,
            timestamp_ns: (self.t as u128) * 16_666_667, // 60 Hz
            vec: [0.0; 3],
            scalar: self.level_db,
            raw: None,
        })
    }
    fn start(&mut self) -> Result<(), SensorError> {
        self.running = true;
        Ok(())
    }
    fn stop(&mut self) -> Result<(), SensorError> {
        self.running = false;
        Ok(())
    }
    fn is_running(&self) -> bool {
        self.running
    }
}

// ============================================================
// 传感器中枢
// ============================================================

/// 传感器中枢:管理多个传感器
pub struct SensorHub {
    sensors: HashMap<SensorKind, Box<dyn Sensor>>,
    /// 最近一次读数
    last_readings: HashMap<SensorKind, SensorReading>,
    /// 推流回调
    subscribers: Vec<Box<dyn FnMut(&SensorReading) + Send>>,
}

impl SensorHub {
    pub fn new() -> Self {
        Self {
            sensors: HashMap::new(),
            last_readings: HashMap::new(),
            subscribers: Vec::new(),
        }
    }

    /// 注册一个传感器
    pub fn register(&mut self, sensor: Box<dyn Sensor>) {
        self.sensors.insert(sensor.kind(), sensor);
    }

    /// 启动所有
    pub fn start_all(&mut self) {
        for s in self.sensors.values_mut() {
            let _ = s.start();
        }
    }

    /// 停止所有
    pub fn stop_all(&mut self) {
        for s in self.sensors.values_mut() {
            let _ = s.stop();
        }
    }

    /// 读一个传感器
    pub fn read(&mut self, kind: SensorKind) -> Option<SensorReading> {
        let s = self.sensors.get_mut(&kind)?;
        if let Some(r) = s.read() {
            // 推流给订阅者
            for sub in &mut self.subscribers {
                sub(&r);
            }
            self.last_readings.insert(kind, r.clone());
            return Some(r);
        }
        None
    }

    /// 读所有运行的传感器
    pub fn poll_all(&mut self) -> Vec<SensorReading> {
        let kinds: Vec<SensorKind> = self.sensors.keys().copied().collect();
        let mut out = Vec::new();
        for k in kinds {
            if let Some(r) = self.read(k) {
                out.push(r);
            }
        }
        out
    }

    /// 最近读数
    pub fn last(&self, kind: SensorKind) -> Option<&SensorReading> {
        self.last_readings.get(&kind)
    }

    /// 添加推流订阅者
    pub fn subscribe<F: FnMut(&SensorReading) + Send + 'static>(&mut self, f: F) {
        self.subscribers.push(Box::new(f));
    }

    pub fn sensor_count(&self) -> usize {
        self.sensors.len()
    }
}

impl Default for SensorHub {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// 测试
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulated_accel_produces_data() {
        let mut s = SimulatedAccelerometer::new();
        s.start().unwrap();
        let r = s.read().unwrap();
        // 应该感受到重力(负 y 方向)
        assert!(r.vec[1] < 0.0);
        assert_eq!(r.kind, SensorKind::Accelerometer);
    }

    #[test]
    fn test_simulated_gps_walks() {
        let mut s = SimulatedGps::default();
        s.start().unwrap();
        let r1 = s.read().unwrap();
        let r2 = s.read().unwrap();
        // 两次读数应该不同
        assert!(r1.vec[0] != r2.vec[0] || r1.vec[1] != r2.vec[1]);
    }

    #[test]
    fn test_simulated_mic() {
        let mut s = SimulatedMicrophone::new();
        s.start().unwrap();
        let r = s.read().unwrap();
        assert_eq!(r.kind, SensorKind::Microphone);
        assert!(r.scalar > 0.0);
    }

    #[test]
    fn test_sensor_hub_routes() {
        let mut hub = SensorHub::new();
        hub.register(Box::new(SimulatedAccelerometer::new()));
        hub.register(Box::new(SimulatedGps::default()));
        hub.start_all();
        let r1 = hub.read(SensorKind::Accelerometer);
        let r2 = hub.read(SensorKind::Gps);
        assert!(r1.is_some());
        assert!(r2.is_some());
    }

    #[test]
    fn test_sensor_hub_poll_all() {
        let mut hub = SensorHub::new();
        hub.register(Box::new(SimulatedAccelerometer::new()));
        hub.register(Box::new(SimulatedMicrophone::new()));
        hub.start_all();
        let readings = hub.poll_all();
        assert_eq!(readings.len(), 2);
    }

    #[test]
    fn test_sensor_hub_subscribe() {
        use std::sync::{Arc, Mutex};
        let counter = Arc::new(Mutex::new(0));
        let c2 = counter.clone();
        let mut hub = SensorHub::new();
        hub.register(Box::new(SimulatedAccelerometer::new()));
        hub.subscribe(move |_r| {
            *c2.lock().unwrap() += 1;
        });
        hub.start_all();
        hub.poll_all();
        hub.poll_all();
        let n = *counter.lock().unwrap();
        assert!(n >= 2);
    }

    #[test]
    fn test_sensor_kind_distinct() {
        assert_ne!(SensorKind::Accelerometer, SensorKind::Gyroscope);
        assert_eq!(SensorKind::Accelerometer, SensorKind::Accelerometer);
    }
}
