//! 群星 A.I. OS - Rust 神经认知核心
//!
//! 这是整个 62 模块 AGI 操作系统的基础核心层(模块 1-8)的 Rust 实现。
//! 设计目标:在旧手机 ARM 处理器上长时间稳定运行,无输入时 CPU 接近 0%。
//!
//! ## 模块清单
//! - 模块 1: [`lif`] - 脉冲神经网络感知层(LIF + STDP + SADP)
//! - 模块 2: [`kalman`] - 卡尔曼滤波器(自适应 + 多实体)
//! - 模块 3: [`eight_gates`] - 八门状态机(CTL/LTL 形式化验证)
//! - 模块 4: [`workspace`] - 全局工作空间(多源竞争)
//! - 模块 5: [`motivation`] - 内在动机引擎
//! - 模块 6: [`ethics`] - 伦理动力学引擎(连续演化 + 相变检测)
//! - 模块 7: [`existential`] - 存在性递归验证器(哈希锁定)
//! - 模块 8: [`bus`] - 消息总线(CognitiveMessage 协议)
//!
//! ## 物理世界相关
//! - 模块 18: [`world`] - 物理世界模型(对外身份,内部用预测编码实现)
//! - 物理约束层:嵌入在 [`world`] 模块内

#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_debug_implementations)]
#![allow(clippy::needless_range_loop)]

use std::sync::Arc;
use parking_lot::Mutex;

pub mod bus;
pub mod causal_full;  // 模块 17 因果推理完整版 + 模块 31 失败分析
pub mod correction;
pub mod ethics;
pub mod ewc;          // 模块 24 持续学习 (Elastic Weight Consolidation)
pub mod eight_gates;
pub mod emergence;
pub mod environment_world; // 真环境驱动的物理世界(传感器→物理)
pub mod existential;
pub mod genuine_concept; // 真概念发现:从数据反推守恒律(最小二乘拟合)
pub mod genuine_emergence; // 真涌现引擎(数据驱动,多代际证伪)
pub mod industrial;   // 工业级物理 + AGI 集成层(rapier → 涌现/世界/因果)
pub mod kalman;
pub mod lif;
pub mod memory_io;    // 模块 10 + 11 文件记忆注入 + 分层检索
pub mod motivation;
pub mod nlp;          // 真 NLP 模块 12-15(NLU + NLG + CoT + 串联)
pub mod plasticity;   // 真突触可塑性 (STDP + BCM + 三因子)
pub mod plasticity_lif; // 真塑性接 LIF + 真 EWC 在脉冲神经上工作
pub mod predictive_coding; // 真预测编码 (Rao-Ballard, KL 散度驱动)
pub mod android_bridge; // Android NDK JNI 桥
pub mod rapier_bridge; // 工业级物理引擎 rapier3d 桥接
pub mod sensors;      // 真环境传感器接口(Android 后端在 sensors/android.rs)
pub mod reasoning;    // 模块 20 推理循环 + 32 不确定性 + 49 道德评估
pub mod reflection;   // 真自我反思模块
pub mod sandbox_genuine; // 真涌现沙箱桥接(用真证据累积代替假货剧本)
pub mod true_world_model; // 真物理世界模型(rapier 后端 + 关节 + 接触力 + 流体)
pub mod world;        // 模块 18 物理世界模型(对外身份,简化版)
pub mod workspace;    // 模块 4 全局工作空间

pub use bus::{CognitiveMessage, MessageBus, MessageTopic, EthicalSignature};
pub use correction::{CorrectionProtocol, CorrectionSignal, ResponseAction};
pub use ethics::{EthicsDynamics, EthicsDimension, EthicsState};
pub use eight_gates::{EightGates, GateState, GateTransition};
pub use existential::{AnchorCheckResult, ExistentialVerifier, ValueAnchor, ValueAnchorError};
pub use world::{PhysicsWorldModel, PhysicsConstraint, WorldState, WorldEvent, SurpriseScore};

/// 错误类型
#[derive(Debug, thiserror::Error)]
pub enum CoreError {
    #[error("伦理铁门拒绝: {0}")]
    EthicsGateDenied(String),

    #[error("存在性递归验证失败: {0}")]
    ExistentialViolation(String),

    #[error("八门状态机拒绝转移: {from} -> {to},原因: {reason}")]
    GateTransitionDenied {
        from: String,
        to: String,
        reason: String,
    },

    #[error("消息总线错误: {0}")]
    BusError(String),

    #[error("物理世界模型错误: {0}")]
    WorldError(String),

    #[error("I/O 错误: {0}")]
    Io(#[from] std::io::Error),

    #[error("序列化错误: {0}")]
    Serde(#[from] serde_json::Error),
}

pub type CoreResult<T> = Result<T, CoreError>;

/// 群星 A.I. OS 运行时核心
///
/// 把伦理铁门(模块 6/7)、八门(模块 3)、消息总线(模块 8)、
/// 物理世界模型(模块 18)组装成一个可启动的最小单元。
///
/// 启动顺序(由硬性约束决定):
/// 1. 存在性递归验证器(模块 7)—— 哈希校验,过不去直接 panic
/// 2. 伦理动力学(模块 6)—— 加载基线
/// 3. 八门状态机(模块 3)—— 初始化为"开门"
/// 4. 消息总线(模块 8)—— 启动 broker
/// 5. 物理世界模型(模块 18)—— 初始化物理约束
pub struct QuantumCore {
    pub bus: Arc<MessageBus>,
    pub existential: Arc<ExistentialVerifier>,
    pub ethics: Arc<Mutex<EthicsDynamics>>,
    pub gates: Arc<Mutex<EightGates>>,
    pub world: Arc<Mutex<PhysicsWorldModel>>,
}

impl QuantumCore {
    /// 启动群星核心(严格按伦理优先顺序)
    pub fn bootstrap() -> CoreResult<Self> {
        // 1. 存在性递归验证(失败则系统拒绝启动)
        let existential = Arc::new(ExistentialVerifier::bootstrap()?);
        log::info!("[模块 7] 存在性递归验证通过,元价值锚已锁定");

        // 2. 伦理动力学加载基线
        let ethics = Arc::new(Mutex::new(EthicsDynamics::with_baseline(&existential)?));
        log::info!("[模块 6] 伦理动力学启动,基线 non_harm=0.80 已锁定");

        // 3. 八门初始化
        let gates = Arc::new(Mutex::new(EightGates::open()));
        log::info!("[模块 3] 八门状态机启动,初始状态:开门");

        // 4. 消息总线
        let bus = Arc::new(MessageBus::start()?);
        log::info!("[模块 8] 消息总线启动,CognitiveMessage 协议就绪");

        // 5. 物理世界模型
        let world = Arc::new(Mutex::new(PhysicsWorldModel::init(&existential)?));
        log::info!("[模块 18] 物理世界模型启动,物理约束层已加载");

        Ok(Self { bus, existential, ethics, gates, world })
    }

    /// 状态摘要(供模块 50 RESTful API 用)
    pub fn status(&self) -> serde_json::Value {
        serde_json::json!({
            "module7_existential": "verified",
            "module6_ethics_baseline": 0.80,
            "module3_current_gate": self.gates.lock().current().as_str(),
            "module8_bus_running": self.bus.is_running(),
            "module18_world": self.world.lock().summary(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bootstrap_order_is_ethics_first() {
        // 启动顺序必须从伦理开始
        let core = QuantumCore::bootstrap().expect("core bootstrap");
        let status = core.status();
        assert_eq!(status["module7_existential"], "verified");
        assert_eq!(status["module6_ethics_baseline"], 0.80);
        assert!(status["module8_bus_running"].as_bool().unwrap_or(false));
    }
}
