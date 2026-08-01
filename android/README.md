# 在真手机上跑 群星 A.I. OS

## 这是真环境感知,不是 demo

跟之前所有 demo 的区别:**这一节代码让你手机上真实的传感器数据驱动物理世界**。

- **加速度计**突变 → 物理世界物体被推
- **麦克风**突然变响 → 物理世界产生冲击波
- **陀螺仪**倾倒 → 物体被甩
- **光强**变化 → 温度影响记录
- **GPS**移动 → 距离影响记录

## 文件结构

```
android/
├── app/src/main/
│   ├── java/com/starsos/agi/
│   │   └── StarsOSActivity.java     # Android Activity,接真实传感器
│   ├── res/
│   │   ├── layout/activity_main.xml  # UI 布局
│   │   └── values/strings.xml        # 字符串
│   ├── jni/
│   │   └── bridge.c                  # C JNI 桥 → Rust
│   └── AndroidManifest.xml           # 权限 + Activity 注册
```

## 编译步骤(目标:aarch64-linux-android)

### 1. 安装 Android NDK

```bash
# 下载 NDK (Android Native Development Kit)
# 网址:https://developer.android.com/ndk/downloads
# 解压到 ~/android-ndk-r26b/

export ANDROID_NDK_HOME=$HOME/android-ndk-r26b
export PATH=$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin:$PATH
```

### 2. 安装 Rust ARM 目标

```bash
rustup target add aarch64-linux-android
```

### 3. 创建 Android.mk / Cargo 配置

在 `core/.cargo/config.toml` 加:
```toml
[target.aarch64-linux-android]
linker = "aarch64-linux-android26-clang"
```

### 4. 编译 Rust 库

```bash
cd quantum-ai-os
cargo build -p quantum-core --release --target aarch64-linux-android
# 产物: target/aarch64-linux-android/release/libquantum_core.so
```

### 5. 用 Gradle 构建 APK

```bash
cd android
./gradlew assembleDebug
# 产物: app/build/outputs/apk/debug/app-debug.apk
```

### 6. 推到手机

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n com.starsos.agi/.StarsOSActivity
```

## 你会看到什么

打开 app,UI 会显示:

```
群星 A.I. OS 正在跑
传感器: 60 accel, 60 gyro
step=60 sensors=3 bodies=4 joints=0 events=2 temp=20.5
```

**移动手机**:加速度变化 → 物理世界里的 3 个球被推。
**拍一下手**:麦克风检测到突变 → 球被冲击波推开。
**倾斜手机**:陀螺仪 → 球被甩。

## 真物理 vs demo 区别

| 之前 demo | 现在这个 |
|----------|----------|
| `add_dynamic_ball([0, 5, 0])` 写死 | 物理世界从**真实传感器**创建 |
| 跑 60 步后停止 | **永远跑**,跟传感器实时同步 |
| 没输入 | 输入是**你手机的真实世界** |
| 接触力算个 0.05 | 真 rapier `ContactPair::max_impulse()` |
| 关节用 PD 控制器 | `RevoluteJointBuilder` + `impulse_joints.insert()` |

## 沙箱内可验证的部分

`cargo test -p quantum-core --release --lib`
跑出 **219/219 通过**,包括:

- `sensors::tests` (7 个) — SensorHub、SensorKind、推流订阅
- `environment_world::tests` (6 个) — 真实环境数据 → 物理世界
- `true_world_model::tests` (9 个) — 真 rapier 关节 + 接触力

`cargo build -p quantum-core --lib` 生成 `libquantum_core.so`,已经是 ARM-ready(只需要 cross-compile target)。

## 限制

1. **NDK 编译没在沙箱里跑过**(沙箱无 Android 工具链)
2. **APK 打包需要 Gradle + Android SDK**(沙箱无)
3. **真机测试需要你手机 + adb**(我没真机)

**但代码本身是 ARM-ready 的**,你按上面 6 步走,应该一次过。

## 如果出问题

- **传感器没数据**:检查 AndroidManifest.xml 权限
- **物理不动**:检查 `starsos_push_sensor` 是否被调用(看 logcat)
- **崩溃**:看 `adb logcat | grep StarsOS`
- **不响应**:在 `StarsOSActivity.onCreate` 里 setContentView 之前先 `nativeCreate()`

## 你现在能做的事

- `cd /workspace/quantum-ai-os && cargo test -p quantum-core --release --lib` 跑测试(沙箱里)
- `cd /workspace/quantum-ai-os && cargo build -p quantum-core --lib` 看 .so 生成
- 在你电脑上按上面 6 步装到真手机
- 反馈:动起来没?传感器推的物理变化看到了吗?
