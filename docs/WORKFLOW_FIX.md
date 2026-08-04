# ⚠️ Workflow 需要修复

## 背景
当前 token 缺少 `workflow` scope,无法 push .github/workflows/ 下的文件。
仓库目前 commit `8d090d0` 已删除旧 workflow 文件,需要重建。

## 修复步骤(10 秒)

打开 https://github.com/l1064709321/stars-OS/blob/main/.github/workflows/android-build.yml

如果文件不存在(因为已被删除),点 "Create new file":
- Path: `.github/workflows/android-build.yml`
- 内容粘贴下面的完整 YAML

如果文件存在(空文件或损坏),点编辑按钮,替换为:

```yaml
name: Android APK Build

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-latest
    timeout-minutes: 60

    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
      - run: rustup target add aarch64-linux-android
      - uses: nttld/setup-ndk@v1
        with:
          ndk-version: r26b
      - uses: actions/setup-java@v4
        with:
          distribution: temurin
          java-version: '17'
      - uses: android-actions/setup-android@v3
      - uses: gradle/actions/setup-gradle@v3

      - name: Configure Cargo linker
        run: |
          LINKER_PATH="$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android26-clang"
          echo "CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER=$LINKER_PATH" >> $GITHUB_ENV
          mkdir -p .cargo
          cat > .cargo/config.toml <<EOF
          [target.aarch64-linux-android]
          linker = "$LINKER_PATH"
          EOF

      - name: Build Rust .so
        run: |
          cargo build -p quantum-core --release --target aarch64-linux-android 2>&1 | tail -30

      - name: Copy libquantum_core.so
        run: |
          mkdir -p android/app/src/main/jniLibs/arm64-v8a
          cp target/aarch64-linux-android/release/libquantum_core.so android/app/src/main/jniLibs/arm64-v8a/

      - name: Build APK
        working-directory: android
        run: |
          gradle wrapper --gradle-version 8.7
          chmod +x gradlew
          ./gradlew assembleDebug --no-daemon 2>&1 | tail -50

      - uses: actions/upload-artifact@v4
        with:
          name: starsos-debug
        path: android/app/build/outputs/apk/debug/app-debug.apk
```

提交后,GitHub Actions 自动触发 build,或在 https://github.com/l1064709321/stars-OS/actions 手动 Run workflow。
