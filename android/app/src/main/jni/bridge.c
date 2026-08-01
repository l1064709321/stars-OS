// 群星 A.I. OS — JNI 桥
// Java → JNI → Rust 物理引擎
//
// 这个 C 文件只做 JNI 接口转发,实际物理计算在 Rust。

#include <jni.h>
#include <stdint.h>
#include <stdlib.h>

// 这些函数在 Rust side 实现
extern uintptr_t starsos_create(void);
extern void starsos_destroy(uintptr_t engine);
extern void starsos_push_sensor(uintptr_t engine, int32_t kind, float x, float y, float z, int64_t ts_nanos);
extern void starsos_step(uintptr_t engine, float dt);
extern const char* starsos_get_stats(uintptr_t engine);
extern const char* starsos_get_state(uintptr_t engine);
extern const char* starsos_get_sensor_stats(uintptr_t engine);

JNIEXPORT jlong JNICALL
Java_com_starsos_agi_StarsOSActivity_nativeCreate(JNIEnv *env, jobject thisObj) {
    (void)env;
    (void)thisObj;
    uintptr_t p = starsos_create();
    return (jlong)p;
}

JNIEXPORT void JNICALL
Java_com_starsos_agi_StarsOSActivity_nativeDestroy(JNIEnv *env, jobject thisObj, jlong ptr) {
    (void)env;
    (void)thisObj;
    starsos_destroy((uintptr_t)ptr);
}

JNIEXPORT void JNICALL
Java_com_starsos_agi_StarsOSActivity_nativePushSensor(
    JNIEnv *env, jobject thisObj, jlong ptr, jint kind, jfloat x, jfloat y, jfloat z, jlong ts) {
    (void)env;
    (void)thisObj;
    starsos_push_sensor((uintptr_t)ptr, (int32_t)kind, (float)x, (float)y, (float)z, (int64_t)ts);
}

JNIEXPORT void JNICALL
Java_com_starsos_agi_StarsOSActivity_nativeStep(
    JNIEnv *env, jobject thisObj, jlong ptr, jfloat dt) {
    (void)env;
    (void)thisObj;
    starsos_step((uintptr_t)ptr, (float)dt);
}

JNIEXPORT jstring JNICALL
Java_com_starsos_agi_StarsOSActivity_nativeGetStats(
    JNIEnv *env, jobject thisObj, jlong ptr) {
    (void)thisObj;
    const char* s = starsos_get_stats((uintptr_t)ptr);
    return (*env)->NewStringUTF(env, s);
}

JNIEXPORT jstring JNICALL
Java_com_starsos_agi_StarsOSActivity_nativeGetState(
    JNIEnv *env, jobject thisObj, jlong ptr) {
    (void)thisObj;
    const char* s = starsos_get_state((uintptr_t)ptr);
    return (*env)->NewStringUTF(env, s);
}

JNIEXPORT jstring JNICALL
Java_com_starsos_agi_StarsOSActivity_nativeGetSensorStats(
    JNIEnv *env, jobject thisObj, jlong ptr) {
    (void)thisObj;
    const char* s = starsos_get_sensor_stats((uintptr_t)ptr);
    return (*env)->NewStringUTF(env, s);
}
