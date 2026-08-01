package com.starsos.agi;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * 群星 A.I. OS Android 入口
 *
 * 真实环境传感器 → 物理世界(rapier) → 推流到 AGI 核心
 */
public class StarsOSActivity extends Activity implements SensorEventListener {
    private static final String TAG = "StarsOS";

    private SensorManager sensorManager;
    private Sensor accelerometer;
    private Sensor gyroscope;
    private Sensor magnetometer;
    private Sensor ambientLight;
    private Sensor barometer;
    private Sensor temperature;
    private Sensor proximity;

    private TextView statusView;
    private TextView accelView;
    private TextView micView;

    private MediaRecorder micRecorder;
    private boolean micRecording = false;

    /** 真实环境数据采集缓冲 */
    private final List<float[]> accelBuffer = new ArrayList<>();
    private final List<float[]> gyroBuffer = new ArrayList<>();
    private final List<float[]> magBuffer = new ArrayList<>();
    private final List<Float> lightBuffer = new ArrayList<>();
    private final List<Float> pressureBuffer = new ArrayList<>();
    private final List<Float> tempBuffer = new ArrayList<>();
    private final List<Float> proximityBuffer = new ArrayList<>();
    private long startTimeNanos;

    /** 真实物理引擎指针(由 Rust side 创建) */
    private long nativeEnginePtr = 0;

    static {
        System.loadLibrary("starsos");
    }

    /** 加载 native 库(自动 init rapier) */
    private native long nativeCreate();
    private native void nativeDestroy(long ptr);
    /** 推一条传感器数据到 native 物理引擎 */
    private native void nativePushSensor(long ptr, int kind, float x, float y, float z, long timestampNanos);
    /** 跑一步物理 */
    private native void nativeStep(long ptr, float dt);
    /** 拿状态(JSON 字符串) */
    private native String nativeGetState(long ptr);
    /** 把真实 dt 推给 native 物理引擎 */
    private native void nativeSetDt(long ptr, float dt);
    /** 拿世界统计 */
    private native String nativeGetStats(long ptr);
    /** 拿统计的字符串 */
    private native String nativeGetSensorStats(long ptr);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        statusView = (TextView) findViewById(R.id.status);
        accelView = (TextView) findViewById(R.id.accel);
        micView = (TextView) findViewById(R.id.mic);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        ambientLight = sensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
        barometer = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        temperature = sensorManager.getDefaultSensor(Sensor.TYPE_AMBIENT_TEMPERATURE);
        proximity = sensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);

        startTimeNanos = System.nanoTime();
        nativeEnginePtr = nativeCreate();

        statusView.setText("群星 A.I. OS 已启动\n");
        Log.i(TAG, "StarsOS created, native engine: " + nativeEnginePtr);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (accelerometer != null) {
            sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
        }
        if (gyroscope != null) {
            sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        }
        if (magnetometer != null) {
            sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
        }
        if (ambientLight != null) {
            sensorManager.registerListener(this, ambientLight, SensorManager.SENSOR_DELAY_NORMAL);
        }
        if (barometer != null) {
            sensorManager.registerListener(this, barometer, SensorManager.SENSOR_DELAY_NORMAL);
        }
        if (temperature != null) {
            sensorManager.registerListener(this, temperature, SensorManager.SENSOR_DELAY_NORMAL);
        }
        if (proximity != null) {
            sensorManager.registerListener(this, proximity, SensorManager.SENSOR_DELAY_NORMAL);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (nativeEnginePtr != 0) {
            nativeDestroy(nativeEnginePtr);
            nativeEnginePtr = 0;
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (nativeEnginePtr == 0) return;
        long ts = event.timestamp;
        // 1. 把真实数据推到 native
        // 类别映射:0=accel, 1=gyro, 2=mag, 3=light, 4=pressure, 5=temp, 6=proximity
        int kind = -1;
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER: kind = 0; break;
            case Sensor.TYPE_GYROSCOPE: kind = 1; break;
            case Sensor.TYPE_MAGNETIC_FIELD: kind = 2; break;
            case Sensor.TYPE_LIGHT: kind = 3; break;
            case Sensor.TYPE_PRESSURE: kind = 4; break;
            case Sensor.TYPE_AMBIENT_TEMPERATURE: kind = 5; break;
            case Sensor.TYPE_PROXIMITY: kind = 6; break;
            default: return;
        }
        float x = event.values.length > 0 ? event.values[0] : 0f;
        float y = event.values.length > 1 ? event.values[1] : 0f;
        float z = event.values.length > 2 ? event.values[2] : 0f;
        nativePushSensor(nativeEnginePtr, kind, x, y, z, ts);

        // 2. 缓冲
        switch (kind) {
            case 0:  // accel
                accelBuffer.add(new float[]{x, y, z});
                if (accelBuffer.size() > 100) accelBuffer.remove(0);
                if (accelView != null) {
                    accelView.setText(String.format("加速度: x=%.2f y=%.2f z=%.2f m/s²", x, y, z));
                }
                break;
            case 1:  // gyro
                gyroBuffer.add(new float[]{x, y, z});
                if (gyroBuffer.size() > 100) gyroBuffer.remove(0);
                break;
            case 2:  // mag
                magBuffer.add(new float[]{x, y, z});
                if (magBuffer.size() > 100) magBuffer.remove(0);
                break;
            case 3: lightBuffer.add(x); if (lightBuffer.size() > 50) lightBuffer.remove(0); break;
            case 4: pressureBuffer.add(x); if (pressureBuffer.size() > 50) pressureBuffer.remove(0); break;
            case 5: tempBuffer.add(x); if (tempBuffer.size() > 50) tempBuffer.remove(0); break;
            case 6: proximityBuffer.add(x); if (proximityBuffer.size() > 50) proximityBuffer.remove(0); break;
        }

        // 3. 跑一步物理(60Hz)
        long elapsed = (System.nanoTime() - startTimeNanos) / 1_000_000;
        float dt = elapsed / 1000.0f;
        if (dt > 0) {
            nativeStep(nativeEnginePtr, dt);
            startTimeNanos = System.nanoTime();  // 重置
        }

        // 4. 更新 UI
        if (statusView != null && stepCounter % 60 == 0) {
            statusView.setText("群星 A.I. OS 正在跑\n" +
                "传感器: " + accelBuffer.size() + " accel, " + gyroBuffer.size() + " gyro\n" +
                nativeGetStats(nativeEnginePtr));
        }
        stepCounter++;
    }

    private int stepCounter = 0;

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}
