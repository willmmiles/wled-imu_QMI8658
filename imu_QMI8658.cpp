#include "wled.h"
#include "imu_interface.h"

/* QMI8658 driver: reads accel + gyro and provides data to the motion_reactive
   IMU accessor.  Gravity is estimated via a complementary filter (low-pass on
   accel); linear acceleration is accel minus that estimate. */
#include <QMI8658.h>

class QMI8658Mod : public Usermod, public IMUBase {
  private:
    QMI8658 device;

    struct config_t {
      bool    enabled;
      uint8_t address;
      long    update_interval_ms;
      float   filter_tau;   // complementary filter time constant, seconds
      float   pitch_deg;    // misalignment correction: rotation around X (deg)
      float   roll_deg;     // misalignment correction: rotation around Y (deg)
    };
    config_t config = { false, 0x6B, 20, 1.5f, 0.f, 0.f };
    bool configDirty = true;

    QMI8658_Data rawData;
    bool   data_valid  = false;
    decltype(millis()) next_read = 0;

    // Gyro bias measured at startup (raw sensor units, dps)
    float gyroBias[3]  = {0.f, 0.f, 0.f};

    // Corrected (misalignment-adjusted) raw values
    float corrAccel[3] = {0.f, 0.f, 0.f};
    float corrGyro[3]  = {0.f, 0.f, 0.f};

    // Complementary filter state
    // Initialised with gravity pointing in −Z (typical flat-mount orientation).
    float gravEst[3]   = {0.f, 0.f, -9.81f};
    float linAccel[3]  = {0.f,  0.f,  0.f};

    static const char _name[];
    static const char _enabled[];
    static const char _address[];
    static const char _update_interval_ms[];
    static const char _filter_tau[];
    static const char _pitch_deg[];
    static const char _roll_deg[];

    // Collect samples at startup and compute per-axis gyro bias.
    // Assumes the device is stationary; any non-zero mean is treated as offset.
    void calibrateGyro() {
      const int N = 200;
      float sum[3] = {0.f, 0.f, 0.f};
      int   count  = 0;
      QMI8658_Data d;
      for (int i = 0; i < N; i++) {
        if (device.readSensorData(d)) {
          sum[0] += d.gyroX;
          sum[1] += d.gyroY;
          sum[2] += d.gyroZ;
          count++;
        }
        delay(5);
      }
      if (count > 0) {
        for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / count;
        DEBUG_PRINTF_P(PSTR("%s: Gyro bias (dps): %.4f  %.4f  %.4f\n"),
                       FPSTR(_name), gyroBias[0], gyroBias[1], gyroBias[2]);
      }
    }

    // Apply the configured pitch/roll correction rotation to a 3-vector.
    // Uses a full Ry(pitch) × Rx(roll) matrix — correct for any angle.
    void applyCorrection(const float in[3], float out[3]) const {
      float pr = config.pitch_deg * (float)(M_PI / 180.0);
      float rr = config.roll_deg  * (float)(M_PI / 180.0);
      float cp = cosf(pr), sp = sinf(pr);
      float cr = cosf(rr), sr = sinf(rr);
      // Ry(pitch) × Rx(roll):
      out[0] =  cp * in[0] + sp * sr * in[1] + sp * cr * in[2];
      out[1] =               cr * in[1]       - sr       * in[2];
      out[2] = -sp * in[0] + cp * sr * in[1] + cp * cr * in[2];
    }

  public:

    // ── IMUBase interface ────────────────────────────────────────────────────

    bool  isValid()              const override { return config.enabled && data_valid; }
    float accel(int axis)        const override { return corrAccel[axis]; }
    float gyro(int axis)         const override { return corrGyro[axis]; }
    float gravityAccel(int axis) const override { return gravEst[axis]; }
    float linearAccel(int axis)  const override { return linAccel[axis]; }

    // ── Usermod interface ────────────────────────────────────────────────────

    void setup() override {
      device.setAccelUnit_mps2();
      device.setGyroUnit_dps();

      if (i2c_scl < 0 || i2c_sda < 0) { config.enabled = false; }

      configDirty = false;

      if (!config.enabled) return;

      if (!device.begin(Wire, config.address)) {
        DEBUG_PRINTF_P(PSTR("%s: Device did not respond\n"), FPSTR(_name));
        config.enabled = false;
        return;
      }

      calibrateGyro();

      next_read = 0;
      DEBUG_PRINTF_P(PSTR("%s: Device initialized\n"), FPSTR(_name));
    }

    void loop() override {
      if (configDirty) setup();
      if (!config.enabled) return;

      auto now   = millis();
      auto tdiff = static_cast<long>(next_read - now);
      if (tdiff > 0) return;

      data_valid = device.readSensorData(rawData);

      if (data_valid) {
        // Apply misalignment correction to raw sensor values
        float rawA[3] = { rawData.accelX, rawData.accelY, rawData.accelZ };
        float rawG[3] = { rawData.gyroX  - gyroBias[0],
                          rawData.gyroY  - gyroBias[1],
                          rawData.gyroZ  - gyroBias[2] };
        applyCorrection(rawA, corrAccel);
        applyCorrection(rawG, corrGyro);

        // Complementary filter: gravity estimate = low-pass of accel
        float dt    = config.update_interval_ms / 1000.f;
        float alpha = config.filter_tau / (config.filter_tau + dt);
        for (int i = 0; i < 3; i++) {
          gravEst[i]  = alpha * gravEst[i] + (1.f - alpha) * corrAccel[i];
          linAccel[i] = corrAccel[i] - gravEst[i];
        }
      }

      if (tdiff >= -config.update_interval_ms) {
        next_read += config.update_interval_ms;
      } else {
        auto reads_missed = (-tdiff) / config.update_interval_ms;
        next_read += (reads_missed + 1) * config.update_interval_ms;
      }
    }

    void addToJsonInfo(JsonObject& root) override {
      if (!config.enabled) return;

      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      auto& imu_meas = user;

      JsonArray accel_json = imu_meas.createNestedArray("Accel").createNestedArray();
      JsonArray gyro_json  = imu_meas.createNestedArray("Gyro").createNestedArray();
      JsonArray grav_json  = imu_meas.createNestedArray("GravEst").createNestedArray();
      JsonArray lin_json   = imu_meas.createNestedArray("LinAccel").createNestedArray();
      JsonArray temp_json  = imu_meas.createNestedArray("Temp");

      if (data_valid) {
        for (int i = 0; i < 3; i++) accel_json.add(corrAccel[i]);
        for (int i = 0; i < 3; i++) gyro_json.add(corrGyro[i]);
        for (int i = 0; i < 3; i++) grav_json.add(gravEst[i]);
        for (int i = 0; i < 3; i++) lin_json.add(linAccel[i]);
        temp_json.add(rawData.temperature);
        temp_json.add("C");
      } else {
        for (int i = 0; i < 3; i++) {
          accel_json.add("N/A"); gyro_json.add("N/A");
          grav_json.add("N/A");  lin_json.add("N/A");
        }
        temp_json.add("N/A");
      }
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)]            = config.enabled;
      top[FPSTR(_address)]            = config.address;
      top[FPSTR(_update_interval_ms)] = config.update_interval_ms;
      top[FPSTR(_filter_tau)]         = config.filter_tau;
      top[FPSTR(_pitch_deg)]          = config.pitch_deg;
      top[FPSTR(_roll_deg)]           = config.roll_deg;
    }

    bool readFromConfig(JsonObject& root) override {
      auto old_cfg = config;
      JsonObject top = root[FPSTR(_name)];

      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top[FPSTR(_enabled)],            config.enabled,            false);
      configComplete &= getJsonValue(top[FPSTR(_address)],            config.address,            (uint8_t)0x6B);
      configComplete &= getJsonValue(top[FPSTR(_update_interval_ms)], config.update_interval_ms, (long)20);
      configComplete &= getJsonValue(top[FPSTR(_filter_tau)],         config.filter_tau,         1.5f);
      configComplete &= getJsonValue(top[FPSTR(_pitch_deg)],          config.pitch_deg,          0.f);
      configComplete &= getJsonValue(top[FPSTR(_roll_deg)],           config.roll_deg,           0.f);

      DEBUG_PRINT(F("QMI8658: "));
      if (top.isNull()) {
        DEBUG_PRINTLN(F("No config found. (Using defaults.)"));
      } else if (memcmp(&config, &old_cfg, sizeof(config)) == 0) {
        DEBUG_PRINTLN(F("config unchanged."));
      } else {
        DEBUG_PRINTLN(F("config updated."));
        configDirty = true;
      }

      return configComplete;
    }

    uint16_t getId() override {
      return USERMOD_ID_IMU;
    }
};


const char QMI8658Mod::_name[]               PROGMEM = "IMU_QMI8658";
const char QMI8658Mod::_enabled[]            PROGMEM = "enabled";
const char QMI8658Mod::_address[]            PROGMEM = "address";
const char QMI8658Mod::_update_interval_ms[] PROGMEM = "update_interval_ms";
const char QMI8658Mod::_filter_tau[]         PROGMEM = "filter_tau";
const char QMI8658Mod::_pitch_deg[]          PROGMEM = "pitch_deg";
const char QMI8658Mod::_roll_deg[]           PROGMEM = "roll_deg";

static QMI8658Mod imu_qmi8658;
REGISTER_USERMOD(imu_qmi8658);

// Compile-time provider registration: strong definition overrides the weak
// default in motion_reactive.cpp.  Linking two IMU drivers simultaneously
// produces a duplicate-symbol error — the correct behavior.
IMUBase* IMU_getProvider() { return &imu_qmi8658; }
