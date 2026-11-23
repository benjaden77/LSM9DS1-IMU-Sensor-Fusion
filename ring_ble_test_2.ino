
#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

// ============================================================
// 1. Hardware and timing configuration
// ============================================================

constexpr uint8_t IMU_AG_ADDRESS = 0x6A;
constexpr uint8_t IMU_MAG_ADDRESS = 0x1C;

constexpr uint8_t SDA_PIN = 11;
constexpr uint8_t SCL_PIN = 12;

constexpr uint32_t SEND_INTERVAL_US = 20000UL;

constexpr float FILTER_CUTOFF_HZ = 5.0f;
constexpr float LINEAR_ACCEL_ALPHA = 0.2f;
constexpr float ACCEL_COUNTS_PER_G = 16457.0f;
constexpr float GYRO_RAD_PER_COUNT =
    0.00875f * PI / 180.0f;

constexpr float MAHONY_KP = 50.0f;
constexpr bool SCAN_I2C_AT_BOOT = true;

// ============================================================
// 2. Sensor calibration parameters
// Recalibrate when the sensor or configuration changes.
// ============================================================

const float ACCEL_BIAS[3] = {
  -133.33f, 72.29f, -291.92f
};

const float ACCEL_MATRIX[3][3] = {
  {1.00260f, 0.00404f, 0.00023f},
  {0.00404f, 1.00708f, 0.00263f},
  {0.00023f, 0.00263f, 0.99905f}
};

const float MAG_BIAS[3] = {
  -922.31f, 2199.41f, 373.17f
};

const float MAG_MATRIX[3][3] = {
  {1.04492f, 0.03452f, -0.01714f},
  {0.03452f, 1.05168f, 0.00644f},
  {-0.01714f, 0.00644f, 1.07005f}
};

// ============================================================
// 3. Global state and BLE services
// ============================================================

LSM9DS1 imu;

BLEDfu bleDfu;
BLEDis bleDeviceInfo;
BLEUart bleUart;

// Quaternion: w, x, y, z
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

float accelFiltered[3] = {0};
float gyroFiltered[3] = {0};
float magFiltered[3] = {0};

float gyroBiasRaw[3] = {0};
float linearFiltered[3] = {0};

bool haveAGFilter = false;
bool haveMag = false;

uint32_t lastSampleUs = 0;
uint32_t lastMagUs = 0;
uint32_t lastSendUs = 0;
uint32_t lastDebugMs = 0;

uint32_t sendAttempts = 0;

// ============================================================
// 4. Vector operations and low-pass filter
// ============================================================

bool normalizeVector(float v[3]) {
  float norm = sqrtf(
    v[0] * v[0] +
    v[1] * v[1] +
    v[2] * v[2]
  );

  if (norm < 1.0e-6f) return false;

  for (uint8_t i = 0; i < 3; ++i) {
    v[i] /= norm;
  }

  return true;
}

void normalizeQuaternion() {
  float norm = sqrtf(
    q[0] * q[0] +
    q[1] * q[1] +
    q[2] * q[2] +
    q[3] * q[3]
  );

  if (norm <= 1.0e-6f) {
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    return;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    q[i] /= norm;
  }
}

float lowPassAlpha(float dt) {
  const float rc =
      1.0f / (2.0f * PI * FILTER_CUTOFF_HZ);

  return dt / (dt + rc);
}

void lowPass3(
    float state[3],
    const float input[3],
    float alpha,
    bool initialize
) {
  for (uint8_t i = 0; i < 3; ++i) {
    state[i] = initialize
        ? input[i]
        : state[i] + alpha * (input[i] - state[i]);
  }
}

void applyCalibration(
    const float raw[3],
    const float bias[3],
    const float matrix[3][3],
    float corrected[3]
) {
  float centered[3];

  for (uint8_t i = 0; i < 3; ++i) {
    centered[i] = raw[i] - bias[i];
  }

  for (uint8_t i = 0; i < 3; ++i) {
    corrected[i] = 0.0f;

    for (uint8_t j = 0; j < 3; ++j) {
      corrected[i] +=
          matrix[i][j] * centered[j];
    }
  }
}

// ============================================================
// 5. I2C debugging and gyro calibration
// ============================================================

void scanI2C() {
  Serial.println("I2C scan started");

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device at 0x");
      Serial.println(address, HEX);
    }
  }
}

bool calibrateGyroBias() {
  Serial.println(
    "Keep the sensor still for gyro calibration"
  );

  delay(2000);

  int32_t sums[3] = {0, 0, 0};
  uint16_t count = 0;

  uint32_t started = millis();

  while (
    count < 500 &&
    (millis() - started) < 7000UL
  ) {
    if (!imu.gyroAvailable()) {
      delay(1);
      continue;
    }

    imu.readGyro();

    sums[0] += imu.gx;
    sums[1] += imu.gy;
    sums[2] += imu.gz;

    ++count;
    delay(4);
  }

  if (count < 500) {
    return false;
  }

  for (uint8_t i = 0; i < 3; ++i) {
    gyroBiasRaw[i] = sums[i] / float(count);
  }

  return true;
}

// ============================================================
// 6. Mahony orientation filter
// ============================================================

// Uses normalized acceleration and magnetic field.
// The horizontal reference is accel cross mag.

void mahonyUpdate(
    const float a[3],
    const float g[3],
    const float m[3],
    float dt
) {
  const float q0 = q[0];
  const float q1 = q[1];
  const float q2 = q[2];
  const float q3 = q[3];

  float hx = a[1] * m[2] - a[2] * m[1];
  float hy = a[2] * m[0] - a[0] * m[2];
  float hz = a[0] * m[1] - a[1] * m[0];

  float h[3] = {hx, hy, hz};

  if (!normalizeVector(h)) {
    return;
  }

  hx = h[0];
  hy = h[1];
  hz = h[2];

  // Estimated Up vector

  const float ux =
      2.0f * (q1 * q3 - q0 * q2);

  const float uy =
      2.0f * (q0 * q1 + q2 * q3);

  const float uz =
      q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Estimated horizontal reference

  const float wx =
      2.0f * (q1 * q2 + q0 * q3);

  const float wy =
      q0*q0 - q1*q1 + q2*q2 - q3*q3;

  const float wz =
      2.0f * (q2 * q3 - q0 * q1);

  // Orientation error

  const float ex =
      (a[1]*uz - a[2]*uy) +
      (hy*wz - hz*wy);

  const float ey =
      (a[2]*ux - a[0]*uz) +
      (hz*wx - hx*wz);

  const float ez =
      (a[0]*uy - a[1]*ux) +
      (hx*wy - hy*wx);

  // Proportional feedback

  const float dx =
      (g[0] + MAHONY_KP * ex) * (0.5f * dt);

  const float dy =
      (g[1] + MAHONY_KP * ey) * (0.5f * dt);

  const float dz =
      (g[2] + MAHONY_KP * ez) * (0.5f * dt);

  // Quaternion integration

  q[0] = q0 - q1*dx - q2*dy - q3*dz;
  q[1] = q1 + q0*dx + q2*dz - q3*dy;
  q[2] = q2 + q0*dy - q1*dz + q3*dx;
  q[3] = q3 + q0*dz + q1*dy - q2*dx;
}

// ============================================================
// 7. Gravity compensation and direction vector
// ============================================================

void getLinearAccel(
    const float accelG[3],
    float out[3]
) {
  const float gravity[3] = {
    2.0f * (q[1]*q[3] - q[0]*q[2]),
    2.0f * (q[0]*q[1] + q[2]*q[3]),
    q[0]*q[0] - q[1]*q[1] -
      q[2]*q[2] + q[3]*q[3]
  };

  for (uint8_t i = 0; i < 3; ++i) {
    float sample =
        (accelG[i] - gravity[i]) * 9.81f;

    linearFiltered[i] +=
        LINEAR_ACCEL_ALPHA *
        (sample - linearFiltered[i]);

    out[i] = linearFiltered[i];
  }
}

// Sensor Y axis expressed in the reference frame.

void getDirectionVector(float out[3]) {
  out[0] =
      2.0f * (q[1]*q[2] - q[0]*q[3]);

  out[1] =
      1.0f - 2.0f *
      (q[1]*q[1] + q[3]*q[3]);

  out[2] =
      2.0f * (q[2]*q[3] + q[0]*q[1]);
}

// ============================================================
// 8. BLE connection and advertising
// ============================================================

void onConnect(uint16_t handle) {
  (void)handle;

  lastSampleUs = 0;
  lastSendUs = 0;

  Serial.println("BLE connected");
}

void onDisconnect(
    uint16_t handle,
    uint8_t reason
) {
  (void)handle;

  lastSampleUs = 0;

  Serial.print("BLE disconnected, reason: 0x");
  Serial.println(reason, HEX);
}

void startAdvertising() {
  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
  );

  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleUart);

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ============================================================
// 9. Initialization
// ============================================================

void setup() {
  Serial.begin(115200);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  if (SCAN_I2C_AT_BOOT) {
    scanI2C();
  }

  imu.settings.gyro.scale = 245;
  imu.settings.accel.scale = 2;
  imu.settings.mag.scale = 4;

  if (!imu.begin(
      IMU_AG_ADDRESS,
      IMU_MAG_ADDRESS,
      Wire
  )) {
    Serial.println(
      "LSM9DS1 initialization failed"
    );

    while (true) {
      delay(1000);
    }
  }

  if (!calibrateGyroBias()) {
    Serial.println(
      "Gyro calibration timed out; using zero bias"
    );
  }

  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setName("rrrr1");
  Bluefruit.setTxPower(4);

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  bleDfu.begin();

  bleDeviceInfo.setManufacturer(
    "Adafruit Industries"
  );
  bleDeviceInfo.setModel("Bluefruit Feather52");
  bleDeviceInfo.begin();

  bleUart.begin();

  startAdvertising();
}

// ============================================================
// 10. Sensor processing and BLE output
// ============================================================

void loop() {
  if (!Bluefruit.connected()) {
    lastSampleUs = 0;
    delay(5);
    return;
  }

  // Read magnetometer independently.

  if (imu.magAvailable()) {
    imu.readMag();

    const float newMag[3] = {
      float(imu.mx),
      float(imu.my),
      float(imu.mz)
    };

    uint32_t t = micros();

    const float dtMag = lastMagUs
        ? (t - lastMagUs) * 1.0e-6f
        : 0.02f;

    lowPass3(
      magFiltered,
      newMag,
      lowPassAlpha(dtMag),
      !haveMag
    );

    haveMag = true;
    lastMagUs = t;
  }

  if (
    !haveMag ||
    !imu.accelAvailable() ||
    !imu.gyroAvailable()
  ) {
    delay(1);
    return;
  }

  imu.readAccel();
  imu.readGyro();

  uint32_t nowUs = micros();

  if (lastSampleUs == 0) {
    lastSampleUs = nowUs;
    return;
  }

  const float dt =
      (nowUs - lastSampleUs) * 1.0e-6f;

  lastSampleUs = nowUs;

  if (dt <= 0.0f || dt > 0.2f) {
    return;
  }

  const float newAccel[3] = {
    float(imu.ax),
    float(imu.ay),
    float(imu.az)
  };

  const float newGyro[3] = {
    float(imu.gx),
    float(imu.gy),
    float(imu.gz)
  };

  const float weight = lowPassAlpha(dt);

  lowPass3(
    accelFiltered,
    newAccel,
    weight,
    !haveAGFilter
  );

  lowPass3(
    gyroFiltered,
    newGyro,
    weight,
    !haveAGFilter
  );

  haveAGFilter = true;

  // Apply calibration matrices.

  float accelCounts[3];
  float magCorrected[3];

  applyCalibration(
    accelFiltered,
    ACCEL_BIAS,
    ACCEL_MATRIX,
    accelCounts
  );

  applyCalibration(
    magFiltered,
    MAG_BIAS,
    MAG_MATRIX,
    magCorrected
  );

  float accelG[3];
  float accelUnit[3];
  float gyroRad[3];

  for (uint8_t i = 0; i < 3; ++i) {
    accelG[i] =
        accelCounts[i] / ACCEL_COUNTS_PER_G;

    accelUnit[i] = accelG[i];

    gyroRad[i] =
        (gyroFiltered[i] - gyroBiasRaw[i]) *
        GYRO_RAD_PER_COUNT;
  }

  // Preserve the original X-axis sign convention.
  // Validate all sensor frames on the real device.

  accelG[0] = -accelG[0];
  accelUnit[0] = -accelUnit[0];
  gyroRad[0] = -gyroRad[0];

  if (
    !normalizeVector(accelUnit) ||
    !normalizeVector(magCorrected)
  ) {
    return;
  }

  // Update orientation.

  mahonyUpdate(
    accelUnit,
    gyroRad,
    magCorrected,
    dt
  );

  normalizeQuaternion();

  // Compute output values.

  float linearAccel[3];
  float direction[3];

  getLinearAccel(accelG, linearAccel);
  getDirectionVector(direction);

  // Send a 13-field CSV record.

  if (nowUs - lastSendUs >= SEND_INTERVAL_US) {
    lastSendUs = nowUs;

    char payload[192];

    int size = snprintf(
      payload,
      sizeof(payload),
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.4f,%.4f,%.4f,%.4f",
      linearAccel[0],
      linearAccel[1],
      linearAccel[2],
      gyroRad[0],
      gyroRad[1],
      gyroRad[2],
      direction[0],
      direction[1],
      direction[2],
      q[0],
      q[1],
      q[2],
      q[3]
    );

    if (
      size > 0 &&
      size < int(sizeof(payload))
    ) {
      bleUart.println(payload);
      ++sendAttempts;
    }
  }

  // Debugging output once per second.

  if (millis() - lastDebugMs >= 1000UL) {
    lastDebugMs = millis();

    Serial.print("BLE send attempts/s: ");
    Serial.println(
      static_cast<unsigned long>(sendAttempts)
    );

    sendAttempts = 0;
  }
}
