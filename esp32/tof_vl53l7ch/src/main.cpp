#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <stdarg.h>
#include <mbedtls/base64.h>
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_cnh.h"

// All I2C, sensor state, and output queue access belongs to the Arduino loop.
static VL53LMZ_Configuration dev;
static VL53LMZ_ResultsData results;
static constexpr size_t CHUNK = 120;
static char tx[49152], record[16384], command[128];
static VL53LMZ_Motion_Configuration cnhConfig;
static cnh_data_buffer_t cnhBuffer;
static unsigned char cnhBase64[8220];
static bool cnhEnabled = false;
static uint32_t cnhBytes = 0, cnhErrors = 0, recoveryAttempts = 0, healthyFrames = 0;
static int16_t cnhMinMm = 0, cnhMaxMm = 0;
static size_t txHead = 0, txTail = 0, txUsed = 0, used = 0, commandLen = 0;
static bool overflow = false, commandOverflow = false, running = false, initialized = false;
static uint8_t zones = 64, frequency = 10, lastStream = 0;
static bool haveStream = false;
static uint32_t busHz = 400000, lastFrameMs = 0, nextPoll = 0, retryAt = 0;
static uint32_t frames = 0, i2cErrors = 0, readErrors = 0, missed = 0, dropped = 0, counterDiscontinuities = 0;
static uint32_t eventDrops = 0, recoveries = 0, retries = 0, initMs = 0, maxReadUs = 0;
static uint16_t lastReg = 0;
static uint8_t lastIoStatus = 0;
static const char *state = "boot";
static bool wantRunning = true;

static void append(const char *fmt, ...) {
  if (overflow) return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(record + used, sizeof(record) - used, fmt, ap);
  va_end(ap);
  if (n < 0 || size_t(n) >= sizeof(record) - used) { overflow = true; return; }
  used += size_t(n);
}

static void beginRecord() { used = 0; overflow = false; }

static bool enqueue(bool frame = false) {
  append("\n");
  if (overflow || used + (frame ? 2048 : 0) > sizeof(tx) - txUsed) {
    if (frame) ++dropped; else ++eventDrops;
    return false;
  }
  for (size_t i = 0; i < used; ++i) { tx[txHead] = record[i]; txHead = (txHead + 1) % sizeof(tx); }
  txUsed += used;
  return true;
}

static void flushOutput() {
  if (!txUsed || !Serial) return;
  int available = Serial.availableForWrite();
  if (available <= 0) return;
  size_t n = min(txUsed, min(sizeof(tx) - txTail, size_t(min(available, 2048))));
  n = Serial.write(reinterpret_cast<const uint8_t *>(tx + txTail), n);
  txTail = (txTail + n) % sizeof(tx);
  txUsed -= n;
}

static void event(const char *name, uint8_t status = 0) {
  beginRecord();
  append("{\"type\":\"event\",\"event\":\"%s\",\"status\":%u,\"state\":\"%s\",\"uptime_ms\":%lu,\"last_i2c_status\":%u,\"last_register\":%u}",
         name, status, state, (unsigned long)millis(), lastIoStatus, lastReg);
  enqueue();
}

static uint8_t ioFailure(uint16_t reg, uint8_t status) {
  ++i2cErrors; lastReg = reg; lastIoStatus = status;
  return VL53LMZ_STATUS_ERROR;
}

static uint8_t writeSensor(void *, uint16_t reg, uint8_t *data, uint32_t size) {
  for (uint32_t pos = 0; pos < size;) {
    const size_t n = min(size_t(size - pos), CHUNK);
    const uint16_t address = reg + pos;
    uint8_t status = 0;
    for (int attempt = 0; attempt < 2; ++attempt) {
      Wire.beginTransmission(dev.platform.address >> 1);
      size_t a = Wire.write(uint8_t(address >> 8));
      size_t b = Wire.write(uint8_t(address));
      size_t c = Wire.write(data + pos, n);
      status = Wire.endTransmission(true);
      if (a != 1 || b != 1 || c != n) status = 0xFE;
      if (!status) break;
      ioFailure(address, status);
      delay(2);
    }
    if (status) return VL53LMZ_STATUS_ERROR;
    pos += n;
    yield();
  }
  return 0;
}

static uint8_t readSensor(void *, uint16_t reg, uint8_t *data, uint32_t size) {
  // ULD byte/poll callers can inspect the destination even on I/O failure.
  memset(data, 0, size);
  for (uint32_t pos = 0; pos < size;) {
    const size_t n = min(size_t(size - pos), CHUNK);
    const uint16_t address = reg + pos;
    uint8_t status = 0;
    for (int attempt = 0; attempt < 2; ++attempt) {
      Wire.beginTransmission(dev.platform.address >> 1);
      Wire.write(uint8_t(address >> 8)); Wire.write(uint8_t(address));
      status = Wire.endTransmission(false);
      if (!status) {
        size_t got = Wire.requestFrom(uint8_t(dev.platform.address >> 1), n, true);
        if (got != n) status = 0xFD;
        size_t count = 0;
        while (Wire.available()) {
          int value = Wire.read();
          if (count < n) data[pos + count] = uint8_t(value);
          ++count;
        }
        if (count != n) status = 0xFD;
      }
      if (!status) break;
      ioFailure(address, status); delay(2);
    }
    if (status) return VL53LMZ_STATUS_ERROR;
    pos += n;
  }
  return 0;
}

static uint8_t waitSensor(void *, uint32_t ms) { delay(ms); return 0; }

static bool check(uint8_t status, const char *name) {
  if (!status) return true;
  event(name, status); return false;
}

static bool beginBus() {
  Wire.end();
  // An ESP reset may interrupt a still-powered sensor mid-transaction.
  // Release SDA with up to nine open-drain SCL pulses, then generate STOP.
  pinMode(8, OUTPUT_OPEN_DRAIN); digitalWrite(8, HIGH);
  pinMode(9, OUTPUT_OPEN_DRAIN); digitalWrite(9, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && !digitalRead(8); ++i) {
    digitalWrite(9, LOW); delayMicroseconds(10);
    digitalWrite(9, HIGH); delayMicroseconds(10);
  }
  digitalWrite(9, LOW); digitalWrite(8, LOW); delayMicroseconds(10);
  digitalWrite(9, HIGH); delayMicroseconds(10);
  digitalWrite(8, HIGH); delayMicroseconds(10);
  bool released = digitalRead(8) && digitalRead(9);
  pinMode(8, INPUT); pinMode(9, INPUT);
  if (!released) return false;
  if (!Wire.begin(8, 9, busHz)) return false;
  Wire.setTimeOut(30);
  delay(10);
  return true;
}

static void info() {
  beginRecord();
  append("{\"type\":\"info\",\"firmware\":\"tof-esp32-1.0\",\"uld\":\"%s\",\"upstream\":\"600e275895ddd79c03282c45f8dcd02e763fecb4\",\"device_id\":%u,\"revision_id\":%u,\"module_type\":%u,\"sda\":8,\"scl\":9,\"address_7bit\":41,\"zones\":%u,\"hz\":%u,\"bus_hz\":%lu,\"mode\":\"continuous\",\"state\":\"%s\",\"init_ms\":%lu,\"sensor_bytes_per_frame\":%lu,\"target_per_zone\":1,\"orientation\":\"native_zone_order\",\"flash_bytes\":%lu,\"cnh_enabled\":%s}",
         VL53LMZ_API_REVISION, dev.device_id, dev.revision_id, dev.module_type,
         zones, frequency, (unsigned long)busHz, state, (unsigned long)initMs,
         (unsigned long)dev.data_read_size, (unsigned long)ESP.getFlashChipSize(), cnhEnabled ? "true" : "false");
  enqueue();
}

static void stats() {
  beginRecord();
  append("{\"type\":\"stats\",\"uptime_ms\":%lu,\"frames\":%lu,\"i2c_errors\":%lu,\"read_errors\":%lu,\"estimated_missed_frames\":%lu,\"usb_dropped_frames\":%lu,\"event_drops\":%lu,\"recoveries\":%lu,\"recovery_attempts\":%lu,\"queue_bytes\":%u,\"max_read_us\":%lu,\"heap_free\":%lu,\"state\":\"%s\",\"cnh_errors\":%lu,\"sensor_counter_discontinuities\":%lu}",
         (unsigned long)millis(), (unsigned long)frames, (unsigned long)i2cErrors,
         (unsigned long)readErrors, (unsigned long)missed, (unsigned long)dropped,
         (unsigned long)eventDrops, (unsigned long)recoveries, (unsigned long)recoveryAttempts,
         unsigned(txUsed), (unsigned long)maxReadUs, (unsigned long)ESP.getFreeHeap(), state, (unsigned long)cnhErrors, (unsigned long)counterDiscontinuities);
  enqueue();
}

static bool startConfigured() {
  if (!check(vl53lmz_set_resolution(&dev, zones), "set_resolution_failed")) return false;
  if (!check(vl53lmz_set_ranging_frequency_hz(&dev, frequency), "set_frequency_failed")) return false;
  if (!check(vl53lmz_set_ranging_mode(&dev, VL53LMZ_RANGING_MODE_CONTINUOUS), "set_mode_failed")) return false;
  if (!check(vl53lmz_create_output_config(&dev), "output_config_failed")) return false;
  if (cnhEnabled) {
    memset(&cnhConfig, 0, sizeof(cnhConfig));
    if (!check(vl53lmz_cnh_init_config(&cnhConfig, 0, 48, 1), "cnh_config_failed") ||
        !check(vl53lmz_cnh_create_agg_map(&cnhConfig, 64, 0, 0, 2, 2, 4, 4), "cnh_map_failed") ||
        !check(vl53lmz_cnh_calc_required_memory(&cnhConfig, &cnhBytes), "cnh_memory_failed") ||
        !check(vl53lmz_cnh_calc_min_max_distance(&cnhConfig, &cnhMinMm, &cnhMaxMm), "cnh_range_failed") ||
        !check(vl53lmz_cnh_send_config(&dev, &cnhConfig), "cnh_send_failed")) return false;
    if (cnhBytes > sizeof(cnhBuffer) || cnhBytes > 4095) { event("cnh_size_invalid"); return false; }
    if (!check(vl53lmz_add_output_block(&dev, (VL53LMZ_CNH_DATA_IDX << 16) | (cnhBytes << 4)), "cnh_output_failed")) return false;
  }
  if (!check(vl53lmz_send_output_config_and_start(&dev), "start_failed")) return false;
  running = true; state = "running"; haveStream = false; healthyFrames = 0; lastFrameMs = millis();
  event("started"); info(); return true;
}

static bool initialize() {
  state = "initializing"; event("initializing"); flushOutput();
  initialized = false; running = false;
  if (!beginBus()) { event("bus_begin_failed"); return false; }
  memset(&dev, 0, sizeof(dev));
  dev.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;
  dev.platform.Read = readSensor; dev.platform.Write = writeSensor; dev.platform.Wait = waitSensor;
  uint8_t alive = 0;
  if (!check(vl53lmz_is_alive(&dev, &alive), "identity_read_failed") || !alive || dev.module_type != VL53LMZ_MODULE_TYPE_L7) {
    event("identity_not_l7"); info(); return false;
  }
  event("identity_l7");
  uint32_t start = millis();
  if (!check(vl53lmz_init(&dev), "sensor_init_failed")) return false;
  initMs = millis() - start;
  initialized = true;
  event("sensor_initialized");
  return startConfigured();
}

static void requestRecovery(const char *reason) {
  running = false; initialized = false; state = "recovering"; retryAt = millis() + 2000;
  event(reason);
}

template<class T> static void array(const char *name, const T *values) {
  append(",\"%s\":[", name);
  for (unsigned i = 0; i < zones; ++i) append(i ? ",%ld" : "%ld", (long)values[i]);
  append("]");
}

static void emitFrame(uint32_t readUs) {
  beginRecord();
  append("{\"type\":\"frame\",\"seq\":%lu,\"sensor_stream\":%u,\"time_us\":%llu,\"zones\":%u,\"hz\":%u,\"read_us\":%lu,\"temperature_c\":%d",
         (unsigned long)frames, dev.streamcount, (unsigned long long)esp_timer_get_time(), zones,
         frequency, (unsigned long)readUs, results.silicon_temp_degc);
  array("distance_mm", results.distance_mm);
  array("target_status", results.target_status);
  array("target_count", results.nb_target_detected);
  append(",\"valid\":[");
  for (unsigned i = 0; i < zones; ++i) {
    bool valid = results.nb_target_detected[i] > 0 && (results.target_status[i] == 5 || results.target_status[i] == 9);
    append(i ? ",%s" : "%s", valid ? "true" : "false");
  }
  append("]");
  array("signal_kcps_per_spad", results.signal_per_spad);
  array("ambient_kcps_per_spad", results.ambient_per_spad);
  array("sigma_mm", results.range_sigma_mm);
  array("reflectance_percent", results.reflectance);
  if (cnhEnabled) {
    uint8_t status = vl53lmz_results_extract_block(&dev, VL53LMZ_CNH_DATA_IDX, reinterpret_cast<uint8_t *>(cnhBuffer), cnhBytes);
    if (status) { ++cnhErrors; append(",\"cnh_error\":%u", status); }
    else {
      size_t encoded = 0;
      int err = mbedtls_base64_encode(cnhBase64, sizeof(cnhBase64), &encoded, reinterpret_cast<uint8_t *>(cnhBuffer), cnhBytes);
      if (err) { ++cnhErrors; append(",\"cnh_error\":255"); }
      else {
        cnhBase64[encoded] = 0;
        uint32_t crc = 0xFFFFFFFF;
        const uint8_t *p = reinterpret_cast<uint8_t *>(cnhBuffer);
        for (uint32_t i = 0; i < cnhBytes; ++i) {
          crc ^= p[i];
          for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
        }
        append(",\"cnh\":{\"aggregates\":16,\"bins\":48,\"start_bin\":0,\"sub_sample\":1,\"merge_x\":2,\"merge_y\":2,\"min_target_mm\":%d,\"max_target_mm\":%d,\"bytes\":%lu,\"crc32\":%lu,\"encoding\":\"base64-le-uld-buffer\",\"data\":\"%s\"}",
               cnhMinMm, cnhMaxMm, (unsigned long)cnhBytes, (unsigned long)(crc ^ 0xFFFFFFFFu), cnhBase64);
      }
    }
  }
  append("}"); enqueue(true);
}

static void handleCommand(char *line) {
  if (!strcmp(line, "info")) { info(); return; }
  if (!strcmp(line, "stats")) { stats(); return; }
  if (!strcmp(line, "cnh on") || !strcmp(line, "cnh off")) {
    bool enable = !strcmp(line, "cnh on");
    if (enable && (zones != 64 || frequency > (busHz == 1000000 ? 10 : 5) || busHz == 100000)) {
      event("cnh_requires_64zones_max5hz_400khz_or_max10hz_1mhz"); return;
    }
    if (running && !check(vl53lmz_stop_ranging(&dev), "stop_failed")) initialized = false;
    running = false; cnhEnabled = enable;
    if (wantRunning) {
      if (!(initialized ? startConfigured() : initialize())) requestRecovery("cnh_recovery");
    } else { event("configured"); info(); }
    return;
  }
  if (!strcmp(line, "stop")) {
    wantRunning = false;
    bool ok = !running || check(vl53lmz_stop_ranging(&dev), "stop_failed");
    running = false; state = ok ? "stopped" : "fault";
    if (!ok) initialized = false;
    event(ok ? "stopped" : "stop_error"); return;
  }
  if (!strcmp(line, "start") || !strcmp(line, "reinit")) {
    bool full = !strcmp(line, "reinit") || !initialized;
    wantRunning = true; retries = 0;
    if (running && !full) { event("already_running"); return; }
    if (running && !check(vl53lmz_stop_ranging(&dev), "stop_failed")) full = true;
    running = false;
    if (!(full ? initialize() : startConfigured())) requestRecovery("start_recovery");
    return;
  }
  unsigned z, f, b; char trailing;
  if (sscanf(line, "config %u %u %u %c", &z, &f, &b, &trailing) == 3) {
    if ((z != 16 && z != 64) || f < 1 || f > (z == 64 ? 15u : 60u) ||
        (b != 100000 && b != 400000 && b != 1000000) ||
        (cnhEnabled && (z != 64 || b == 100000 || f > (b == 1000000 ? 10u : 5u)))) { event("invalid_config"); return; }
    bool resume = wantRunning;
    if (running && !check(vl53lmz_stop_ranging(&dev), "stop_failed")) initialized = false;
    running = false; zones = z; frequency = f; busHz = b; haveStream = false;
    if (!beginBus()) { initialized = false; state = "fault"; event("bus_begin_failed"); return; }
    retries = 0;
    if (resume) {
      if (!(initialized ? startConfigured() : initialize())) requestRecovery("config_recovery");
    } else { state = "stopped"; event("configured"); info(); }
    return;
  }
  event("unknown_command");
}

static void commands() {
  for (unsigned count = 0; count < 128 && Serial.available(); ++count) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (commandOverflow) event("command_too_long");
      else if (commandLen) { command[commandLen] = 0; handleCommand(command); }
      commandLen = 0; commandOverflow = false;
    } else if (commandLen + 1 < sizeof(command) && !commandOverflow) command[commandLen++] = c;
    else commandOverflow = true;
  }
}

void setup() {
  Serial.setTxBufferSize(4096); Serial.setTxTimeoutMs(0); Serial.begin(115200);
  delay(1500);
  if (!initialize()) requestRecovery("boot_recovery");
}

void loop() {
  flushOutput(); commands();
  uint32_t now = millis();
  if (wantRunning && !running && !strcmp(state, "recovering") && int32_t(now - retryAt) >= 0) {
    if (retries >= 3) { state = "fault"; event("recovery_exhausted"); }
    else {
      ++retries; ++recoveryAttempts;
      if (initialize()) { ++recoveries; }
      else requestRecovery("retry_failed");
    }
  }
  if (running && int32_t(now - nextPoll) >= 0) {
    nextPoll = now + 5;
    uint8_t ready = 0;
    if (!check(vl53lmz_check_data_ready(&dev, &ready), "ready_failed")) {
      ++readErrors; requestRecovery("ready_recovery");
    } else if (ready) {
      uint64_t start = esp_timer_get_time();
      uint8_t status = vl53lmz_get_ranging_data(&dev, &results);
      uint32_t readUs = esp_timer_get_time() - start;
      maxReadUs = max(maxReadUs, readUs);
      if (!check(status, "read_failed")) { ++readErrors; requestRecovery("read_recovery"); }
      else {
        // Only the low seven bits are cyclic on the observed L7 firmware.
        // 8x8 uses four subframes. Keep raw counters in every output frame.
        unsigned delta = (unsigned(dev.streamcount) - lastStream) & 0x7Fu;
        if (haveStream && delta != (zones == 64 ? 4u : 1u)) ++counterDiscontinuities;
        // Timing is an estimate; it cannot prove exact sensor frame losses.
        unsigned periods = ((millis() - lastFrameMs) * frequency + 500u) / 1000u;
        if (haveStream && periods > 1) missed += periods - 1;
        haveStream = true; lastStream = dev.streamcount;
        lastFrameMs = millis(); ++frames;
        if (++healthyFrames >= 10) retries = 0;
        emitFrame(readUs);
      }
    }
    if (running && uint32_t(millis() - lastFrameMs) > max(3000u, 3000u / frequency)) requestRecovery("frame_timeout");
  }
  flushOutput(); delay(1);
}
