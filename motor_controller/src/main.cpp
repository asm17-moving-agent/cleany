#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {

constexpr char kTag[] = "motor_controller";
constexpr char kWifiSsid[] = "Cleany";
constexpr char kWifiPassword[] = "ASM_2026";

constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kPwmTimer = LEDC_TIMER_0;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr ledc_timer_bit_t kPwmResolution = LEDC_TIMER_8_BIT;
constexpr int64_t kCommandTimeoutUs = 750000;

struct Motor {
  gpio_num_t pwmPin;
  gpio_num_t directionPin;
  ledc_channel_t channel;
  int polarity;
  int speed = 0;
  int64_t lastCommandUs = 0;
};

struct Encoder {
  gpio_num_t pinA;
  gpio_num_t pinB;
  volatile int32_t count = 0;
  volatile uint8_t previousState = 0;
};

std::array<Motor, 4> motors = {{
    {GPIO_NUM_2, GPIO_NUM_1, LEDC_CHANNEL_0, -1},    // M1: front-left
    {GPIO_NUM_12, GPIO_NUM_11, LEDC_CHANNEL_1, 1},   // M2: front-right
    {GPIO_NUM_5, GPIO_NUM_6, LEDC_CHANNEL_2, 1},     // M3: rear-right
    {GPIO_NUM_16, GPIO_NUM_17, LEDC_CHANNEL_3, -1},  // M4: rear-left
}};

std::array<Encoder, 4> encoders = {{
    {GPIO_NUM_10, GPIO_NUM_9},
    {GPIO_NUM_14, GPIO_NUM_13},
    {GPIO_NUM_7, GPIO_NUM_15},
    {GPIO_NUM_18, GPIO_NUM_8},
}};

SemaphoreHandle_t motorMutex;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

constexpr int8_t kEncoderTable[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

constexpr char kIndexHtml[] = R"HTML(
<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cleany motor control</title>
<style>
  :root { color-scheme: dark; font-family: system-ui, sans-serif; }
  body { max-width: 1100px; margin: auto; padding: 20px; background: #111827; color: #f9fafb; }
  h1 { margin-bottom: 4px; }
  .status { color: #9ca3af; margin: 0 0 18px; }
  .layout { display: grid; grid-template-columns: minmax(280px, 360px) 1fr; gap: 16px; align-items: start; }
  .controls, .graph { background: #1f2937; border: 1px solid #374151; border-radius: 12px; padding: 16px; }
  .controls h2, .graph h2 { margin-top: 0; }
  .front { text-align: center; color: #60a5fa; font-weight: 700; margin: 4px; }
  .drive { display: grid; grid-template-columns: repeat(3, 64px); justify-content: center; gap: 8px; margin: 14px 0; }
  .drive button { height: 54px; padding: 4px; background: #2563eb; color: white; font-size: 1rem; touch-action: none; user-select: none; }
  .drive .stop { background: #dc2626; }
  .drive-speed { margin-bottom: 18px; }
  .grid { display: grid; gap: 10px; }
  .motor { background: #1f2937; border: 1px solid #374151; border-radius: 12px; padding: 16px; }
  .motor h2 { margin: 0 0 12px; font-size: 1.1rem; }
  .value { float: right; color: #93c5fd; }
  input { width: 100%; margin: 14px 0; }
  button { border: 0; border-radius: 8px; padding: 10px 16px; font-weight: 700; cursor: pointer; }
  .motor button { width: 100%; background: #4b5563; color: white; }
  #stop-all { width: 100%; margin-top: 16px; background: #dc2626; color: white; font-size: 1.1rem; }
  .axis { display: flex; justify-content: space-between; color: #9ca3af; font-size: .8rem; }
  canvas { display: block; width: 100%; height: 420px; background: #111827; border-radius: 8px; }
  .legend { display: flex; flex-wrap: wrap; gap: 14px; margin-top: 12px; }
  .legend span::before { content: ''; display: inline-block; width: 10px; height: 10px; margin-right: 5px; border-radius: 50%; background: var(--color); }
  @media (max-width: 720px) { .layout { grid-template-columns: 1fr; } canvas { height: 300px; } }
</style>
<h1>Cleany motors</h1>
<p class="status" id="status">Connected — controls use calibrated wheel directions</p>
<div class="layout">
<section class="controls">
  <h2>Mecanum drive</h2>
  <div class="front">▲ FRONT</div>
  <div class="drive">
    <button data-x="0" data-y="0" data-w="100">CCW</button>
    <button data-x="100" data-y="0" data-w="0">▲ F</button>
    <button data-x="0" data-y="0" data-w="-100">CW</button>
    <button data-x="0" data-y="100" data-w="0">◀ L</button>
    <button class="stop">STOP</button>
    <button data-x="0" data-y="-100" data-w="0">R ▶</button>
    <span></span>
    <button data-x="-100" data-y="0" data-w="0">▼ B</button>
  </div>
  <div class="drive-speed">
    <label>Drive speed <b id="drive-speed-value">40%</b></label>
    <input id="drive-speed" type="range" min="10" max="100" value="40">
  </div>
  <h2>Individual motors</h2>
  <div class="grid">
    <section class="motor" data-id="1"><h2>M1 · Front left <span class="value">STOP</span></h2><input type="range" min="-100" max="100" value="0"><div class="axis"><span>Reverse</span><span>Forward</span></div><button>Stop M1</button></section>
    <section class="motor" data-id="2"><h2>M2 · Front right <span class="value">STOP</span></h2><input type="range" min="-100" max="100" value="0"><div class="axis"><span>Reverse</span><span>Forward</span></div><button>Stop M2</button></section>
    <section class="motor" data-id="3"><h2>M3 · Rear right <span class="value">STOP</span></h2><input type="range" min="-100" max="100" value="0"><div class="axis"><span>Reverse</span><span>Forward</span></div><button>Stop M3</button></section>
    <section class="motor" data-id="4"><h2>M4 · Rear left <span class="value">STOP</span></h2><input type="range" min="-100" max="100" value="0"><div class="axis"><span>Reverse</span><span>Forward</span></div><button>Stop M4</button></section>
  </div>
  <button id="stop-all">STOP ALL</button>
</section>
<section class="graph">
  <h2>Encoder counts</h2>
  <canvas id="encoder-graph"></canvas>
  <div class="legend">
    <span style="--color:#60a5fa">Encoder 1: <b id="enc1">0</b></span>
    <span style="--color:#f59e0b">Encoder 2: <b id="enc2">0</b></span>
    <span style="--color:#34d399">Encoder 3: <b id="enc3">0</b></span>
    <span style="--color:#f472b6">Encoder 4: <b id="enc4">0</b></span>
  </div>
</section>
</div>
<script>
  const status = document.querySelector('#status');
  let activeDrive = null;
  const driveSpeed = document.querySelector('#drive-speed');
  driveSpeed.addEventListener('input', () => document.querySelector('#drive-speed-value').textContent = `${driveSpeed.value}%`);
  const sendDrive = () => {
    if (!activeDrive) return;
    const scale = +driveSpeed.value / 100;
    const [x, y, w] = activeDrive.map(value => Math.round(value * scale));
    fetch(`/api/drive?x=${x}&y=${y}&w=${w}`, {method:'POST'})
      .then(r => { if (!r.ok) throw Error(); status.textContent = 'Connected — mecanum drive active'; })
      .catch(() => status.textContent = 'Connection lost — firmware watchdog will stop motors');
  };
  const stopDrive = () => {
    if (!activeDrive) return;
    activeDrive = null;
    fetch('/api/drive?x=0&y=0&w=0', {method:'POST'}).catch(() => {});
  };
  document.querySelectorAll('.drive button[data-x]').forEach(button => {
    button.addEventListener('pointerdown', event => {
      event.preventDefault();
      activeDrive = [+button.dataset.x, +button.dataset.y, +button.dataset.w];
      controls.forEach(c => { c.slider.value = 0; c.show(); });
      sendDrive();
    });
  });
  document.querySelector('.drive .stop').addEventListener('click', () => {
    activeDrive = [0, 0, 0]; sendDrive(); activeDrive = null;
  });
  addEventListener('pointerup', stopDrive);
  addEventListener('pointercancel', stopDrive);
  addEventListener('blur', stopDrive);
  setInterval(() => { if (activeDrive) sendDrive(); }, 250);

  const controls = [...document.querySelectorAll('.motor')].map(card => {
    const id = card.dataset.id, slider = card.querySelector('input'), value = card.querySelector('.value');
    const show = () => { const n = +slider.value; value.textContent = n ? `${n > 0 ? 'FWD' : 'REV'} ${Math.abs(n)}%` : 'STOP'; };
    const send = () => fetch(`/api/motor?id=${id}&speed=${slider.value}`, {method:'POST'})
      .then(r => { if (!r.ok) throw Error(); status.textContent = 'Connected — controls use calibrated wheel directions'; })
      .catch(() => status.textContent = 'Connection lost — firmware watchdog will stop motors');
    slider.addEventListener('input', () => { stopDrive(); show(); send(); });
    card.querySelector('button').addEventListener('click', () => { slider.value = 0; show(); send(); });
    return {slider, send, show};
  });
  setInterval(() => controls.filter(c => +c.slider.value).forEach(c => c.send()), 250);
  document.querySelector('#stop-all').addEventListener('click', () => {
    activeDrive = null;
    controls.forEach(c => { c.slider.value = 0; c.show(); });
    fetch('/api/stop', {method:'POST'}).catch(() => {});
  });
  addEventListener('pagehide', () => navigator.sendBeacon('/api/stop'));

  const canvas = document.querySelector('#encoder-graph'), ctx = canvas.getContext('2d');
  const colors = ['#60a5fa', '#f59e0b', '#34d399', '#f472b6'];
  const history = [];
  function drawGraph() {
    const ratio = devicePixelRatio || 1, rect = canvas.getBoundingClientRect();
    if (canvas.width !== Math.round(rect.width * ratio) || canvas.height !== Math.round(rect.height * ratio)) {
      canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio);
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    const w = rect.width, h = rect.height, pad = 28;
    ctx.clearRect(0, 0, w, h);
    const values = history.flat();
    let min = values.length ? Math.min(...values) : -1, max = values.length ? Math.max(...values) : 1;
    if (min === max) { min--; max++; }
    ctx.strokeStyle = '#374151'; ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) { const y = pad + (h - 2 * pad) * i / 4; ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(w - pad, y); ctx.stroke(); }
    colors.forEach((color, motor) => {
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
      history.forEach((sample, i) => {
        const x = pad + (w - 2 * pad) * i / Math.max(1, history.length - 1);
        const y = h - pad - (sample[motor] - min) * (h - 2 * pad) / (max - min);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.stroke();
    });
    ctx.fillStyle = '#9ca3af'; ctx.font = '11px system-ui';
    ctx.fillText(String(max), 3, pad + 4); ctx.fillText(String(min), 3, h - pad + 4);
  }
  setInterval(() => fetch('/api/status').then(r => r.json()).then(data => {
    history.push(data.encoders); if (history.length > 120) history.shift();
    data.encoders.forEach((count, i) => document.querySelector(`#enc${i + 1}`).textContent = count);
    drawGraph();
  }).catch(() => status.textContent = 'Connection lost — firmware watchdog will stop motors'), 100);
  addEventListener('resize', drawGraph);
  drawGraph();
</script>
</html>
)HTML";

uint8_t readEncoderState(const Encoder& encoder) {
  return (static_cast<uint8_t>(gpio_get_level(encoder.pinA)) << 1) |
         static_cast<uint8_t>(gpio_get_level(encoder.pinB));
}

void updateEncoder(void* argument) {
  auto* encoder = static_cast<Encoder*>(argument);
  const uint8_t currentState = readEncoderState(*encoder);
  portENTER_CRITICAL_ISR(&encoderMux);
  const uint8_t tableIndex = (encoder->previousState << 2) | currentState;
  encoder->count += kEncoderTable[tableIndex];
  encoder->previousState = currentState;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

std::array<int32_t, 4> readEncoderCounts() {
  std::array<int32_t, 4> counts;
  portENTER_CRITICAL(&encoderMux);
  for (size_t i = 0; i < encoders.size(); ++i) {
    counts[i] = encoders[i].count;
  }
  portEXIT_CRITICAL(&encoderMux);
  return counts;
}

esp_err_t setMotorSpeedLocked(size_t index, int speed) {
  Motor& motor = motors[index];
  const int rawSpeed = speed * motor.polarity;
  const uint32_t duty = static_cast<uint32_t>(std::abs(speed) * 255 / 100);

  esp_err_t result = ledc_set_duty(kPwmMode, motor.channel, 0);
  if (result == ESP_OK) {
    result = ledc_update_duty(kPwmMode, motor.channel);
  }
  if (result == ESP_OK) {
    result = gpio_set_level(motor.directionPin, rawSpeed > 0 ? 1 : 0);
  }
  if (result == ESP_OK) {
    result = ledc_set_duty(kPwmMode, motor.channel, duty);
  }
  if (result == ESP_OK) {
    result = ledc_update_duty(kPwmMode, motor.channel);
  }
  if (result == ESP_OK) {
    motor.speed = speed;
    motor.lastCommandUs = esp_timer_get_time();
  }
  return result;
}

esp_err_t setMotorSpeed(size_t index, int speed) {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  const esp_err_t result = setMotorSpeedLocked(index, speed);
  xSemaphoreGive(motorMutex);
  return result;
}

esp_err_t setDriveSpeed(int forward, int left, int counterclockwise) {
  // Standard X-configuration inverse kinematics in FL, FR, RR, RL order.
  std::array<int, 4> speeds = {
      forward - left - counterclockwise,
      forward + left + counterclockwise,
      forward - left + counterclockwise,
      forward + left - counterclockwise,
  };
  int maximum = 100;
  for (const int speed : speeds) {
    if (std::abs(speed) > maximum) {
      maximum = std::abs(speed);
    }
  }

  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t result = ESP_OK;
  for (size_t i = 0; i < speeds.size(); ++i) {
    speeds[i] = speeds[i] * 100 / maximum;
    const esp_err_t motorResult = setMotorSpeedLocked(i, speeds[i]);
    if (motorResult != ESP_OK) {
      result = motorResult;
    }
  }
  xSemaphoreGive(motorMutex);
  return result;
}

void stopAll() {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }
  for (size_t i = 0; i < motors.size(); ++i) {
    if (setMotorSpeedLocked(i, 0) != ESP_OK) {
      ESP_LOGE(kTag, "Failed to stop motor %u", static_cast<unsigned>(i + 1));
    }
  }
  xSemaphoreGive(motorMutex);
}

esp_err_t configureMotors() {
  uint64_t directionMask = 0;
  for (const Motor& motor : motors) {
    directionMask |= 1ULL << motor.directionPin;
  }
  const gpio_config_t directionConfig = {
      .pin_bit_mask = directionMask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t result = gpio_config(&directionConfig);
  if (result != ESP_OK) {
    return result;
  }

  const ledc_timer_config_t timerConfig = {
      .speed_mode = kPwmMode,
      .duty_resolution = kPwmResolution,
      .timer_num = kPwmTimer,
      .freq_hz = kPwmFrequencyHz,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
  result = ledc_timer_config(&timerConfig);
  if (result != ESP_OK) {
    return result;
  }

  for (const Motor& motor : motors) {
    const ledc_channel_config_t channelConfig = {
        .gpio_num = motor.pwmPin,
        .speed_mode = kPwmMode,
        .channel = motor.channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = kPwmTimer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {},
    };
    result = ledc_channel_config(&channelConfig);
    if (result != ESP_OK) {
      return result;
    }
    gpio_set_level(motor.directionPin, 0);
  }
  return ESP_OK;
}

esp_err_t configureEncoders() {
  uint64_t encoderMask = 0;
  for (const Encoder& encoder : encoders) {
    encoderMask |= (1ULL << encoder.pinA) | (1ULL << encoder.pinB);
  }
  const gpio_config_t encoderConfig = {
      .pin_bit_mask = encoderMask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  esp_err_t result = gpio_config(&encoderConfig);
  if (result != ESP_OK) {
    return result;
  }
  result = gpio_install_isr_service(0);
  if (result != ESP_OK) {
    return result;
  }
  for (Encoder& encoder : encoders) {
    encoder.previousState = readEncoderState(encoder);
    result = gpio_isr_handler_add(encoder.pinA, updateEncoder, &encoder);
    if (result == ESP_OK) {
      result = gpio_isr_handler_add(encoder.pinB, updateEncoder, &encoder);
    }
    if (result != ESP_OK) {
      return result;
    }
  }
  return ESP_OK;
}

void motorWatchdog(void*) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      continue;
    }
    const int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < motors.size(); ++i) {
      if (motors[i].speed != 0 &&
          now - motors[i].lastCommandUs > kCommandTimeoutUs) {
        ESP_LOGW(kTag, "Motor %u command timed out", static_cast<unsigned>(i + 1));
        setMotorSpeedLocked(i, 0);
      }
    }
    xSemaphoreGive(motorMutex);
  }
}

bool parseInt(const char* text, int* value) {
  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

esp_err_t indexHandler(httpd_req_t* request) {
  httpd_resp_set_type(request, "text/html");
  return httpd_resp_send(request, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t motorHandler(httpd_req_t* request) {
  char query[64];
  char idText[8];
  char speedText[8];
  int id = 0;
  int speed = 0;
  if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "id", idText, sizeof(idText)) != ESP_OK ||
      httpd_query_key_value(query, "speed", speedText, sizeof(speedText)) != ESP_OK ||
      !parseInt(idText, &id) || !parseInt(speedText, &speed) ||
      id < 1 || id > static_cast<int>(motors.size()) ||
      speed < -100 || speed > 100) {
    return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                               "Expected id=1..4 and speed=-100..100");
  }
  if (setMotorSpeed(static_cast<size_t>(id - 1), speed) != ESP_OK) {
    return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Motor update failed");
  }
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(request, "{\"ok\":true}");
}

esp_err_t driveHandler(httpd_req_t* request) {
  char query[64];
  char xText[8];
  char yText[8];
  char wText[8];
  int forward = 0;
  int left = 0;
  int counterclockwise = 0;
  if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "x", xText, sizeof(xText)) != ESP_OK ||
      httpd_query_key_value(query, "y", yText, sizeof(yText)) != ESP_OK ||
      httpd_query_key_value(query, "w", wText, sizeof(wText)) != ESP_OK ||
      !parseInt(xText, &forward) || !parseInt(yText, &left) ||
      !parseInt(wText, &counterclockwise) ||
      forward < -100 || forward > 100 || left < -100 || left > 100 ||
      counterclockwise < -100 || counterclockwise > 100) {
    return httpd_resp_send_err(
        request, HTTPD_400_BAD_REQUEST,
        "Expected x, y and w values from -100 to 100");
  }
  if (setDriveSpeed(forward, left, counterclockwise) != ESP_OK) {
    return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Drive update failed");
  }
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(request, "{\"ok\":true}");
}

esp_err_t stopHandler(httpd_req_t* request) {
  stopAll();
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(request, "{\"ok\":true}");
}

esp_err_t statusHandler(httpd_req_t* request) {
  const auto counts = readEncoderCounts();
  char response[128];
  std::snprintf(response, sizeof(response),
                "{\"encoders\":[%ld,%ld,%ld,%ld]}",
                static_cast<long>(counts[0]), static_cast<long>(counts[1]),
                static_cast<long>(counts[2]), static_cast<long>(counts[3]));
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(request, response);
}

void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t server = nullptr;
  ESP_ERROR_CHECK(httpd_start(&server, &config));

  const httpd_uri_t index = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = indexHandler,
      .user_ctx = nullptr,
  };
  const httpd_uri_t motor = {
      .uri = "/api/motor",
      .method = HTTP_POST,
      .handler = motorHandler,
      .user_ctx = nullptr,
  };
  const httpd_uri_t stop = {
      .uri = "/api/stop",
      .method = HTTP_POST,
      .handler = stopHandler,
      .user_ctx = nullptr,
  };
  const httpd_uri_t drive = {
      .uri = "/api/drive",
      .method = HTTP_POST,
      .handler = driveHandler,
      .user_ctx = nullptr,
  };
  const httpd_uri_t status = {
      .uri = "/api/status",
      .method = HTTP_GET,
      .handler = statusHandler,
      .user_ctx = nullptr,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &motor));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &drive));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &stop));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status));
}

void wifiEventHandler(void*, esp_event_base_t eventBase, int32_t eventId,
                      void*) {
  if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_AP_STADISCONNECTED) {
    stopAll();
    ESP_LOGW(kTag, "Wi-Fi client disconnected; motors stopped");
  }
}

void startWifiAccessPoint() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t* accessPoint = esp_netif_create_default_wifi_ap();

  wifi_init_config_t initConfig = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&initConfig));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             wifiEventHandler, nullptr));

  wifi_config_t wifiConfig = {};
  std::strncpy(reinterpret_cast<char*>(wifiConfig.ap.ssid), kWifiSsid,
               sizeof(wifiConfig.ap.ssid) - 1);
  std::strncpy(reinterpret_cast<char*>(wifiConfig.ap.password), kWifiPassword,
               sizeof(wifiConfig.ap.password) - 1);
  wifiConfig.ap.ssid_len = std::strlen(kWifiSsid);
  wifiConfig.ap.channel = 1;
  wifiConfig.ap.authmode = WIFI_AUTH_WPA2_PSK;
  wifiConfig.ap.max_connection = 4;
  wifiConfig.ap.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifiConfig));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_netif_ip_info_t ipInfo;
  ESP_ERROR_CHECK(esp_netif_get_ip_info(accessPoint, &ipInfo));
  ESP_LOGI(kTag, "Wi-Fi: %s, web interface: http://" IPSTR,
           kWifiSsid, IP2STR(&ipInfo.ip));
}

}  // namespace

extern "C" void app_main() {
  esp_err_t nvsResult = nvs_flash_init();
  if (nvsResult == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvsResult == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvsResult = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvsResult);

  motorMutex = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(motorMutex == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  ESP_ERROR_CHECK(configureMotors());
  ESP_ERROR_CHECK(configureEncoders());
  stopAll();
  xTaskCreate(motorWatchdog, "motor_watchdog", 3072, nullptr, 5, nullptr);

  startWifiAccessPoint();
  startWebServer();
}
