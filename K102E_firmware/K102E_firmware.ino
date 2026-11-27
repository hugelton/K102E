///////// K102 v1.7 by Leo Kuroshita
///// hardware for 0.5  for K102E

// 133Mhz, non profiling, pico-probe
/*
 * K102 Synthesizer - Core Definitions and Headers

 */

// === Basic Definitions ===
#define MODEL_NAME "K102E"
#define VERSION "1.7.0"
#define PCB_VERSION "0.5"

// === Production/Debug Configuration ===
// Set to 1 for production builds, 0 for debug builds
#define PRODUCTION_BUILD 1

// === Include Headers ===
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
#include <Arduino.h>
// #include <EEPROM.h>  // Removed for simplicity
#include <FastLED.h>
#include <I2S.h>
#include <MCP_ADC.h>
#include <SPI.h>
#include <cmath>
#include <stdint.h>

// === Waveform Banks ===
#include "waves/header/chaos.h"
#include "waves/header/formant.h"
#include "waves/header/lfsr.h"
#include "waves/header/metal.h"
#include "waves/header/pulse_width.h"
#include "waves/header/sin_saw.h"
#include "waves/header/sin_sqr.h"
#include "waves/header/sin_tri.h"

// Q9 Tools
#define FP_SCALE 512  // 1.0f ≒ 512（Q9）
#define FLOAT_TO_Q9(x) ((int16_t)((x)*FP_SCALE))
#define Q9_TO_FLOAT(x) ((float)(x) / FP_SCALE)
#define Q9_MUL(a, b) ((int16_t)constrain((((int32_t)(a) * (b)) >> 9), -32767, 32767))
#define Q9_CLIP(x) (constrain((x), -512, 511))


// サイクルカウントに基づく安全なタイミング関数
inline uint32_t safe_millis() {
  // return rp2040.getCycleCount() / (rp2040.f_cpu() / 1000);
  return millis();
}

inline uint32_t safe_micros() {
  // return rp2040.getCycleCount() / (rp2040.f_cpu() / 1000000);
  return micros();
}


#if !PRODUCTION_BUILD
// デバッガ用のスクラッチメモリ領域（RAM内の特定領域を確保）
// この領域はリセット後も値が保持される
// デバッガ用のスクラッチメモリ領域の拡張
struct DebugMemory {
  uint32_t marker;           // 0xDEADBEEF などの特定値
  uint32_t resetCounter;     // リセット回数
  uint32_t spiErrorCount;    // SPIエラーカウント
  uint32_t adcErrorCount;    // ADCエラーカウント
  uint32_t core1ErrorCount;  // Core1エラーカウント
  uint32_t resetReason;      // 最後のリセット理由

  // 以下の新しいデバッグフィールドを追加
  uint8_t currentZone;         // 現在処理中のゾーン
  uint8_t lastErrorZone;       // エラーが発生した最後のゾーン
  uint32_t zoneStartTime;      // 現在のゾーン処理開始時間
  uint32_t zoneDurations[10];  // 各ゾーンの処理時間履歴
  uint32_t lastWdtTime;        // 最後にWDTが更新された時間
  uint32_t core1LoopCount;     // Core1のループカウント
  uint32_t longestZoneTime;    // 最も長く時間がかかったゾーンの処理時間
  uint8_t longestZoneId;       // 最も長く時間がかかったゾーンのID


  // 追加するフィールド
  uint32_t core0_activity_counter;  // Core0活動カウンター
  uint32_t core1_activity_counter;  // Core1活動カウンター
  uint32_t last_active_core0;       // Core0最終活動時間
  uint32_t last_active_core1;       // Core1最終活動時間
  uint32_t loopCounter;             // ループカウンター
};

// この領域は.noinit セクションに配置して通常の初期化から除外
// 0x20030000は一般的に未使用のRAM領域
volatile DebugMemory *debugMem __attribute__((section(".noinit")));
#endif

// === Constants ===
// Buffer and sampling settings
#define BUFFER_SIZE 256  // Temporarily increase for debugging
#define LUT_SIZE 1024
#define NUM_BUFFERS 2
const int32_t sampleRate = 48000;
float global_voltage;
// Pin definitions
#define PIN_I2S_BCLK 6
#define PIN_I2S_LRCLK (PIN_I2S_BCLK + 1)
#define PIN_I2S_DOUT 8
#define CS_PIN 9
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 10
#define METHOD_SELECT_PIN 14
#define METHOD_A_PIN 4
#define METHOD_B_PIN 5
#define INTERNAL_ADC_PIN_1 27
#define INTERNAL_ADC_PIN_2 29
#define INTERNAL_ADC_PIN_3 28
#define LFO_WAVEFORM_SELECT_PIN 27
#define LFO_EFFECT_1 15
#define LFO_EFFECT_2 26
#define LFO_WAVEFORM_1 13
#define LFO_WAVEFORM_2 14
#define SYNC_PIN 3
#define OSCB_COPY_PIN 2
#define DETUNE_PIN 1
#define LFO_WAVE_ALT_PIN 0
//
#define NUM_LEDS 1
#define DATA_PIN 16

// ADC settings
#define NUM_CHANNELS 8
#define MAX_INPUT_LENGTH 32
#define DEFAULT_ROOT_FREQ 130.813f
#define MAX_ADC_VAL 4092

#define CENTER_ADC_OFFSET 2048
//

const float CH1_SCALE = 5.0f / 3.3f;  // ≒1.515
const int ADC_MAX = MAX_ADC_VAL;

#define NEOPIXEL_DARKEST 2

// === WDT管理 ===
#define WDT_TIMEOUT_MS 1000         // WDTタイムアウト
#define WDT_UPDATE_INTERVAL 100     // WDT更新間隔
#define CORE_HEARTBEAT_TIMEOUT 500  // Core間のハートビートタイムアウト

// WDT用タイムスタンプ
volatile unsigned long lastWdtUpdateCore0 = 0;
volatile unsigned long lastWdtUpdateCore1 = 0;
volatile unsigned long lastHeartbeatCore0 = 0;
volatile unsigned long lastHeartbeatCore1 = 0;

// エラー統計
volatile uint32_t errorCountSPI = 0;
volatile uint32_t errorCountADC = 0;
volatile uint32_t errorCountCore1 = 0;
volatile uint32_t consecutiveErrors = 0;

// EEPROM関連
#define WDT_RESET_COUNT_ADDR 0
#define ERROR_STATS_ADDR 4  // エラー統計の保存アドレス
#define MAX_WDT_COUNT 50000

// === Forward Declarations ===
bool isOscillateStarted = false;  // 構造体定義の前に関数プロトタイプを宣言
struct Oscillator;
struct LFO;
struct WaveformBank;


// 関数プロトタイプ宣言
void updatePhaseData(volatile Oscillator &osc);
void updateLfoPhaseData(volatile LFO &lfo);
int16_t interpolateWaveform(const WaveformBank &bank, float waveform, uint32_t phase);
int16_t generateLfoWaveform_fixed(volatile LFO &lfo);
int16_t synthesize(volatile Oscillator &osc1, volatile Oscillator &osc2, volatile uint32_t &phaseA, volatile uint32_t &phaseB);
void generate_wave(int16_t *buffer, size_t size, volatile Oscillator &osc1, volatile Oscillator &osc2);
bool readAdcWithTimeout(int channel, int &result, unsigned long timeout_ms = 50);
void updateOscillatorFrequencies();
// Serial command function prototypes removed
// === Enums and Structs ===
// Synthesis method enum
enum SynthesisMethod { FADE,
                       PHASE,
                       MULT };

// Oscillator structure
struct Oscillator {
  volatile uint32_t phase;
  volatile uint32_t phase_delta;
  volatile uint8_t bank_index;
  volatile float waveform;
  volatile float frequency;
  volatile bool is_sync;
  volatile bool is_detune;
};

// LFO waveform types
enum LfoWaveform { TRIANGLE,
                   SQUARE,
                   RAMP_UP,
                   RAMP_DOWN,
                   SINE,
                   RANDOM };

struct LFO {
  volatile uint32_t phase;
  volatile uint32_t phase_delta;
  volatile float frequency;
  volatile float amplitude;
  volatile LfoWaveform waveform;
  volatile float current_random_value;
  volatile uint32_t last_step_change;
};

// Waveform bank structure
struct WaveformBank {
  const char *name;
  const int16_t (*waves)[1024];
};

// === Global Variables and Instances ===
// Class instances
I2S i2s(OUTPUT);
MCP3208 adc(&SPI1);

// NEOPIXEL
unsigned long previousLEDMillis = 0;
const unsigned long LED_INTERVAL = 100;  // 100ms = 10Hz

// Oscillators and LFO
Oscillator osc1, osc2;
LFO lfo;
volatile float combine_alpha = 0.5f;
volatile SynthesisMethod currentMethod = FADE;

// State variables
volatile bool should_mute = false;
volatile bool pending_reboot = false;
volatile bool pending_bootloader = false;
volatile bool specialLfoActive = false;
volatile uint8_t specialLfoMode = 0;
volatile bool needsEepromUpdate = false;
volatile bool lfoForPitchA = false;
volatile bool lfoForPitchB = false;
volatile bool lfoForAlpha = false;
volatile float detune_cents = 0;

// Audio buffer variables
int16_t buffers[NUM_BUFFERS][BUFFER_SIZE];
volatile uint8_t current_buffer = 0;
volatile uint8_t next_buffer = 1;
volatile bool buffer_ready = false;

volatile uint16_t cvAlpha = 0;
volatile uint16_t cvForm = 0;
volatile uint16_t knobAlpha = 0;
volatile uint16_t knobWaveformA = 0;
volatile uint16_t knobWaveformB = 0;

// LED array
CRGB leds[NUM_LEDS];

// Configuration variables
float ROOT_FREQ = DEFAULT_ROOT_FREQ;
String inputString = "";
boolean stringComplete = false;
int centerOffset0 = 2048;
int centerOffset2 = 2048;
int maxAdcValue = 4095;
int maxAdcValue2 = 4095;

char uniqueId[32];

// ADC and timing variables
volatile int filteredMcpRawValues[NUM_CHANNELS] = { 0 };
volatile int prevMcpSamples[NUM_CHANNELS][5] = { 0 };
volatile int changeMcpSampleCount[NUM_CHANNELS] = { 0 };
volatile bool mcpValueChanged[NUM_CHANNELS] = { false };
volatile int filteredCH7Value = 0;
volatile bool ch7ValueChanged = false;
volatile bool new_data_available = false;
unsigned long previousMillis = 0;

// Internal ADC variables
volatile int filteredInternalAdcValues[3] = { 0 };
volatile int prevInternalAdcSamples[3][5] = { { 0 } };
volatile int changeInternalAdcSampleCounts[3] = { 0 };
volatile bool internalAdcValueChanged[3] = { false };

////
// グローバル変数
volatile uint32_t core1LoopCounter = 0;
volatile uint32_t core1LoopRate = 0;
volatile unsigned long core1LastWdtUpdate = 0;

// Filter constants
const float CH7_FILTER_ALPHA = 0.4f;
float FILTER_ALPHA[8] = { 0.6f, 0.6f, 0.6f, 0.6f, 0.3f, 0.6f, 0.6f, 0.6f };  // CH5 faster response

const int NUM_SAMPLES = 5;
const unsigned long INTERVAL = 2;

// Low latency control
volatile bool forceBufferUpdate = false;

// Ultra low latency pitch CV reading for Core0
void getPitchCvValue() {
  static unsigned long lastPitchCvRead = 0;
  unsigned long currentTime = safe_micros();
  
  // Read pitch CV at every buffer generation (ultra low latency)
  if (currentTime - lastPitchCvRead >= 300) {  // Max 2kHz sampling
    lastPitchCvRead = currentTime;
    
    int rawValue;
    if (readAdcWithTimeout(5, rawValue)) {  // CH5 = Pitch CV
      filteredMcpRawValues[5] = FILTER_ALPHA[5] * rawValue + (1 - FILTER_ALPHA[5]) * filteredMcpRawValues[5];
      
      // Update sample history for CH5
      for (int j = NUM_SAMPLES - 1; j > 0; j--) {
        prevMcpSamples[5][j] = prevMcpSamples[5][j - 1];
      }
      prevMcpSamples[5][0] = filteredMcpRawValues[5];
      
      // Immediately update oscillator frequencies
      updateOscillatorFrequencies();
    }
  }
}

// Available waveform banks
const WaveformBank availableBanksOSC1[] = {
  { "Sin-Saw", sin_saw },    // Sine to sawtooth morphing
  { "Sin-Sqr", sin_sqr },    // Sine to square morphing
  { "Sin-Tri", sin_tri },    // Sine to triangle morphing
  { "Chaos", chaos },        // PPG waveforms with progressive harmonics
  { "Metal", metal },        // PPG-style metallic formants
  { "Formant", formant },    // Step-wise degraded sawtooth
  { "Pulse", pulse_width },  // Variable pulse width waveforms
  { "LFSR", lfsr }           // Square wave morphing into LFSR noise
};

const WaveformBank availableBanksOSC2[] = {
  { "Sin-Saw", sin_saw },    // Sine to sawtooth morphing
  { "Sin-Sqr", sin_sqr },    // Sine to square morphing
  { "Sin-Tri", sin_tri },    // Sine to triangle morphing
  { "Chaos", chaos },        // PPG waveforms with progressive harmonics
  { "Metal", metal },        // PPG-style metallic formants
  { "Formant", formant },    // Step-wise degraded sawtooth
  { "Pulse", pulse_width },  // Variable pulse width waveforms
  { "LFSR", lfsr }           // Square wave morphing into LFSR noise
};

const int NUM_BANKS_OSC1 = sizeof(availableBanksOSC1) / sizeof(WaveformBank);

//

//
/*
 * K102 Synthesizer - Helper Functions
 */

// === Helper Functions ===
// WDT更新関数（両方のコアから呼び出し可能）
void updateWatchdog() {
  uint32_t currentTime = safe_millis();

  if (rp2040.cpuid() == 0) {
    lastWdtUpdateCore0 = currentTime;
    lastHeartbeatCore0 = currentTime;
#if !PRODUCTION_BUILD
    debugMem->last_active_core0 = currentTime;
#endif
  } else {
    lastWdtUpdateCore1 = currentTime;
    lastHeartbeatCore1 = currentTime;
#if !PRODUCTION_BUILD
    debugMem->last_active_core1 = currentTime;
#endif
  }

  // 常に更新（複雑な条件チェックを避ける）
  watchdog_update();
}

void checkCoreHeartbeats() {
  uint32_t now = safe_millis();

  // Core0がCore1を監視
  if (rp2040.cpuid() == 0) {
    if (now - lastHeartbeatCore1 > CORE_HEARTBEAT_TIMEOUT) {
// Core1が応答していない
#if !PRODUCTION_BUILD
      debugMem->lastErrorZone = 101;  // Core1問題
#endif
      errorCountCore1++;
#if !PRODUCTION_BUILD
      debugMem->core1ErrorCount = errorCountCore1;
#endif

      // WDTを更新して通常の監視に任せる
      watchdog_update();
    }
  }
  // Core1がCore0を監視
  else {
    if (now - lastHeartbeatCore0 > CORE_HEARTBEAT_TIMEOUT) {
// Core0が応答していない
#if !PRODUCTION_BUILD
      debugMem->lastErrorZone = 100;  // Core0問題
#endif

      // WDTを更新して通常の監視に任せる
      watchdog_update();
    }
  }
}



// Frequency conversion functions
float centToFreqMultiplier(float cents) {
  return pow(2.0f, cents / 1200.0f);
}

float logMap(float value, float in_min, float in_max, float out_min,
             float out_max) {
  float normalized = (value - in_min) / (in_max - in_min);
  float logged = pow(10, normalized * 5 - 3);
  return logged;
}

// Oscillator phase update
void updatePhaseData(volatile Oscillator &osc) {
  // より精密な位相デルタ計算
  double phase_increment = (double)osc.frequency * (double)(1ULL << 32) / (double)sampleRate;
  osc.phase_delta = (uint32_t)phase_increment;
}

// LFO phase update
void updateLfoPhaseData(volatile LFO &lfo) {
  // より精密な位相デルタ計算
  double phase_increment = (double)lfo.frequency * (double)(1ULL << 32) / (double)sampleRate;
  lfo.phase_delta = (uint32_t)phase_increment;
}

// Waveform generation
int16_t interpolateWaveform(const WaveformBank &bank, float waveform,
                            uint32_t phase) {
  uint32_t index = (phase >> 22) & 0x3FF;

  // Get integer and fractional parts
  uint8_t wave1 = static_cast<uint8_t>(waveform) & 0x0F;
  uint8_t wave2 = (wave1 + 1) & 0x0F;

  // Convert fractional part to fixed point (0-1024 range for better precision)
  uint32_t frac =
    static_cast<uint32_t>((waveform - static_cast<float>(wave1)) * 1024.0f);

  // Get the two samples to interpolate between
  int32_t sample1 = static_cast<int32_t>(bank.waves[wave1][index]);
  int32_t sample2 = static_cast<int32_t>(bank.waves[wave2][index]);

  // Perform interpolation with larger coefficient (1024 instead of 256)
  // Using 32-bit arithmetic to prevent overflow
  int32_t interpolated = sample1 + ((sample2 - sample1) * frac) / 1024;

  return static_cast<int16_t>(interpolated);
}

int16_t generateLfoWaveform_fixed(volatile LFO &lfo) {
  uint32_t index = (lfo.phase >> 22) & 0x3FF;
  static int16_t current_random_value = 0;
  static uint32_t last_index = 0;
  int16_t output = 0;

  switch (lfo.waveform) {
    case TRIANGLE:
      output = sin_tri[15][index];  // すでに-512～512
      break;
    case SQUARE:
      output = (index < 512) ? 512 : -512;
      break;
    case SINE:
      output = sin_sqr[0][index];
      break;
    case RAMP_UP:
      output = (index * 2) - 1023;
      break;
    case RAMP_DOWN:
      output = 1023 - (index * 2);
      break;
    case RANDOM:
      if (index < last_index) {
        current_random_value = random(-512, 513);
      }
      last_index = index;
      output = current_random_value;
      break;
  }

  return Q9_CLIP(output);
}

// Safe shutdown procedure
void safeShutdown() {
  should_mute = true;
  delay(50);
  i2s.end();
}

/*
 * K102 Synthesizer - Core Synthesis Functions
 */

// === Synthesis Core Functions ===
// Main synthesis function
int16_t synthesize(volatile Oscillator &osc1, volatile Oscillator &osc2,
                   volatile uint32_t &phaseA, volatile uint32_t &phaseB) {
  // LFO processing
  // float raw_lfo = generateLfoWaveform(lfo);
  float raw_lfo = Q9_TO_FLOAT(generateLfoWaveform_fixed(lfo));
  float current_combine_alpha = combine_alpha;

  // LFOがアルファに影響する場合
  if (lfoForAlpha) {
    current_combine_alpha = constrain(
      current_combine_alpha + (raw_lfo * lfo.amplitude), 0.0f, 1.0f);
  }

  // Phase updates with LFO
  uint32_t prevPhaseB = phaseB;  // シンク用に前の値を保存

  // 個別のピッチモジュレーション
  if (lfoForPitchA) {
    float pitch_mod = 1.0f + (raw_lfo * lfo.amplitude);
    phaseA += (uint32_t)((float)osc1.frequency * pitch_mod * (float)(1ULL << 32) / sampleRate);
  } else {
    phaseA += osc1.phase_delta;
  }

  if (lfoForPitchB) {
    float pitch_mod = 1.0f + (raw_lfo * lfo.amplitude);
    phaseB += (uint32_t)((float)osc2.frequency * pitch_mod * (float)(1ULL << 32) / sampleRate);
  } else {
    phaseB += osc2.phase_delta;
  }

  // シンク処理
  if (osc1.is_sync && (prevPhaseB > phaseB)) {
    phaseA = 0;
  }

  // 波形の生成
  uint32_t indexA = (phaseA >> 22) & 0x3FF;
  uint32_t indexB = (phaseB >> 22) & 0x3FF;
  int16_t sampleA = availableBanksOSC1[osc1.bank_index]
                      .waves[static_cast<int>(osc1.waveform)][indexA];

  // OSC2の波形生成 - MIRRORピンがアクティブな時はOSC1のバンクを使用
  int16_t sampleB;
  if (digitalRead(OSCB_COPY_PIN)) {
    sampleB = availableBanksOSC1[osc1.bank_index]
                .waves[static_cast<int>(osc2.waveform)][indexB];
  } else {
    sampleB =
      availableBanksOSC2[0].waves[static_cast<int>(osc2.waveform)][indexB];
  }

  int32_t result;

  switch (currentMethod) {
    case FADE:
      {
        // クロスフェード
        const int32_t scale = 1024;
        int32_t alpha_fixed = (int32_t)(current_combine_alpha * 1024.0f);
        result = ((int32_t)sampleA * (scale - alpha_fixed) + (int32_t)sampleB * alpha_fixed) >> 10;
      }
      break;

    case PHASE:
      {
        // Phase distortion
        int32_t phase_offset = (int32_t)sampleB * 12;
        phase_offset =
          (phase_offset * (int32_t)(current_combine_alpha * 1024)) >> 10;
        phase_offset = phase_offset / 32;
        uint32_t final_index = (indexA + phase_offset) & 0x3FF;
        result = availableBanksOSC1[osc1.bank_index]
                   .waves[static_cast<int>(osc1.waveform)][final_index];
      }
      break;

    case MULT:
      {
        // 固定小数点での乗算と正規化
        int32_t modulated = ((int32_t)sampleA * (int32_t)sampleB) >> 9;

        // ドライ/ウェットミックス
        int32_t alpha_fixed = (int32_t)(current_combine_alpha * 1024.0f);
        result =
          ((int32_t)sampleA * (1024 - alpha_fixed) + modulated * alpha_fixed) >> 10;

        // DCオフセット除去（固定小数点版）
        static int32_t dc_offset = 0;
        dc_offset = dc_offset - (dc_offset >> 8) + (result >> 8);
        result -= dc_offset >> 8;
      }
      break;

    default:
      result = 0;
      break;
  }

  // Update phases and LFO
  lfo.phase += lfo.phase_delta;
  osc1.phase = phaseA;
  osc2.phase = phaseB;

  return constrain(result * 64, -32768, 32767);
}

// Pitch A/B更新
void updateOscillatorFrequencies() {
  int deadZoneSizeA = 50;  // OSC A用
  int deadZoneSizeB = 50;

  // OSC A の周波数計算
  float baseFreqA =
    ROOT_FREQ * exp2f(global_voltage);  // CV入力を含めた基準周波数

  if (abs(filteredMcpRawValues[0] - centerOffset0) <= deadZoneSizeA) {
    osc1.frequency = baseFreqA;
  } else if (filteredMcpRawValues[0] > centerOffset0) {
    float cents = map(filteredMcpRawValues[0], centerOffset0 + deadZoneSizeA,
                      maxAdcValue, 0, 3600);
    osc1.frequency = baseFreqA * centToFreqMultiplier(cents);
  } else {
    float cents = map(filteredMcpRawValues[0], 0, centerOffset0 - deadZoneSizeA,
                      -3600, 0);
    osc1.frequency = baseFreqA * centToFreqMultiplier(cents);
  }
  updatePhaseData(osc1);

  // OSC B の周波数計算
  if (osc2.is_detune) {
    // デチューンモード (以前のコードと同じ)
    if (abs(filteredMcpRawValues[1] - centerOffset2) <= deadZoneSizeB) {
      detune_cents = 0;
    } else if (filteredMcpRawValues[1] > centerOffset2) {
      detune_cents = map(filteredMcpRawValues[1], centerOffset2 + deadZoneSizeB,
                         maxAdcValue2, 0, 1200);
    } else {
      detune_cents = map(filteredMcpRawValues[1], 0,
                         centerOffset2 - deadZoneSizeB, -1200, 0);
    }
    osc2.frequency = osc1.frequency * centToFreqMultiplier(detune_cents);
  } else {
    // フリーモード (以前のコードと同じ)
    float baseFreqB = ROOT_FREQ * exp2f(global_voltage);
    if (abs(filteredMcpRawValues[1] - centerOffset2) <= deadZoneSizeB) {
      osc2.frequency = baseFreqB;
    } else if (filteredMcpRawValues[1] > centerOffset2) {
      float cents = map(filteredMcpRawValues[1], centerOffset2 + deadZoneSizeB,
                        maxAdcValue2, 0, 3600);
      osc2.frequency = baseFreqB * centToFreqMultiplier(cents);
    } else {
      float cents = map(filteredMcpRawValues[1], 0,
                        centerOffset2 - deadZoneSizeB, -3600, 0);
      osc2.frequency = baseFreqB * centToFreqMultiplier(cents);
    }
  }
  updatePhaseData(osc2);
}

// CV mix更新
void updateCVMix() {
  combine_alpha =
    constrain(map(knobAlpha, 0, cvAlpha, 0, 1000) / 1000.0f, 0.0f, 1.0f);

  if (cvForm != 0) {
    osc1.waveform = constrain(
      (static_cast<float>(knobWaveformA) / static_cast<float>(cvForm)) * 15.0f,
      0.0f, 15.0f);
    osc2.waveform = constrain(
      (static_cast<float>(knobWaveformB) / static_cast<float>(cvForm)) * 15.0f,
      0.0f, 15.0f);
  } else {
    osc1.waveform = 0.0f;
    osc2.waveform = 0.0f;
  }
}

// Wave buffer generation
void generate_wave(int16_t *buffer, size_t size, volatile Oscillator &osc1,
                   volatile Oscillator &osc2) {
  if (!isOscillateStarted)
    return;

  static uint32_t last_phase1 = 0;
  static uint32_t last_phase2 = 0;

  // 前回のバッファ終了時の位相を使用して開始
  if (buffer_ready) {
    osc1.phase = last_phase1;
    osc2.phase = last_phase2;
  }

  for (size_t i = 0; i < size; i++) {
    // Ultra low latency: Check for pitch CV changes every 8 samples
    // Temporarily disabled for debugging
    /*
    if ((i & 0x7) == 0) {  // Every 8 samples (reduces overhead)
      static float lastRootFreq = ROOT_FREQ;
      if (ROOT_FREQ != lastRootFreq) {
        lastRootFreq = ROOT_FREQ;
        // Update phase deltas immediately within buffer generation
        updatePhaseData(osc1);
        updatePhaseData(osc2);
      }
    }
    */
    
    buffer[i] = synthesize(osc1, osc2, osc1.phase, osc2.phase);
  }

  // 次のバッファのために最後の位相を保存
  last_phase1 = osc1.phase;
  last_phase2 = osc2.phase;

  buffer_ready = true;
}

/*
 * K102 Synthesizer - ADC and Parameter Update Functions
 */
// === ADC Processing Functions ===
bool readAdcWithTimeout(int channel, int &result,
                        unsigned long timeout_ms) {
  // 関数の先頭で静的変数を宣言
  static bool severeErrorDetected = false;
  static unsigned long severeErrorTime = 0;

  unsigned long startTime = millis();

  // クリティカルセクション開始（割り込み無効化）
  noInterrupts();

  bool success = false;
  for (int retry = 0; retry < 3 && !success; retry++) {  // 最大3回リトライ
    // try-catchの代わりに直接読み取りを試みる
    int rawValue = adc.read(channel);

    // 値が範囲内であれば成功と判断
    if (rawValue >= 0 && rawValue <= 4096) {
      result = rawValue;
      success = true;
      break;  // 成功したらループを抜ける
    }

    // エラー時は少し待ってリトライ
    delayMicroseconds(100);
  }

  // クリティカルセクション終了（割り込み再有効化）
  interrupts();

  if (!success) {
    errorCountSPI++;
    consecutiveErrors++;
#if !PRODUCTION_BUILD
    debugMem->spiErrorCount = errorCountSPI;
#endif

    // エラー発生時には前回の有効な値を使用
    result = prevMcpSamples[channel][0];

    // 連続エラーが多すぎる場合でもすぐにはWDTリセットを引き起こさない
    if (consecutiveErrors > 50) {  // より多くのエラーを許容
      // 重大なエラーフラグをセット
      if (!severeErrorDetected) {
        severeErrorDetected = true;
        severeErrorTime = millis();
      } else if (millis() - severeErrorTime > 5000) {  // 5秒間エラーが続いたら
        // この場合のみ、後でWDTリセットを誘発
        return false;
      }
    }
  } else {
    consecutiveErrors = 0;  // 成功したらカウンターリセット
    // 重大なエラーフラグをクリア
    severeErrorDetected = false;
  }

  return true;  // エラーでも一応値は返す（前回の値）
}
void getMcpValues() {
  static unsigned long lastSpiAccessTime = 0;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    int rawValue;
    if (readAdcWithTimeout(i, rawValue)) {

      // ★唯一の補正ポイント
      if (i == 1) {  // CH1 = OSC B Pitch
        rawValue = (int)(rawValue * CH1_SCALE + 0.5f);
        if (rawValue > ADC_MAX) rawValue = ADC_MAX;
      }
      filteredMcpRawValues[i] = FILTER_ALPHA[i] * rawValue + (1 - FILTER_ALPHA[i]) * filteredMcpRawValues[i];

      // サンプル履歴の更新
      for (int j = NUM_SAMPLES - 1; j > 0; j--) {
        prevMcpSamples[i][j] = prevMcpSamples[i][j - 1];
      }
      prevMcpSamples[i][0] = filteredMcpRawValues[i];

      // 値変更の検出
      bool valueChanged = false;
      for (int j = 0; j < NUM_SAMPLES - 1; j++) {
        if (abs(prevMcpSamples[i][j] - prevMcpSamples[i][j + 1]) > 1) {
          valueChanged = true;
          break;
        }
      }

      // 変更カウンターの更新
      if (valueChanged) {
        changeMcpSampleCount[i] = NUM_SAMPLES;
      } else if (changeMcpSampleCount[i] > 0) {
        changeMcpSampleCount[i]--;
      }

      mcpValueChanged[i] = (changeMcpSampleCount[i] > 0);
    } else {
      // タイムアウトした場合は前回の値を使用
      errorCountADC++;
#if !PRODUCTION_BUILD
      debugMem->adcErrorCount = errorCountADC;
#endif
      // 値変更フラグはオフに
      mcpValueChanged[i] = false;
    }
  }

  // SPI利用時間を記録
  lastSpiAccessTime = micros();

  // WDT更新
  updateWatchdog();
}

void getInternalAdcValue() {
  const int adcPins[3] = { INTERNAL_ADC_PIN_1, INTERNAL_ADC_PIN_2,
                           INTERNAL_ADC_PIN_3 };

  for (int i = 0; i < 3; i++) {
    int rawValue = analogRead(adcPins[i]);
    filteredInternalAdcValues[i] =
      FILTER_ALPHA[0] * rawValue + (1 - FILTER_ALPHA[0]) * filteredInternalAdcValues[i];

    // Update sample history
    for (int j = NUM_SAMPLES - 1; j > 0; j--) {
      prevInternalAdcSamples[i][j] = prevInternalAdcSamples[i][j - 1];
    }
    prevInternalAdcSamples[i][0] = filteredInternalAdcValues[i];

    // Check for value changes
    bool valueChanged = false;
    for (int j = 0; j < NUM_SAMPLES - 1; j++) {
      if (abs(prevInternalAdcSamples[i][j] - prevInternalAdcSamples[i][j + 1]) > 1) {
        valueChanged = true;
        break;
      }
    }

    // Update change counters
    if (valueChanged) {
      changeInternalAdcSampleCounts[i] = NUM_SAMPLES;
    } else if (changeInternalAdcSampleCounts[i] > 0) {
      changeInternalAdcSampleCounts[i]--;
    }

    internalAdcValueChanged[i] = (changeInternalAdcSampleCounts[i] > 0);
  }

  // 内部ADC読み取り後もWDT更新
  updateWatchdog();
}

// === Parameter Update Function ===
void updateSynthParameters() {
  float rootFrequency = ROOT_FREQ;
  float baseFrequency = rootFrequency;
  float subFrequency = rootFrequency;
  float detuneCents = 0;

  // OSC A Pitch (CH0)
  if (mcpValueChanged[0]) {
    updateOscillatorFrequencies();
  }

  // OSC B Detune (CH1)
  if (mcpValueChanged[1]) {
    updateOscillatorFrequencies();
  }

  // LFO Frequency (CH2)
  if (mcpValueChanged[2]) {
    lfo.frequency =
      logMap(filteredMcpRawValues[2], 0, maxAdcValue, 0.005f, 50.0f);
    updateLfoPhaseData(lfo);
  }

  // LFO Amplitude (CH3)
  if (mcpValueChanged[3]) {
    // Add deadzone at the bottom of the range
    if (filteredMcpRawValues[3] < 50) {  // Small deadzone at minimum
      lfo.amplitude = 0.0f;
    } else {
      lfo.amplitude =
        map(filteredMcpRawValues[3], 50, maxAdcValue, 0, 1000) / 1000.0f;
    }
  }

  // Pitch CV Input (CH5)処理とグローバル電圧更新
  float voltage = (filteredMcpRawValues[5] / 4095.0f) * 5.0f;
  global_voltage = voltage;

  // Internal ADC Processing
  // OSC A Bank Index (PIN28)
  if (internalAdcValueChanged[1]) {
    osc1.bank_index =
      map(filteredInternalAdcValues[1], 0, 4095, 0, NUM_BANKS_OSC1);
  }

  // Knob Alpha (CH4)
  if (mcpValueChanged[4]) {
    knobAlpha = filteredMcpRawValues[4];
  }

  // OSC A Waveform (PIN29)
  if (internalAdcValueChanged[0]) {
    knobWaveformA = filteredInternalAdcValues[0];
  }

  // OSC B Waveform (PIN27)
  if (internalAdcValueChanged[2]) {
    knobWaveformB = filteredInternalAdcValues[2];
  }

  // CV alpha
  if (mcpValueChanged[6]) {
    cvAlpha = filteredMcpRawValues[6];
  }

  // CV waveform both
  if (mcpValueChanged[7]) {
    cvForm = filteredMcpRawValues[7];
  }
}

// LEDの更新関数 - LFOと同期させつつ負荷を軽減
void updateLedSafely() {
  static unsigned long lastLedUpdate = 0;
  unsigned long currentTime =
    millis();  // microsではなくmillisを使用（より適切な時間スケール）

  // 更新頻度を制限（60ms = 約16Hz, 人間の目には十分滑らか）
  if (currentTime - lastLedUpdate >= 60) {
    lastLedUpdate = currentTime;

    // LFOの位相から直接LEDの輝度を計算
    uint8_t ledBrightness;

    if (lfo.amplitude > 0.01f) {
      // LFOの位相から輝度を直接計算（サイン波形のように）
      uint32_t lfoPhaseIndex = (lfo.phase >> 22) & 0x3FF;  // 0-1023の範囲

      // 簡単な計算で輝度を求める（サイン波に近い効果）
      if (lfoPhaseIndex < 512) {
        // 上昇部分
        ledBrightness = map(lfoPhaseIndex, 0, 511, NEOPIXEL_DARKEST, 128);
      } else {
        // 下降部分
        ledBrightness = map(lfoPhaseIndex, 512, 1023, 128, NEOPIXEL_DARKEST);
      }
    } else {
      // LFOがオフの場合は固定輝度
      ledBrightness = 48;  // 低めの輝度に設定
    }

    // 合成方法でLEDの色を設定（変更なし）
    switch (currentMethod) {
      case PHASE:
        leds[0] = CRGB(ledBrightness, 0, osc2.is_detune ? 40 : 0);
        break;
      case FADE:
        leds[0] = CRGB(0, ledBrightness, osc2.is_detune ? 40 : 0);
        break;
      case MULT:
        leds[0] =
          CRGB(ledBrightness / 2, ledBrightness / 2, osc2.is_detune ? 40 : 0);
        break;
    }

    // クリティカルセクション（LED更新中は割り込み無効）
    // これはFastLED.show()の前後だけに限定
    noInterrupts();
    FastLED.show();
    interrupts();
  }
}
/*
 * K102 Synthesizer - Serial Command Processing (REMOVED)
 * Now using constants only for simplicity
 */

// Command mode functions removed for simplicity

void removedCommandMode() {  // Renamed to avoid conflict
  Serial.println("============================");
  Serial.println("K102E Command Mode");
  Serial.println("Available commands:");
  Serial.println("B: Enter Bootloader Mode");
#if !PRODUCTION_BUILD
  Serial.println("D: Diagnostic Information");
#endif
  Serial.println("S: Display Serial Number and Version");
  Serial.println("C: Clear Error Counters");
  Serial.println("H: Help (display this menu)");
  Serial.println("============================");

  while (true) {  // 永続ループ
    if (Serial.available() > 0) {
      char cmd = Serial.read();

      switch (toupper(cmd)) {
        case 'B':
          Serial.println();
          Serial.println("Entering Bootloader Mode...");
          delay(500);
          rp2040.rebootToBootloader();
          break;

        case 'W':
          {
            Serial.println();
            uint32_t resetCount = 0; // Default value since EEPROM removed
            // EEPROM.get(WDT_RESET_COUNT_ADDR, resetCount); // EEPROM removed
            Serial.print("Watchdog Reset Count: ");
            Serial.println("DISABLED (EEPROM removed)");
          }
          break;

        case 'S':
          Serial.println();
          Serial.print("Model: ");
          Serial.println(MODEL_NAME);
          Serial.print("Version: ");
          Serial.println(VERSION);
          Serial.print("PCB Version: ");
          Serial.println(PCB_VERSION);

          Serial.print("Serial Number: ");
          pico_unique_board_id_t board_id;
          pico_get_unique_board_id(&board_id);
          for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
            Serial.print(board_id.id[i], HEX);
            if (i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1)
              Serial.print(":");
          }
          Serial.println();
          break;

        case 'D':
#if !PRODUCTION_BUILD
          Serial.println();
          Serial.println("--- Diagnostic Information ---");
          Serial.print("SPI Error Count: ");
          Serial.println(errorCountSPI);
          Serial.print("ADC Error Count: ");
          Serial.println(errorCountADC);
          Serial.print("Core1 Error Count: ");
          Serial.println(errorCountCore1);
          Serial.print("Core0 Heartbeat: ");
          Serial.println(lastHeartbeatCore0);
          Serial.print("Core1 Heartbeat: ");
          Serial.println(lastHeartbeatCore1);
          Serial.print("Free Memory: ");
          Serial.println(rp2040.getFreeHeap());
          uniqueCode = generateOneTimeCode();
          Serial.print("Dump: ");
          Serial.println(uniqueCode);
          Serial.println("----------------------------");
#else
          Serial.println("Command not available in production mode.");
#endif
          break;

        case 'C':
          errorCountSPI = 0;
          errorCountADC = 0;
          errorCountCore1 = 0;
          consecutiveErrors = 0;
          Serial.println();
          Serial.println("Error counters cleared.");
          break;

        case 'Z':
          // WDTリセットカウンターをリセット

          // EEPROM.put(WDT_RESET_COUNT_ADDR, 0); // EEPROM removed
          // EEPROM.commit();
          Serial.println();
          Serial.println("Watchdog Reset Count cleared.");
          break;

        case 'H':
          // ヘルプを再表示
          Serial.println("============================");
          Serial.println("K102E Command Mode");
          Serial.println("Available commands:");
          Serial.println("B: Enter Bootloader Mode");
#if !PRODUCTION_BUILD
          Serial.println("D: Diagnostic Information");
#endif
          Serial.println("S: Display Serial Number and Version");
          Serial.println("C: Clear Error Counters");
          Serial.println("H: Help (display this menu)");
          Serial.println("============================");
          break;

        default:
          // 改行やスペースなどは無視
          if (cmd != '\r' && cmd != '\n' && cmd != ' ') {
            Serial.println("Unknown command. Type 'H' for help.");
          }
          break;
      }

      // 残りの入力をクリア
      while (Serial.available()) {
        Serial.read();
      }
    }

    // コマンドモード中もWDTを更新
    updateWatchdog();
    delay(10);
  }
}

void printMemoryStats() {
  Serial.print("Total heap: ");
  Serial.print(rp2040.getTotalHeap());
  Serial.print(" bytes, Free heap: ");
  Serial.print(rp2040.getFreeHeap());
  Serial.print(" bytes, Used: ");
  Serial.print(rp2040.getTotalHeap() - rp2040.getFreeHeap());
  Serial.println(" bytes");
}

void checkHardwareStatus() {
  Serial.println("--- Hardware Status ---");

  // ADC Readings
  Serial.print("ADC0: ");
  Serial.print(filteredMcpRawValues[0]);
  Serial.print(" | ADC1: ");
  Serial.println(filteredMcpRawValues[1]);

  // GPIO States
  Serial.print("METHOD_A: ");
  Serial.print(!digitalRead(METHOD_A_PIN));
  Serial.print(" | METHOD_B: ");
  Serial.print(!digitalRead(METHOD_B_PIN));
  Serial.print(" | SYNC: ");
  Serial.print(digitalRead(SYNC_PIN));
  Serial.print(" | DETUNE: ");
  Serial.println(digitalRead(DETUNE_PIN));

  // Oscillator state
  Serial.print("OSC1 freq: ");
  Serial.print(osc1.frequency);
  Serial.print(" | OSC2 freq: ");
  Serial.println(osc2.frequency);

  // Current Method
  Serial.print("Method: ");
  switch (currentMethod) {
    case FADE:
      Serial.println("FADE");
      break;
    case PHASE:
      Serial.println("PHASE");
      break;
    case MULT:
      Serial.println("MULT");
      break;
    default:
      Serial.println("UNKNOWN");
  }

  Serial.println("---------------------");
}

// EEPROM functions removed - using constants only

/*
 * K102 Synthesizer - Setup and Main Loop Functions
 */

// === Setup Function ===
void setup() {

  set_sys_clock_khz(133000, true);

  uint32_t seed = rp2040.hwrand32();
  randomSeed(seed);  // Arduino互換のrandomSeed()


#if !PRODUCTION_BUILD
  debugMem = (DebugMemory *)0x20030000;
#endif
  // EEPROM removed for simplicity

  // リセット理由を取得
  int resetReason = rp2040.getResetReason();

#if !PRODUCTION_BUILD
  // デバッグメモリの初期化チェック
  if (debugMem->marker != 0xDEADBEEF) {
    // 初回起動時は初期化
    debugMem->marker = 0xDEADBEEF;
    debugMem->resetCounter = 0;
    debugMem->spiErrorCount = 0;
    debugMem->adcErrorCount = 0;
    debugMem->core1ErrorCount = 0;
    debugMem->core0_activity_counter = 0;
    debugMem->core1_activity_counter = 0;
  }
  // リセットカウンターをインクリメント
  debugMem->resetCounter++;

  // 最後のリセット理由を保存
  debugMem->resetReason = resetReason;
#endif

  // シリアルポートを初期化（すべてのケースで）
  Serial.begin(115200);

  // WDTリセットの場合のみカウンターを更新
  if (resetReason == 4) {  // 4 = WATCHDOG_RESET
    //   uint32_t resetCount;
    //   EEPROM.get(WDT_RESET_COUNT_ADDR, resetCount);
    //   if (resetCount < MAX_WDT_COUNT) {
    //     resetCount++;
    //     EEPROM.put(WDT_RESET_COUNT_ADDR, resetCount);
    //     EEPROM.commit();
    //   }
  }
  // 電源投入/通常起動時
  else {
    // Brief delay for normal startup
    delay(100);
  }

  // Serial removed for simplicity
  // Serial.end(); // Commented out to allow runtime serial communication

  // 1. 基本的なハードウェア初期化
  analogReadResolution(12);

  // 2. SPI/ADCの初期化
  SPI1.setRX(MISO_PIN);
  SPI1.setTX(MOSI_PIN);
  SPI1.setSCK(SCK_PIN);
  SPI1.begin();
  adc.begin(CS_PIN);

  // 3. GPIOピンの設定
  pinMode(LFO_WAVEFORM_SELECT_PIN, INPUT_PULLUP);
  pinMode(METHOD_A_PIN, INPUT_PULLUP);
  pinMode(METHOD_B_PIN, INPUT_PULLUP);
  pinMode(LFO_EFFECT_1, INPUT_PULLUP);
  pinMode(LFO_EFFECT_2, INPUT_PULLUP);
  pinMode(LFO_WAVEFORM_1, INPUT_PULLUP);
  pinMode(LFO_WAVEFORM_2, INPUT_PULLUP);
  pinMode(SYNC_PIN, INPUT_PULLUP);
  pinMode(DETUNE_PIN, INPUT_PULLUP);
  pinMode(OSCB_COPY_PIN, INPUT_PULLUP);
  pinMode(LFO_WAVE_ALT_PIN, INPUT_PULLUP);

  // 4. オシレータとLFOの初期化（シンプルに一括）
  osc1.phase = osc2.phase = 0;
  osc1.phase_delta = osc2.phase_delta = 0;
  osc1.bank_index = osc2.bank_index = 0;
  osc1.waveform = osc2.waveform = 0.0f;
  osc1.is_sync = osc2.is_sync = false;
  osc1.is_detune = osc2.is_detune = false;

  lfo.phase = 0;
  lfo.phase_delta = 0;
  lfo.frequency = 0.0f;
  lfo.amplitude = 0.0f;
  lfo.waveform = TRIANGLE;
  lfo.current_random_value = 0.0f;
  lfo.last_step_change = 0;
  updateLfoPhaseData(lfo);

  // 5. ADCの初期値を一度だけ読み取り
  getMcpValues();
  getInternalAdcValue();
  updateSynthParameters();
  
  // Settings loading removed - using constants only

  // 6. I2S オーディオの初期化
  i2s.setLSBJFormat();
  i2s.setBCLK(PIN_I2S_BCLK);
  i2s.setDATA(PIN_I2S_DOUT);
  i2s.setBitsPerSample(16);
  i2s.setBuffers(4, BUFFER_SIZE / 2, 0);  // Conservative DMA buffer
  i2s.begin(sampleRate);

  // 7. LED初期化
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // 8. Core1起動
  multicore_launch_core1(core1_main);

  // 9. 初期パラメータの計算と設定
  updateOscillatorFrequencies();

  // ハートビート初期化
  lastHeartbeatCore0 = millis();
  lastHeartbeatCore1 = millis();

  // WDTを2秒のタイムアウトで有効化（最後に）
  watchdog_enable(WDT_TIMEOUT_MS, 1);  // timeout_ms, pause_on_debug=1

  // 音声出力開始の合図
  isOscillateStarted = true;
}

// === Main Loop Functions ===
void loop() {
  static uint32_t loopCounter = 0;
  loopCounter++;
#if !PRODUCTION_BUILD
  debugMem->loopCounter = loopCounter;
  debugMem->core0_activity_counter++;
#endif

#if !PRODUCTION_BUILD
  debugMem->currentZone = 10;  // Loop start
#endif
  static size_t buffer_index = 0;
  static uint32_t last_buffer_time = 0;
  const uint32_t buffer_threshold =
    BUFFER_SIZE * 1000000UL / sampleRate;  // マイクロ秒単位でのバッファ長

  uint32_t current_time = safe_micros();  // micros() → safe_micros()に変更


  // Ultra low latency pitch CV reading before buffer generation
  // getPitchCvValue();  // Temporarily disabled for debugging
  
  // タイミングを厳密に管理してバッファを生成 + 強制更新フラグ対応
  if (buffer_index == 0 && !should_mute && 
      ((current_time - last_buffer_time >= buffer_threshold) || forceBufferUpdate)) {
#if !PRODUCTION_BUILD
    debugMem->currentZone = 11;  // Before generate_wave
#endif
    generate_wave(buffers[next_buffer], BUFFER_SIZE, osc1, osc2);
    last_buffer_time = current_time;
    buffer_index = BUFFER_SIZE;    // バッファ内のサンプル数を設定
    current_buffer = next_buffer;  // 現在のバッファを更新
    next_buffer = (next_buffer + 1) % NUM_BUFFERS;
    forceBufferUpdate = false;     // フラグをリセット
  }

#if !PRODUCTION_BUILD
  debugMem->currentZone = 12;
#endif                                                    // Before I2S check
  if (i2s.availableForWrite() > 0 && buffer_index > 0) {  // バッファにサンプルがある場合
    if (!should_mute) {
#if !PRODUCTION_BUILD
      debugMem->currentZone = 13;  // Just before i2s.write
#endif
      // バッファから次のサンプルを取得
      int16_t sample = buffers[current_buffer][BUFFER_SIZE - buffer_index];
      int32_t stereo_sample = (sample << 16) | (sample & 0xffff);  // ステレオ化
      size_t written = i2s.write(stereo_sample, false);
      if (written > 0) {
        buffer_index--;  // バッファ位置を進める
      } else {
        // 書き込み失敗時は再試行
        i2s.write(stereo_sample, false);
#if !PRODUCTION_BUILD
        debugMem->lastErrorZone = 200;
#endif
      }
    } else {
      i2s.write(0, false);  // ミュート時はゼロ出力
      buffer_index--;       // バッファ位置を進める
    }
  }

// Core間ハートビート監視
#if !PRODUCTION_BUILD
  debugMem->currentZone = 14;
  debugMem->currentZone = 140;
  debugMem->currentZone = 141;
#endif
  checkCoreHeartbeats();
#if !PRODUCTION_BUILD
  debugMem->currentZone = 142;
#endif
  ;  // Core1応答チェック後


  // Serial processing removed for simplicity

  // 定期的にWDTを更新
  static uint32_t lastWdtTime = 0;
  uint32_t current_millis = safe_millis();  // millis() → safe_millis()に変更
  if (current_millis - lastWdtTime >= WDT_UPDATE_INTERVAL) {
#if !PRODUCTION_BUILD
    debugMem->currentZone = 15;
#endif
    lastWdtTime = current_millis;
    watchdog_update();
#if !PRODUCTION_BUILD
    debugMem->currentZone = 16;
#endif
  }
}
// === Core 1 Main Function ===
void core1_main() {
  static int initialLoopCount = 0;
  const int FORCE_UPDATE_LOOPS = 50;

  unsigned long lastWdtTime = 0;
  unsigned long lastParameterUpdate = 0;

  // Core1開始を通知
  lastHeartbeatCore1 = safe_millis();  // millis() → safe_millis()に変更

// 活動カウンター初期化
#if !PRODUCTION_BUILD
  debugMem->core1_activity_counter = 0;
#endif

  while (1) {
// 活動カウンターの更新
#if !PRODUCTION_BUILD
    debugMem->core1_activity_counter++;
#endif

#if !PRODUCTION_BUILD
    debugMem->currentZone = 200;  // Core1 loop start
#endif
    // タイムスタンプ取得
    unsigned long currentMillis = safe_millis();  // millis() → safe_millis()に変更
    unsigned long currentMicros = safe_micros();  // micros() → safe_micros()に変更


    // パラメータ更新（一定間隔で）
    if (currentMicros - lastParameterUpdate >= (INTERVAL * 1000)) {
      lastParameterUpdate = currentMicros;

// ADC読み取り（エラーハンドリング強化）
#if !PRODUCTION_BUILD
      debugMem->currentZone = 210;  // MCP values前
#endif
      getMcpValues();
#if !PRODUCTION_BUILD
      debugMem->currentZone = 220;  // internal ADC前
#endif
      getInternalAdcValue();

// パラメータ更新
#if !PRODUCTION_BUILD
      debugMem->currentZone = 230;  // synth param更新前
#endif
      updateSynthParameters();
#if !PRODUCTION_BUILD
      debugMem->currentZone = 240;  // oscillator freq前
#endif
      updateOscillatorFrequencies();
      updateCVMix();
      
      // Force buffer regeneration for low latency pitch CV response
      // This reduces latency from buffer_threshold to parameter update interval
      forceBufferUpdate = true;

      if (initialLoopCount < FORCE_UPDATE_LOOPS) {
        initialLoopCount++;
      } else if (initialLoopCount == FORCE_UPDATE_LOOPS) {
        isOscillateStarted = true;
      }
    }

    // コントロール入力の読み取りと設定の更新
    // (頻度を下げて負荷軽減)
    static unsigned long lastControlUpdate = 0;
    if (currentMillis - lastControlUpdate >= 5) {  // 5msごとに更新
      lastControlUpdate = currentMillis;

      // メソッド選択
      bool methodA = !digitalRead(METHOD_A_PIN);
      bool methodB = !digitalRead(METHOD_B_PIN);
      if (methodA && !methodB) {
        currentMethod = PHASE;
      } else if (!methodA && methodB) {
        currentMethod = FADE;
      } else {
        currentMethod = MULT;
      }

      // OSC1 Sync & OSC2 Detune
      osc1.is_sync = digitalRead(SYNC_PIN);
      osc2.is_detune = digitalRead(DETUNE_PIN);

      // LFO設定
      bool lfo1 = digitalRead(LFO_EFFECT_2);
      bool lfo2 = digitalRead(LFO_EFFECT_1);

      // LFO destination設定
      if (lfo1 && !lfo2) {
        lfoForPitchA = true;
        lfoForPitchB = true;
        lfoForAlpha = false;
      } else if (lfo1 && lfo2) {
        lfoForPitchA = false;
        lfoForPitchB = true;
        lfoForAlpha = false;
      } else if (!lfo1 && lfo2) {
        lfoForPitchA = false;
        lfoForPitchB = false;
        lfoForAlpha = true;
      }

      // LFO波形設定
      bool lfoForm1 = digitalRead(LFO_WAVEFORM_1);
      bool lfoForm2 = digitalRead(LFO_WAVEFORM_2);
      bool lfoFormAlt = digitalRead(LFO_WAVE_ALT_PIN);

      if (lfoFormAlt == false) {
        if (lfoForm1 && !lfoForm2) {
          lfo.waveform = TRIANGLE;
        } else if (!lfoForm1 && lfoForm2) {
          lfo.waveform = SQUARE;
        } else {
          lfo.waveform = RAMP_UP;
        }
      } else {
        if (lfoForm1 && !lfoForm2) {
          lfo.waveform = SINE;
        } else if (!lfoForm1 && lfoForm2) {
          lfo.waveform = RANDOM;
        } else {
          lfo.waveform = RAMP_DOWN;
        }
      }
    }

    // LEDの安全な更新
    updateLedSafely();

    // WDT更新とハートビート
    if (currentMillis - lastWdtTime >= WDT_UPDATE_INTERVAL) {
      lastHeartbeatCore1 = currentMillis;  // ハートビート更新
      updateWatchdog();
      lastWdtTime = currentMillis;
    }

    // ループ内で少し待機してCPU負荷を抑える
    sleep_us(250);
  }
}