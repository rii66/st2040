#pragma once
#include <Arduino.h>

// Inisialisasi ADC + DMA continuous 3 channel (GP26,27,28)
void adcDmaBegin();

// nilai (non-blocking)
uint16_t adcGetTip();       // GP26 - TEMP_PIN
uint16_t adcGetHotAir();    // GP27 - PIN_HOTAIR_ADC
uint16_t adcGetFanPot();    // GP28 - PIN_POT_FAN

// Optional: cek berapa sample sudah terkumpul di buffer aktif
uint32_t adcGetSampleCount();