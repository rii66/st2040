#include "adc_dma.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ============================================================
// CONFIG
// ============================================================
#define NUM_CHANNELS          3
#define SAMPLES_PER_CHANNEL   32
#define BUFFER_LEN            (SAMPLES_PER_CHANNEL * NUM_CHANNELS)
#define ROUND_ROBIN_MASK      0b00000111   // ADC0 + ADC1 + ADC2

// ============================================================
// BUFFERS
// ============================================================
static uint16_t bufferA[BUFFER_LEN] __attribute__((aligned(4)));
static uint16_t bufferB[BUFFER_LEN] __attribute__((aligned(4)));

static volatile bool      write_ping   = true;   // true = sedang menulis ke bufferA
static volatile bool      data_ready   = false;
static volatile uint16_t* ready_buf    = nullptr;

static int dma_chan = -1;

static uint16_t avgTip = 0;
static uint16_t avgHotAir = 0;
static uint16_t avgFan = 0;
static uint32_t sampleCount = 0;

// ============================================================
// PROCESS (hitung average)
// ============================================================
static void processBuffer(const uint16_t* buf) {
    uint32_t sum0 = 0, sum1 = 0, sum2 = 0;

    for (int i = 0; i < SAMPLES_PER_CHANNEL; i++) {
        sum0 += buf[i * NUM_CHANNELS + 0]; // Tip   (GP26)
        sum1 += buf[i * NUM_CHANNELS + 1]; // HotAir (GP27)
        sum2 += buf[i * NUM_CHANNELS + 2]; // Fan   (GP28)
    }

    avgTip    = sum0 / SAMPLES_PER_CHANNEL;
    avgHotAir = sum1 / SAMPLES_PER_CHANNEL;
    avgFan    = sum2 / SAMPLES_PER_CHANNEL;
    sampleCount += SAMPLES_PER_CHANNEL;
}

// ============================================================
// DMA IRQ HANDLER
// ============================================================
static void __not_in_flash_func(dma_handler)() {
    // Clear interrupt
    dma_hw->ints0 = 1u << dma_chan;

    // Buffer yang baru saja penuh
    if (write_ping) {
        ready_buf  = bufferA;
        write_ping = false;
        // Mulai isi bufferB
        dma_channel_set_write_addr(dma_chan, bufferB, false);
    } else {
        ready_buf  = bufferB;
        write_ping = true;
        // Mulai isi bufferA
        dma_channel_set_write_addr(dma_chan, bufferA, false);
    }

    // Restart transfer dengan jumlah yang sama
    dma_channel_set_trans_count(dma_chan, BUFFER_LEN, true);

    data_ready = true;
}

// ============================================================
// INIT
// ============================================================
void adcDmaBegin() {
    // --- ADC ---
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    adc_select_input(0);                    // mulai dari channel 0
    adc_set_round_robin(ROUND_ROBIN_MASK);

    adc_fifo_setup(
        true,   // write each conversion to FIFO
        true,   // enable DREQ
        1,      // DREQ when ≥1 sample
        false,  // no error bit
        false   // 12-bit
    );

    adc_set_clkdiv(0);  // full speed (\~500 kS/s total)

    // --- DMA ---
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);   // FIFO fixed
    channel_config_set_write_increment(&cfg, true);   // buffer naik
    channel_config_set_dreq(&cfg, DREQ_ADC);

    // Start dengan bufferA
    dma_channel_configure(
        dma_chan,
        &cfg,
        bufferA,
        &adc_hw->fifo,
        BUFFER_LEN,
        false               // jangan start dulu
    );

    // IRQ
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Start
    dma_channel_start(dma_chan);
    adc_run(true);
}

// ============================================================
// PUBLIC API
// ============================================================
uint16_t adcGetTip() {
    if (data_ready && ready_buf) {
        processBuffer((const uint16_t*)ready_buf);
        data_ready = false;
    }
    return avgTip;
}

uint16_t adcGetHotAir() {
    adcGetTip();          // pastikan buffer terproses
    return avgHotAir;
}

uint16_t adcGetFanPot() {
    adcGetTip();
    return avgFan;
}

uint32_t adcGetSampleCount() {
    return sampleCount;
}