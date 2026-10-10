#include "robo_duplex_codec.h"

#include "config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#define TAG "RoboDuplexCodec"

namespace {
// Amplitude factor of the volume ceiling. NoAudioCodec scales by (volume / 100)^2,
// so pre-scaling by (ceiling / 100)^2 makes "volume 100" play at the ceiling.
constexpr float kCeilingFactor =
    (SPEAKER_MAX_VOLUME / 100.0f) * (SPEAKER_MAX_VOLUME / 100.0f);
}  // namespace

RoboDuplexAudioCodec::RoboDuplexAudioCodec(int sample_rate, gpio_num_t bclk, gpio_num_t ws,
                                           gpio_num_t dout, gpio_num_t din)
    : NoAudioCodecDuplex(sample_rate, sample_rate, bclk, ws, dout, din) {
    // Microphone + playback reference, interleaved ("MR" for the AFE).
    input_reference_ = true;
    input_channels_ = 2;
    for (auto& tag : tags_) tag.store(kNone);

#ifdef AUDIO_I2S_UNUSED_GPIO_A
    // The old microphone clock pins may still be soldered to the mic: keep them
    // high-impedance so they never fight the shared clock.
    const gpio_config_t unused = {
        .pin_bit_mask = (1ULL << AUDIO_I2S_UNUSED_GPIO_A) | (1ULL << AUDIO_I2S_UNUSED_GPIO_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&unused);
#endif

    i2s_event_callbacks_t tx_callbacks = {};
    tx_callbacks.on_sent = OnSent;
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle_, &tx_callbacks, this));

    i2s_event_callbacks_t rx_callbacks = {};
    rx_callbacks.on_recv = OnRecv;
    rx_callbacks.on_recv_q_ovf = OnRecvOverflow;
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle_, &rx_callbacks, this));

    ESP_LOGI(TAG, "Shared-clock I2S at %d Hz, DMA playback reference for echo cancellation",
             sample_rate);
}

// --- I2S interrupts ---------------------------------------------------------------

bool RoboDuplexAudioCodec::OnSent(i2s_chan_handle_t, i2s_event_data_t* event, void* ctx) {
    auto* self = static_cast<RoboDuplexAudioCodec*>(ctx);
    const uint32_t seq = self->tx_blocks_.load(std::memory_order_relaxed);
    TxBlock& block = self->tx_ring_[seq % kTxRing];
    block.seq.store(kNone, std::memory_order_relaxed);
    const int32_t* src = static_cast<const int32_t*>(event->dma_buf);
    const int frames = std::min<int>(static_cast<int>(event->size / sizeof(int32_t)), kBlock);
    for (int i = 0; i < frames; ++i) {
        block.samples[i] = static_cast<int16_t>(src[i] >> 16);  // what NoAudioCodec wrote
    }
    for (int i = frames; i < kBlock; ++i) block.samples[i] = 0;
    block.seq.store(seq, std::memory_order_release);
    self->tx_blocks_.store(seq + 1, std::memory_order_release);
    return false;
}

bool RoboDuplexAudioCodec::OnRecv(i2s_chan_handle_t, i2s_event_data_t*, void* ctx) {
    auto* self = static_cast<RoboDuplexAudioCodec*>(ctx);
    const uint32_t k = self->rx_blocks_.load(std::memory_order_relaxed);
    // The TX block now playing (the next one to be counted) covers the end of this
    // RX block and the 0-15 ms after it.
    const uint32_t tag = self->tx_running_.load(std::memory_order_relaxed)
                             ? self->tx_blocks_.load(std::memory_order_acquire)
                             : kNone;
    self->tags_[k % kTagRing].store(tag, std::memory_order_relaxed);
    self->rx_blocks_.store(k + 1, std::memory_order_release);
    return false;
}

bool RoboDuplexAudioCodec::OnRecvOverflow(i2s_chan_handle_t, i2s_event_data_t*, void* ctx) {
    auto* self = static_cast<RoboDuplexAudioCodec*>(ctx);
    self->rx_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
}

// --- Channel control ----------------------------------------------------------------

void RoboDuplexAudioCodec::EnableInput(bool enable) {
    if (enable == input_enabled()) {
        return;
    }
    if (enable) {
        // Both directions share the TX bit clock: without TX running the microphone
        // gets no clock. TX plays silence when there is nothing to play.
        EnableOutput(true);
        rx_blocks_.store(0);
        rx_dropped_.store(0);
        for (auto& tag : tags_) tag.store(kNone);
        rx_read_index_ = 0;
        rx_dropped_seen_ = 0;
    }
    NoAudioCodec::EnableInput(enable);
}

void RoboDuplexAudioCodec::EnableOutput(bool enable) {
    if (enable == output_enabled()) {
        return;
    }
    if (!enable && input_enabled()) {
        return;  // the microphone still needs the clock
    }
    if (!enable) tx_running_.store(false);
    NoAudioCodec::EnableOutput(enable);
    if (enable) tx_running_.store(true);
}

// --- Data -----------------------------------------------------------------------------

int RoboDuplexAudioCodec::Write(const int16_t* data, int samples) {
    if (samples <= 0) {
        return 0;
    }
    scaled_.resize(samples);
    if (muted_.load()) {
        std::fill(scaled_.begin(), scaled_.end(), 0);  // keep the I2S timing, play silence
    } else {
        for (int i = 0; i < samples; ++i) {
            scaled_[i] = static_cast<int16_t>(data[i] * kCeilingFactor);
        }
    }
    return NoAudioCodec::Write(scaled_.data(), samples);
}

void RoboDuplexAudioCodec::FillReference(int16_t* dest, int frames, int stride) {
    int i = 0;
    while (i < frames) {
        // Blocks the driver dropped were never read: skip their numbers so the tags
        // stay with the right microphone samples. Applied at a block boundary.
        if (rx_read_index_ % kBlock == 0) {
            const uint32_t dropped = rx_dropped_.load(std::memory_order_relaxed);
            if (dropped != rx_dropped_seen_) {
                rx_read_index_ += static_cast<uint64_t>(dropped - rx_dropped_seen_) * kBlock;
                rx_dropped_seen_ = dropped;
            }
        }
        const uint32_t k = static_cast<uint32_t>(rx_read_index_ / kBlock);
        const int offset = static_cast<int>(rx_read_index_ % kBlock);
        const int run = std::min(frames - i, kBlock - offset);

        // Received blocks run ahead of the reader, never behind it.
        const bool received =
            static_cast<int32_t>(rx_blocks_.load(std::memory_order_acquire) - k) > 0;
        const uint32_t tag =
            received ? tags_[k % kTagRing].load(std::memory_order_relaxed) : kNone;
        const TxBlock* block = nullptr;
        if (tag != kNone) {
            // That TX block is still playing for up to 15 ms: wait for it to finish.
            auto pending = [this, tag]() {
                return static_cast<int32_t>(tx_blocks_.load(std::memory_order_acquire) - tag) <= 0;
            };
            for (int tries = 0; tries < 4 && pending() && tx_running_.load(); ++tries) {
                vTaskDelay(pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1);
            }
            const TxBlock& candidate = tx_ring_[tag % kTxRing];
            if (candidate.seq.load(std::memory_order_acquire) == tag) {
                block = &candidate;
            }
        }
        for (int j = 0; j < run; ++j) {
            dest[(i + j) * stride] = block != nullptr ? block->samples[offset + j] : 0;
        }
        i += run;
        rx_read_index_ += run;
    }
}

int RoboDuplexAudioCodec::Read(int16_t* dest, int samples) {
    const int frames = samples / input_channels_;
    mic_.resize(frames);
    const int got = NoAudioCodec::Read(mic_.data(), frames);
    if (got <= 0) {
        return 0;
    }
    for (int i = 0; i < got; ++i) {
        dest[i * 2] = mic_[i];
    }
    FillReference(dest + 1, got, 2);
    UpdateMicStats(mic_.data(), got);
    return got * 2;
}

void RoboDuplexAudioCodec::UpdateMicStats(const int16_t* mic, int samples) {
    for (int i = 0; i < samples; ++i) {
        const int16_t v = static_cast<int16_t>(std::abs(static_cast<int>(mic[i])));
        if (v > peak_) peak_ = v;
        if (v >= INT16_MAX) ++clipped_;
    }
    const int64_t now = esp_timer_get_time();
    if (now - stats_since_us_ < 10 * 1000000LL) {
        return;
    }
    if (clipped_ > 0) {
        ESP_LOGW(TAG, "Microphone clipped %lu samples in 10 s (too loud or too close)",
                 static_cast<unsigned long>(clipped_));
    }
    ESP_LOGD(TAG, "Microphone peak %.1f dBFS",
             peak_ > 0 ? 20.0f * std::log10(peak_ / 32767.0f) : -96.0f);
    stats_since_us_ = now;
    clipped_ = 0;
    peak_ = 0;
}
