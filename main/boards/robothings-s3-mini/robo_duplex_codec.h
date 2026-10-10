// Full-duplex I2S codec for the RoboThings S3 Mini "duplex" wiring: the INMP441
// SCK/WS are wired to the MAX98357A BCLK/LRC, so microphone and speaker run on one
// I2S port, one clock, one sample rate (16 kHz).
//
// Playback reference for the echo canceller, sample-aligned:
// - every TX DMA block that leaves for the speaker is copied (in the I2S "sent"
//   interrupt) into a ring, numbered by a TX block counter. That is exactly what the
//   speaker played: volume, mute and the silence of an output underrun included;
// - every RX DMA block that the microphone fills is tagged (in the I2S "received"
//   interrupt) with the TX block that is playing at that moment;
// - each microphone block is then paired with that TX block. TX and RX share the bit
//   clock and use the same block size, so the offset between them is fixed for as
//   long as the port runs, and the reference always leads the echo by 0-15 ms (the
//   echo canceller needs the reference first, and handles a fixed lead easily).
//
// Also: a mute switch (silence a reply at once without touching the saved volume),
// a speaker volume ceiling and a microphone clipping counter, as in RoboAudioCodec.
#pragma once

#include "codecs/no_audio_codec.h"

#include <driver/i2s_common.h>

#include <atomic>
#include <cstdint>
#include <vector>

class RoboDuplexAudioCodec : public NoAudioCodecDuplex {
public:
    RoboDuplexAudioCodec(int sample_rate, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout,
                         gpio_num_t din);

    void SetMuted(bool muted) { muted_.store(muted); }

    void EnableInput(bool enable) override;
    void EnableOutput(bool enable) override;

protected:
    int Write(const int16_t* data, int samples) override;
    int Read(int16_t* dest, int samples) override;

private:
    static constexpr int kBlock = AUDIO_CODEC_DMA_FRAME_NUM;  // frames per DMA block
    static constexpr int kTxRing = 32;   // TX blocks kept (32 x 15 ms = 480 ms)
    static constexpr int kTagRing = 64;  // RX block tags kept

    // Block numbers are 32-bit and wrap (after ~2 years); kNone marks "no block".
    static constexpr uint32_t kNone = 0xFFFFFFFFu;

    struct TxBlock {
        std::atomic<uint32_t> seq{kNone};  // TX block number stored here
        int16_t samples[kBlock];
    };

    // Written by the I2S interrupts
    TxBlock tx_ring_[kTxRing];
    std::atomic<uint32_t> tx_blocks_{0};   // TX blocks sent since boot
    std::atomic<uint32_t> rx_blocks_{0};   // RX blocks received since input enable
    std::atomic<uint32_t> rx_dropped_{0};  // RX blocks the driver dropped (reader late)
    std::atomic<uint32_t> tags_[kTagRing];  // TX block playing when RX block k ended
    std::atomic<bool> tx_running_{false};

    // Input task only
    uint64_t rx_read_index_ = 0;  // microphone frames consumed since input enable
    uint32_t rx_dropped_seen_ = 0;
    std::vector<int16_t> mic_;
    int64_t stats_since_us_ = 0;
    uint32_t clipped_ = 0;
    int16_t peak_ = 0;

    // Output task only
    std::vector<int16_t> scaled_;
    std::atomic<bool> muted_{false};

    static bool OnSent(i2s_chan_handle_t handle, i2s_event_data_t* event, void* ctx);
    static bool OnRecv(i2s_chan_handle_t handle, i2s_event_data_t* event, void* ctx);
    static bool OnRecvOverflow(i2s_chan_handle_t handle, i2s_event_data_t* event, void* ctx);

    void FillReference(int16_t* dest, int frames, int stride);
    void UpdateMicStats(const int16_t* mic, int samples);
};
