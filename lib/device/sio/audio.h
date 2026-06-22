#ifndef SIO_AUDIO_H
#define SIO_AUDIO_H

#include <cassert>

#include "audioDevice.h"
#include "bus.h"

class sioAudio : public virtualDevice, public audioDevice
{
protected:
    transState_t _transaction_state = TRANS_STATE::INVALID;

    void transaction_begin(transState_t expectMoreData) override;
    void transaction_complete() override;
    void transaction_error() override;
    success_is_true transaction_get(void *data, size_t len) override;
    void transaction_put(const void *data, size_t len, bool err = false) override;

    void sio_status() override { audiocmd_status(); }
    void sio_process(uint32_t commanddata, uint8_t checksum) override;

public:
    void setup() override { audioDevice::setup(); }
};

extern sioAudio audioDev;

#endif // SIO_AUDIO_H
