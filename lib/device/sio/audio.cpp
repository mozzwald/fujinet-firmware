#ifdef BUILD_ATARI

#include "audio.h"

#include "fujiCommandID.h"

sioAudio audioDev;

void sioAudio::transaction_begin(transState_t expectMoreData)
{
    assert(_transaction_state == TRANS_STATE::INVALID);
    _transaction_state = expectMoreData;
    if (expectMoreData == TRANS_STATE::WILL_GET)
        sio_late_ack();
    else
        sio_ack();
}

void sioAudio::transaction_complete()
{
    assert(_transaction_state == TRANS_STATE::NO_GET || _transaction_state == TRANS_STATE::DID_GET);
    sio_complete();
    _transaction_state = TRANS_STATE::INVALID;
}

void sioAudio::transaction_error()
{
    if (_transaction_state == TRANS_STATE::INVALID)
        sio_error();
    else
        sio_nak();
    _transaction_state = TRANS_STATE::INVALID;
}

success_is_true sioAudio::transaction_get(void *data, size_t len)
{
    assert(_transaction_state == TRANS_STATE::WILL_GET);
    _transaction_state = TRANS_STATE::DID_GET;

    uint8_t ck = bus_to_peripheral(static_cast<uint8_t *>(data), static_cast<uint16_t>(len));
    if (sio_checksum(static_cast<uint8_t *>(data), static_cast<unsigned short>(len)) != ck)
        RETURN_ERROR_AS_FALSE();

    RETURN_SUCCESS_AS_TRUE();
}

void sioAudio::transaction_put(const void *data, size_t len, bool err)
{
    assert(_transaction_state == TRANS_STATE::NO_GET);
    bus_to_computer((uint8_t *)data, static_cast<uint16_t>(len), err);
    _transaction_state = TRANS_STATE::INVALID;
}

void sioAudio::sio_process(uint32_t commanddata, uint8_t checksum)
{
    cmdFrame.commanddata = commanddata;
    cmdFrame.checksum = checksum;

    switch (cmdFrame.comnd)
    {
    case AUDIOCMD_STATUS:
        audiocmd_status();
        break;
    case AUDIOCMD_CAPABILITIES:
        audiocmd_capabilities(cmdFrame.aux1);
        break;
    case AUDIOCMD_SET_SOURCE:
        audiocmd_set_source(cmdFrame.aux1 | (cmdFrame.aux2 << 8));
        break;
    case AUDIOCMD_PLAY:
        audiocmd_play(cmdFrame.aux1);
        break;
    case AUDIOCMD_PAUSE:
        audiocmd_pause();
        break;
    case AUDIOCMD_RESUME:
        audiocmd_resume();
        break;
    case AUDIOCMD_STOP:
        audiocmd_stop();
        break;
    case AUDIOCMD_SET_VOLUME:
        audiocmd_set_volume(cmdFrame.aux1, true);
        break;
    case AUDIOCMD_GET_INFO:
        audiocmd_get_info();
        break;
    case AUDIOCMD_GET_METADATA:
        audiocmd_get_metadata(cmdFrame.aux1, cmdFrame.aux2);
        break;
    case AUDIOCMD_SEEK:
        audiocmd_seek();
        break;
    default:
        sio_nak();
        break;
    }
}

#endif // BUILD_ATARI
