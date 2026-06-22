        .export _sio_call

        .segment "CODE"

_sio_call:
        jsr $E459
        lda $0303
        ldx #$00
        rts
