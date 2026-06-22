#include <atari.h>
#include <conio.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AUDIO_DEVICE_ID 0x65
#define AUDIO_UNIT 0x01

#define AUDIOCMD_STATUS 0x53
#define AUDIOCMD_SET_SOURCE 0x4F
#define AUDIOCMD_PLAY 0x50
#define AUDIOCMD_PAUSE 0x41
#define AUDIOCMD_RESUME 0x52
#define AUDIOCMD_STOP 0x58
#define AUDIOCMD_SET_VOLUME 0x56
#define AUDIOCMD_GET_INFO 0x49
#define AUDIOCMD_GET_METADATA 0x4D
#define AUDIOCMD_SEEK 0x4B
#define AUDIOCMD_CAPABILITIES 0x3F

#define SIO_NODATA 0x00
#define SIO_READ 0x40
#define SIO_WRITE 0x80

#define DEFAULT_TIMEOUT_SECONDS 8

extern unsigned char sio_call(void);

static unsigned char buffer[128];
static const char source[] = "gen:test-tone";

static void wait_key(void)
{
    cputs("\r\npress key");
    cgetc();
}

static void print_hex8(unsigned char value)
{
    cprintf("$%02X", value);
}

static void print_bytes(const unsigned char *data, unsigned int len)
{
    unsigned int i;

    for (i = 0; i < len; ++i)
    {
        if ((i & 0x0f) == 0)
            cprintf("\r\n%02X: ", i);
        cprintf("%02X ", data[i]);
    }
    cputs("\r\n");
}

static unsigned char audio_sio(unsigned char command,
                               unsigned char direction,
                               void *data,
                               unsigned int length,
                               unsigned int aux)
{
    OS.dcb.ddevic = AUDIO_DEVICE_ID;
    OS.dcb.dunit = AUDIO_UNIT;
    OS.dcb.dcomnd = command;
    OS.dcb.dstats = direction;
    OS.dcb.dbuf = data;
    OS.dcb.dtimlo = DEFAULT_TIMEOUT_SECONDS;
    OS.dcb.dunuse = 0;
    OS.dcb.dbyt = length;
    OS.dcb.daux = aux;

    return sio_call();
}

static void show_result(const char *label,
                        unsigned char command,
                        unsigned char status,
                        unsigned int response_len)
{
    cprintf("\r\n%s cmd=", label);
    print_hex8(command);
    cputs(" sio=");
    print_hex8(status);
    cprintf(" dstat=");
    print_hex8(OS.dcb.dstats);
    cputs("\r\n");

    if (response_len != 0)
        print_bytes(buffer, response_len);
}

static void cmd_capabilities(void)
{
    unsigned char st;
    memset(buffer, 0, sizeof(buffer));
    st = audio_sio(AUDIOCMD_CAPABILITIES, SIO_READ, buffer, 16, 1);
    show_result("capabilities", AUDIOCMD_CAPABILITIES, st, 16);
}

static void cmd_status(void)
{
    unsigned char st;
    memset(buffer, 0, sizeof(buffer));
    st = audio_sio(AUDIOCMD_STATUS, SIO_READ, buffer, 4, 0);
    show_result("status", AUDIOCMD_STATUS, st, 4);
    cprintf("proto=%u state=%u err=%u flags=%u\r\n",
            buffer[0], buffer[1], buffer[2], buffer[3]);
}

static void cmd_info(void)
{
    unsigned char st;
    memset(buffer, 0, sizeof(buffer));
    st = audio_sio(AUDIOCMD_GET_INFO, SIO_READ, buffer, 32, 0);
    show_result("info", AUDIOCMD_GET_INFO, st, 32);
}

static void cmd_set_source(void)
{
    unsigned char st;
    unsigned int len = (unsigned int)strlen(source);
    st = audio_sio(AUDIOCMD_SET_SOURCE, SIO_WRITE, (void *)source, len, len);
    show_result("set source", AUDIOCMD_SET_SOURCE, st, 0);
    cprintf("source=%s\r\n", source);
}

static void cmd_no_payload(const char *label, unsigned char command)
{
    unsigned char st = audio_sio(command, SIO_NODATA, 0, 0, 0);
    show_result(label, command, st, 0);
}

static void cmd_volume(void)
{
    unsigned char volume = 50;
    unsigned char st = audio_sio(AUDIOCMD_SET_VOLUME, SIO_WRITE, &volume, 1, volume);
    show_result("volume 50", AUDIOCMD_SET_VOLUME, st, 0);
}

static void cmd_metadata(void)
{
    unsigned char st;
    unsigned int aux = 1 | (64U << 8);
    memset(buffer, 0, sizeof(buffer));
    st = audio_sio(AUDIOCMD_GET_METADATA, SIO_READ, buffer, 72, aux);
    show_result("metadata title", AUDIOCMD_GET_METADATA, st, 72);
}

static void cmd_seek(void)
{
    static const unsigned char seek_zero[4] = {0, 0, 0, 0};
    unsigned char st = audio_sio(AUDIOCMD_SEEK, SIO_WRITE, (void *)seek_zero, 4, 0);
    show_result("seek 0", AUDIOCMD_SEEK, st, 0);
}

static void draw_menu(void)
{
    clrscr();
    cputs("fujinet audio test\r\n");
    cputs("device $65 direct sio\r\n\r\n");
    cputs("C capabilities\r\n");
    cputs("S status\r\n");
    cputs("I info\r\n");
    cputs("O set source\r\n");
    cputs("P play\r\n");
    cputs("A pause\r\n");
    cputs("R resume\r\n");
    cputs("X stop\r\n");
    cputs("V volume 50\r\n");
    cputs("M metadata title\r\n");
    cputs("K seek 0\r\n");
    cputs("Q quit\r\n\r\n");
    cputs("select: ");
}

int main(void)
{
    unsigned char key;

    for (;;)
    {
        draw_menu();
        key = cgetc();
        cputc(key);
        cputs("\r\n");

        switch (key)
        {
        case 'c':
        case 'C':
            cmd_capabilities();
            break;
        case 's':
        case 'S':
            cmd_status();
            break;
        case 'i':
        case 'I':
            cmd_info();
            break;
        case 'o':
        case 'O':
            cmd_set_source();
            break;
        case 'p':
        case 'P':
            cmd_no_payload("play", AUDIOCMD_PLAY);
            break;
        case 'a':
        case 'A':
            cmd_no_payload("pause", AUDIOCMD_PAUSE);
            break;
        case 'r':
        case 'R':
            cmd_no_payload("resume", AUDIOCMD_RESUME);
            break;
        case 'x':
        case 'X':
            cmd_no_payload("stop", AUDIOCMD_STOP);
            break;
        case 'v':
        case 'V':
            cmd_volume();
            break;
        case 'm':
        case 'M':
            cmd_metadata();
            break;
        case 'k':
        case 'K':
            cmd_seek();
            break;
        case 'q':
        case 'Q':
            return 0;
        default:
            cputs("unknown command\r\n");
            break;
        }

        wait_key();
    }
}
