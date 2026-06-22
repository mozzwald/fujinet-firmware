#ifndef AUDIODEBUG_H
#define AUDIODEBUG_H

#ifdef AUDIO_DEBUG
#include "debug.h"
#define AudioDebug_printf(...) Debug_printf(__VA_ARGS__)
#else
#define AudioDebug_printf(...)
#endif

#endif // AUDIODEBUG_H
