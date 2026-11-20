#ifndef ZERO_PLATFORM_H
#define ZERO_PLATFORM_H

#include "zero_core.h"
#include "zero_gl_defs.h"

typedef struct ZeroPlatformConfig {
    const char *title;
    uint32_t width;
    uint32_t height;
    bool headless;
} ZeroPlatformConfig;

typedef struct ZeroPlatformState {
    Display *display;
    Window window;
    GLXContext gl_context;
    Atom wm_delete_window;
    
    bool keys[256];
    int mouse_x;
    int mouse_y;
    bool mouse_buttons[8];

    bool should_close;
    bool headless;
    uint32_t width;
    uint32_t height;
} ZeroPlatformState;

bool zero_platform_init(ZeroPlatformState *platform, const ZeroPlatformConfig *config);
void zero_platform_poll_events(ZeroPlatformState *platform);
void zero_platform_swap_buffers(ZeroPlatformState *platform);
void zero_platform_shutdown(ZeroPlatformState *platform);
double zero_platform_get_time(void);

#endif
