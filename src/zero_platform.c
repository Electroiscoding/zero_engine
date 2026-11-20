#define _POSIX_C_SOURCE 199309L
#include "zero_platform.h"
#include "zero_gl_loader.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef GLXContext (*PFNGLXCREATECONTEXTATTRIBSARBPROC)(Display *dpy, GLXFBConfig config, GLXContext share_context, Bool direct, const int *attrib_list);

bool zero_platform_init(ZeroPlatformState *platform, const ZeroPlatformConfig *config) {
    if (!platform || !config) {
        return false;
    }

    memset(platform, 0, sizeof(*platform));
    platform->width = config->width;
    platform->height = config->height;
    platform->headless = config->headless;

    if (platform->headless || getenv("DISPLAY") == NULL) {
        platform->headless = true;
        return true;
    }

    platform->display = XOpenDisplay(NULL);
    if (!platform->display) {
        platform->headless = true;
        return true;
    }

    int screen = DefaultScreen(platform->display);

    static int visual_attribs[] = {
        GLX_X_RENDERABLE, True,
        GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
        GLX_RENDER_TYPE, GLX_RGBA_BIT,
        GLX_X_VISUAL_TYPE, GLX_TRUE_COLOR,
        GLX_RED_SIZE, 8,
        GLX_GREEN_SIZE, 8,
        GLX_BLUE_SIZE, 8,
        GLX_ALPHA_SIZE, 8,
        GLX_DEPTH_SIZE, 24,
        GLX_STENCIL_SIZE, 8,
        GLX_DOUBLEBUFFER, True,
        None
    };

    int fbcount = 0;
    GLXFBConfig *fbc = glXChooseFBConfig(platform->display, screen, visual_attribs, &fbcount);
    if (!fbc || fbcount == 0) {
        XCloseDisplay(platform->display);
        platform->display = NULL;
        platform->headless = true;
        return true;
    }

    GLXFBConfig best_fbc = fbc[0];
    XFree(fbc);

    XVisualInfo *vi = glXGetVisualFromFBConfig(platform->display, best_fbc);
    if (!vi) {
        XCloseDisplay(platform->display);
        platform->display = NULL;
        platform->headless = true;
        return true;
    }

    Window root = RootWindow(platform->display, vi->screen);
    XSetWindowAttributes swa;
    swa.colormap = XCreateColormap(platform->display, root, vi->visual, AllocNone);
    swa.background_pixmap = None;
    swa.border_pixel = 0;
    swa.event_mask = StructureNotifyMask | KeyPressMask | KeyReleaseMask |
                     ButtonPressMask | ButtonReleaseMask | PointerMotionMask;

    platform->window = XCreateWindow(
        platform->display, root,
        0, 0, platform->width, platform->height,
        0, vi->depth, InputOutput, vi->visual,
        CWBorderPixel | CWColormap | CWEventMask, &swa
    );

    XStoreName(platform->display, platform->window, config->title ? config->title : "Zero Engine");
    XMapWindow(platform->display, platform->window);

    platform->wm_delete_window = XInternAtom(platform->display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(platform->display, platform->window, &platform->wm_delete_window, 1);

    PFNGLXCREATECONTEXTATTRIBSARBPROC glXCreateContextAttribsARB = 
        (PFNGLXCREATECONTEXTATTRIBSARBPROC)glXGetProcAddressARB((const GLubyte *)"glXCreateContextAttribsARB");

    if (glXCreateContextAttribsARB) {
        int context_attribs[] = {
            GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
            GLX_CONTEXT_MINOR_VERSION_ARB, 3,
            GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
            None
        };
        platform->gl_context = glXCreateContextAttribsARB(platform->display, best_fbc, 0, True, context_attribs);
    }

    if (!platform->gl_context) {
        platform->gl_context = glXCreateNewContext(platform->display, best_fbc, GLX_RGBA_TYPE, 0, True);
    }

    XFree(vi);

    if (!platform->gl_context) {
        XDestroyWindow(platform->display, platform->window);
        XCloseDisplay(platform->display);
        platform->display = NULL;
        platform->headless = true;
        return true;
    }

    glXMakeCurrent(platform->display, platform->window, platform->gl_context);
    zero_gl_load_core_profile();

    return true;
}

void zero_platform_poll_events(ZeroPlatformState *platform) {
    if (!platform || platform->headless || !platform->display) {
        return;
    }

    while (XPending(platform->display) > 0) {
        XEvent ev;
        XNextEvent(platform->display, &ev);

        switch (ev.type) {
            case ClientMessage:
                if ((Atom)ev.xclient.data.l[0] == platform->wm_delete_window) {
                    platform->should_close = true;
                }
                break;
            case DestroyNotify:
                platform->should_close = true;
                break;
            case ConfigureNotify:
                platform->width = (uint32_t)ev.xconfigure.width;
                platform->height = (uint32_t)ev.xconfigure.height;
                break;
            case KeyPress: {
                KeySym ksym = XLookupKeysym(&ev.xkey, 0);
                if (ksym < 256) {
                    platform->keys[ksym] = true;
                }
                if (ksym == XK_Escape) {
                    platform->should_close = true;
                }
                break;
            }
            case KeyRelease: {
                KeySym ksym = XLookupKeysym(&ev.xkey, 0);
                if (ksym < 256) {
                    platform->keys[ksym] = false;
                }
                break;
            }
            case MotionNotify:
                platform->mouse_x = ev.xmotion.x;
                platform->mouse_y = ev.xmotion.y;
                break;
            case ButtonPress:
                if (ev.xbutton.button < 8) {
                    platform->mouse_buttons[ev.xbutton.button] = true;
                }
                break;
            case ButtonRelease:
                if (ev.xbutton.button < 8) {
                    platform->mouse_buttons[ev.xbutton.button] = false;
                }
                break;
            default:
                break;
        }
    }
}

void zero_platform_swap_buffers(ZeroPlatformState *platform) {
    if (platform && !platform->headless && platform->display && platform->window) {
        glXSwapBuffers(platform->display, platform->window);
    }
}

void zero_platform_shutdown(ZeroPlatformState *platform) {
    if (!platform) {
        return;
    }

    if (!platform->headless && platform->display) {
        glXMakeCurrent(platform->display, None, NULL);
        if (platform->gl_context) {
            glXDestroyContext(platform->display, platform->gl_context);
            platform->gl_context = NULL;
        }
        if (platform->window) {
            XDestroyWindow(platform->display, platform->window);
            platform->window = 0;
        }
        XCloseDisplay(platform->display);
        platform->display = NULL;
    }
    platform->headless = true;
}

double zero_platform_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}
