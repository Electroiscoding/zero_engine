#ifndef ZERO_GL_DEFS_H
#define ZERO_GL_DEFS_H

#include <stddef.h>
#include <stdint.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

typedef unsigned int GLenum;
typedef unsigned char GLboolean;
typedef unsigned int GLbitfield;
typedef void GLvoid;
typedef signed char GLbyte;
typedef short GLshort;
typedef int GLint;
typedef unsigned char GLubyte;
typedef unsigned short GLushort;
typedef unsigned int GLuint;
typedef int GLsizei;
typedef float GLfloat;
typedef float GLclampf;
typedef double GLdouble;
typedef double GLclampd;
typedef char GLchar;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

#define GL_FALSE 0
#define GL_TRUE 1

#define GL_TRIANGLES 0x0004
#define GL_DEPTH_TEST 0x0B71
#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_DEPTH_BUFFER_BIT 0x00000100
#define GL_FLOAT 0x1406
#define GL_VERTEX_SHADER 0x8B31
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_ARRAY_BUFFER 0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW 0x88E4
#define GL_DYNAMIC_DRAW 0x88E8

typedef struct __GLXcontextRec *GLXContext;
typedef XID GLXPixmap;
typedef XID GLXDrawable;
typedef struct __GLXFBConfigRec *GLXFBConfig;
typedef void (*__GLXextFuncPtr)(void);

#define GLX_USE_GL 1
#define GLX_BUFFER_SIZE 2
#define GLX_LEVEL 3
#define GLX_RGBA 4
#define GLX_DOUBLEBUFFER 5
#define GLX_STEREO 6
#define GLX_AUX_BUFFERS 7
#define GLX_RED_SIZE 8
#define GLX_GREEN_SIZE 9
#define GLX_BLUE_SIZE 10
#define GLX_ALPHA_SIZE 11
#define GLX_DEPTH_SIZE 12
#define GLX_STENCIL_SIZE 13
#define GLX_ACCUM_RED_SIZE 14
#define GLX_ACCUM_GREEN_SIZE 15
#define GLX_ACCUM_BLUE_SIZE 16
#define GLX_ACCUM_ALPHA_SIZE 17

#define GLX_X_VISUAL_TYPE 0x22
#define GLX_TRUE_COLOR 0x8002
#define GLX_X_RENDERABLE 0x8012
#define GLX_DRAWABLE_TYPE 0x8013
#define GLX_RENDER_TYPE 0x8014
#define GLX_WINDOW_BIT 0x00000001
#define GLX_RGBA_BIT 0x00000001
#define GLX_RGBA_TYPE 0x8014

#define GLX_CONTEXT_MAJOR_VERSION_ARB 0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB 0x2092
#define GLX_CONTEXT_FLAGS_ARB 0x2094
#define GLX_CONTEXT_PROFILE_MASK_ARB 0x9126
#define GLX_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001

extern XVisualInfo *glXChooseVisual(Display *dpy, int screen, int *attribList);
extern GLXContext glXCreateContext(Display *dpy, XVisualInfo *vis, GLXContext shareList, Bool direct);
extern Bool glXMakeCurrent(Display *dpy, GLXDrawable drawable, GLXContext ctx);
extern void glXSwapBuffers(Display *dpy, GLXDrawable drawable);
extern void glXDestroyContext(Display *dpy, GLXContext ctx);
extern __GLXextFuncPtr glXGetProcAddressARB(const GLubyte *procName);
extern GLXFBConfig *glXChooseFBConfig(Display *dpy, int screen, const int *attribList, int *nitems);
extern XVisualInfo *glXGetVisualFromFBConfig(Display *dpy, GLXFBConfig config);
extern GLXContext glXCreateNewContext(Display *dpy, GLXFBConfig config, int renderType, GLXContext shareList, Bool direct);

extern void glClear(GLbitfield mask);
extern void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
extern void glEnable(GLenum cap);
extern void glViewport(GLint x, GLint y, GLsizei width, GLsizei height);
extern void glDrawArrays(GLenum mode, GLint first, GLsizei count);
extern void glDrawElements(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices);

#endif
