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
#define GL_BLEND 0x0BE2
#define GL_SRC_ALPHA 0x0302
#define GL_ONE_MINUS_SRC_ALPHA 0x0303
#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_DEPTH_BUFFER_BIT 0x00000100
#define GL_UNSIGNED_BYTE 0x1401
#define GL_UNSIGNED_INT 0x1405
#define GL_FLOAT 0x1406
#define GL_RED 0x1903
#define GL_RGB 0x1907
#define GL_RGBA 0x1908
#define GL_TEXTURE_2D 0x0DE1
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_TEXTURE_WRAP_S 0x2802
#define GL_TEXTURE_WRAP_T 0x2803
#define GL_NEAREST 0x2600
#define GL_LINEAR 0x2601
#define GL_REPEAT 0x2901
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_VERTEX_SHADER 0x8B31
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_ARRAY_BUFFER 0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW 0x88E4
#define GL_DYNAMIC_DRAW 0x88E8

/* Framebuffer Objects (FBO) */
#define GL_FRAMEBUFFER                  0x8D40
#define GL_READ_FRAMEBUFFER             0x8CA8
#define GL_DRAW_FRAMEBUFFER             0x8CA9
#define GL_DEPTH_ATTACHMENT             0x8D00
#define GL_STENCIL_ATTACHMENT           0x8D20
#define GL_DEPTH_STENCIL_ATTACHMENT     0x821A
#define GL_COLOR_ATTACHMENT0            0x8CE0
#define GL_COLOR_ATTACHMENT1            0x8CE1
#define GL_COLOR_ATTACHMENT2            0x8CE2
#define GL_COLOR_ATTACHMENT3            0x8CE3
#define GL_FRAMEBUFFER_COMPLETE         0x8CD5
#define GL_FRAMEBUFFER_UNDEFINED        0x8219
#define GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT 0x8CD6
#define GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT 0x8CD7
#define GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER 0x8CDB
#define GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER 0x8CDC
#define GL_FRAMEBUFFER_UNSUPPORTED      0x8CDD
#define GL_NONE                         0

/* Depth Formats */
#define GL_DEPTH_COMPONENT              0x1902
#define GL_DEPTH_COMPONENT16            0x81A5
#define GL_DEPTH_COMPONENT24            0x81A6
#define GL_DEPTH_COMPONENT32            0x81A7
#define GL_DEPTH_COMPONENT32F           0x8CAC

/* Texture Parameters for Shadow Comparison & Border Clamping */
#define GL_TEXTURE_COMPARE_MODE         0x884C
#define GL_COMPARE_REF_TO_TEXTURE       0x884E
#define GL_TEXTURE_COMPARE_FUNC         0x884D
#define GL_LEQUAL                       0x0203
#define GL_GEQUAL                       0x0206
#define GL_LESS                         0x0201
#define GL_GREATER                      0x0204
#define GL_EQUAL                        0x0202
#define GL_NOTEQUAL                     0x0205
#define GL_ALWAYS                       0x0207
#define GL_NEVER                        0x0200
#define GL_CLAMP_TO_BORDER              0x812D
#define GL_TEXTURE_BORDER_COLOR         0x1004

/* Active Texture Unit Enums */
#define GL_TEXTURE0                     0x84C0
#define GL_TEXTURE1                     0x84C1
#define GL_TEXTURE2                     0x84C2
#define GL_TEXTURE3                     0x84C3
#define GL_TEXTURE4                     0x84C4
#define GL_TEXTURE5                     0x84C5
#define GL_TEXTURE6                     0x84C6
#define GL_TEXTURE7                     0x84C7

/* Rasterization and Culling */
#define GL_CULL_FACE                    0x0B44
#define GL_FRONT                        0x0404
#define GL_BACK                         0x0405
#define GL_FRONT_AND_BACK               0x0408
#define GL_CW                           0x0900
#define GL_CCW                          0x0901
#define GL_POLYGON_OFFSET_FILL          0x8037

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
extern void glDisable(GLenum cap);
extern void glBlendFunc(GLenum sfactor, GLenum dfactor);
extern void glViewport(GLint x, GLint y, GLsizei width, GLsizei height);
extern void glDrawArrays(GLenum mode, GLint first, GLsizei count);
extern void glDrawElements(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices);

extern void glGenTextures(GLsizei n, GLuint *textures);
extern void glBindTexture(GLenum target, GLuint texture);
extern void glDeleteTextures(GLsizei n, const GLuint *textures);
extern void glTexParameteri(GLenum target, GLenum pname, GLint param);
extern void glTexParameterfv(GLenum target, GLenum pname, const GLfloat *params);
extern void glTexImage2D(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid *pixels);

extern void glDepthMask(GLboolean flag);
extern void glDepthFunc(GLenum func);
extern void glCullFace(GLenum mode);
extern void glPolygonOffset(GLfloat factor, GLfloat units);
extern void glDrawBuffer(GLenum buf);
extern void glReadBuffer(GLenum src);

#endif
