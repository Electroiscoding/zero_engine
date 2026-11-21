#include "zero_gl_loader.h"

ZeroGL zgl = {0};

static void *get_proc(const char *name) {
    void *ptr = (void *)glXGetProcAddressARB((const GLubyte *)name);
    return ptr;
}

bool zero_gl_load_core_profile(void) {
    zgl.glCreateShader = (PFNGLCREATESHADERPROC)get_proc("glCreateShader");
    zgl.glShaderSource = (PFNGLSHADERSOURCEPROC)get_proc("glShaderSource");
    zgl.glCompileShader = (PFNGLCOMPILESHADERPROC)get_proc("glCompileShader");
    zgl.glGetShaderiv = (PFNGLGETSHADERIVPROC)get_proc("glGetShaderiv");
    zgl.glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC)get_proc("glGetShaderInfoLog");
    zgl.glDeleteShader = (PFNGLDELETESHADERPROC)get_proc("glDeleteShader");

    zgl.glCreateProgram = (PFNGLCREATEPROGRAMPROC)get_proc("glCreateProgram");
    zgl.glAttachShader = (PFNGLATTACHSHADERPROC)get_proc("glAttachShader");
    zgl.glLinkProgram = (PFNGLLINKPROGRAMPROC)get_proc("glLinkProgram");
    zgl.glGetProgramiv = (PFNGLGETPROGRAMIVPROC)get_proc("glGetProgramiv");
    zgl.glGetProgramInfoLog = (PFNGLGETPROGRAMINFOLOGPROC)get_proc("glGetProgramInfoLog");
    zgl.glUseProgram = (PFNGLUSEPROGRAMPROC)get_proc("glUseProgram");
    zgl.glDeleteProgram = (PFNGLDELETEPROGRAMPROC)get_proc("glDeleteProgram");

    zgl.glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC)get_proc("glGenVertexArrays");
    zgl.glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)get_proc("glBindVertexArray");
    zgl.glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC)get_proc("glDeleteVertexArrays");

    zgl.glGenBuffers = (PFNGLGENBUFFERSPROC)get_proc("glGenBuffers");
    zgl.glBindBuffer = (PFNGLBINDBUFFERPROC)get_proc("glBindBuffer");
    zgl.glBufferData = (PFNGLBUFFERDATAPROC)get_proc("glBufferData");
    zgl.glDeleteBuffers = (PFNGLDELETEBUFFERSPROC)get_proc("glDeleteBuffers");

    zgl.glEnableVertexAttribArray = (PFNGLENABLEVERTEXATTRIBARRAYPROC)get_proc("glEnableVertexAttribArray");
    zgl.glVertexAttribPointer = (PFNGLVERTEXATTRIBPOINTERPROC)get_proc("glVertexAttribPointer");

    zgl.glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC)get_proc("glGetUniformLocation");
    zgl.glUniformMatrix4fv = (PFNGLUNIFORMMATRIX4FVPROC)get_proc("glUniformMatrix4fv");
    zgl.glUniform3f = (PFNGLUNIFORM3FPROC)get_proc("glUniform3f");
    zgl.glUniform1f = (PFNGLUNIFORM1FPROC)get_proc("glUniform1f");

    /* Framebuffers */
    zgl.glGenFramebuffers = (PFNGLGENFRAMEBUFFERSPROC)get_proc("glGenFramebuffers");
    zgl.glBindFramebuffer = (PFNGLBINDFRAMEBUFFERPROC)get_proc("glBindFramebuffer");
    zgl.glFramebufferTexture2D = (PFNGLFRAMEBUFFERTEXTURE2DPROC)get_proc("glFramebufferTexture2D");
    zgl.glCheckFramebufferStatus = (PFNGLCHECKFRAMEBUFFERSTATUSPROC)get_proc("glCheckFramebufferStatus");
    zgl.glDeleteFramebuffers = (PFNGLDELETEFRAMEBUFFERSPROC)get_proc("glDeleteFramebuffers");
    zgl.glDrawBuffers = (PFNGLDRAWBUFFERSPROC)get_proc("glDrawBuffers");
    zgl.glDrawBuffer = (PFNGLDRAWBUFFERPROC)get_proc("glDrawBuffer");
    zgl.glReadBuffer = (PFNGLREADBUFFERPROC)get_proc("glReadBuffer");

    /* Texture units & parameters */
    zgl.glActiveTexture = (PFNGLACTIVETEXTUREPROC)get_proc("glActiveTexture");
    zgl.glTexParameterfv = (PFNGLTEXPARAMETERFVPROC)get_proc("glTexParameterfv");

    /* Uniforms */
    zgl.glUniform1i = (PFNGLUNIFORM1IPROC)get_proc("glUniform1i");
    zgl.glUniform1iv = (PFNGLUNIFORM1IVPROC)get_proc("glUniform1iv");
    zgl.glUniform1fv = (PFNGLUNIFORM1FVPROC)get_proc("glUniform1fv");
    zgl.glUniform3fv = (PFNGLUNIFORM3FVPROC)get_proc("glUniform3fv");

    if (!zgl.glCreateShader || !zgl.glCreateProgram || !zgl.glGenVertexArrays || !zgl.glGenBuffers) {
        return false;
    }
    return true;
}
