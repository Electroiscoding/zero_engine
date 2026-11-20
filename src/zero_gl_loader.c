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

    if (!zgl.glCreateShader || !zgl.glCreateProgram || !zgl.glGenVertexArrays || !zgl.glGenBuffers) {
        return false;
    }
    return true;
}
