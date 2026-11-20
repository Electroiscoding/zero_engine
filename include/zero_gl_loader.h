#ifndef ZERO_GL_LOADER_H
#define ZERO_GL_LOADER_H

#include "zero_core.h"
#include "zero_gl_defs.h"

typedef GLuint (*PFNGLCREATESHADERPROC)(GLenum type);
typedef void (*PFNGLSHADERSOURCEPROC)(GLuint shader, GLsizei count, const GLchar *const*string, const GLint *length);
typedef void (*PFNGLCOMPILESHADERPROC)(GLuint shader);
typedef void (*PFNGLGETSHADERIVPROC)(GLuint shader, GLenum pname, GLint *params);
typedef void (*PFNGLGETSHADERINFOLOGPROC)(GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
typedef void (*PFNGLDELETESHADERPROC)(GLuint shader);
typedef GLuint (*PFNGLCREATEPROGRAMPROC)(void);
typedef void (*PFNGLATTACHSHADERPROC)(GLuint program, GLuint shader);
typedef void (*PFNGLLINKPROGRAMPROC)(GLuint program);
typedef void (*PFNGLGETPROGRAMIVPROC)(GLuint program, GLenum pname, GLint *params);
typedef void (*PFNGLGETPROGRAMINFOLOGPROC)(GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
typedef void (*PFNGLUSEPROGRAMPROC)(GLuint program);
typedef void (*PFNGLDELETEPROGRAMPROC)(GLuint program);

typedef void (*PFNGLGENVERTEXARRAYSPROC)(GLsizei n, GLuint *arrays);
typedef void (*PFNGLBINDVERTEXARRAYPROC)(GLuint array);
typedef void (*PFNGLDELETEVERTEXARRAYSPROC)(GLsizei n, const GLuint *arrays);

typedef void (*PFNGLGENBUFFERSPROC)(GLsizei n, GLuint *buffers);
typedef void (*PFNGLBINDBUFFERPROC)(GLenum target, GLuint buffer);
typedef void (*PFNGLBUFFERDATAPROC)(GLenum target, GLsizeiptr size, const void *data, GLenum usage);
typedef void (*PFNGLDELETEBUFFERSPROC)(GLsizei n, const GLuint *buffers);

typedef void (*PFNGLENABLEVERTEXATTRIBARRAYPROC)(GLuint index);
typedef void (*PFNGLVERTEXATTRIBPOINTERPROC)(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer);

typedef GLint (*PFNGLGETUNIFORMLOCATIONPROC)(GLuint program, const GLchar *name);
typedef void (*PFNGLUNIFORMMATRIX4FVPROC)(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
typedef void (*PFNGLUNIFORM3FPROC)(GLint location, GLfloat v0, GLfloat v1, GLfloat v2);
typedef void (*PFNGLUNIFORM1FPROC)(GLint location, GLfloat v0);

typedef struct ZeroGL {
    PFNGLCREATESHADERPROC glCreateShader;
    PFNGLSHADERSOURCEPROC glShaderSource;
    PFNGLCOMPILESHADERPROC glCompileShader;
    PFNGLGETSHADERIVPROC glGetShaderiv;
    PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog;
    PFNGLDELETESHADERPROC glDeleteShader;
    PFNGLCREATEPROGRAMPROC glCreateProgram;
    PFNGLATTACHSHADERPROC glAttachShader;
    PFNGLLINKPROGRAMPROC glLinkProgram;
    PFNGLGETPROGRAMIVPROC glGetProgramiv;
    PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog;
    PFNGLUSEPROGRAMPROC glUseProgram;
    PFNGLDELETEPROGRAMPROC glDeleteProgram;

    PFNGLGENVERTEXARRAYSPROC glGenVertexArrays;
    PFNGLBINDVERTEXARRAYPROC glBindVertexArray;
    PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArrays;

    PFNGLGENBUFFERSPROC glGenBuffers;
    PFNGLBINDBUFFERPROC glBindBuffer;
    PFNGLBUFFERDATAPROC glBufferData;
    PFNGLDELETEBUFFERSPROC glDeleteBuffers;

    PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray;
    PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer;

    PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation;
    PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv;
    PFNGLUNIFORM3FPROC glUniform3f;
    PFNGLUNIFORM1FPROC glUniform1f;
} ZeroGL;

extern ZeroGL zgl;

bool zero_gl_load_core_profile(void);

#endif
