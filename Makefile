CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -Iinclude -D_POSIX_C_SOURCE=199309L
LDFLAGS ?= -lX11 -l:libGL.so.1 -lm -lpthread

SRC = src/zero_memory.c src/zero_platform.c src/zero_gl_loader.c src/zero_shader.c src/zero_mesh.c src/zero_camera.c src/zero_image.c src/zero_obj.c src/zero_zmesh.c
OBJ = $(SRC:src/%.c=build/%.o)
TEST_SRC = tests/test_zero_engine.c
TEST_BIN = build/test_zero_engine
LIB = build/libzero_engine.a
DEMO_BIN = build/demo_3d_viewport

.PHONY: all clean test demo

all: $(LIB) $(TEST_BIN) $(DEMO_BIN)

build:
	mkdir -p build

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB): $(OBJ) | build
	ar rcs $@ $(OBJ)

$(TEST_BIN): $(TEST_SRC) $(LIB) | build
	$(CC) $(CFLAGS) $(TEST_SRC) $(LIB) $(LDFLAGS) -o $@

$(DEMO_BIN): examples/demo_3d_viewport.c $(LIB) | build
	$(CC) $(CFLAGS) examples/demo_3d_viewport.c $(LIB) $(LDFLAGS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

demo: $(DEMO_BIN)
	./$(DEMO_BIN)

clean:
	rm -rf build
