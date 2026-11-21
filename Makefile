CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -Iinclude -D_POSIX_C_SOURCE=199309L
LDFLAGS ?= -lX11 -l:libGL.so.1 -lm -lpthread

SRC = src/zero_memory.c src/zero_platform.c src/zero_gl_loader.c src/zero_shader.c src/zero_mesh.c src/zero_camera.c src/zero_image.c src/zero_obj.c src/zero_zmesh.c src/zero_vehicle.c src/zero_physics.c src/zero_csm.c src/zero_sky.c src/zero_scene.c src/zero_inspect.c src/zero_ui.c src/zero_audio.c src/zero_race_camera.c src/zero_race_ui.c src/zero_track_mesh.c src/zero_vehicle_physics.c src/zero_ai_racers.c src/zero_ai_nn.c
OBJ = $(SRC:src/%.c=build/%.o)
TEST_SRC = tests/test_zero_engine.c
TEST_BIN = build/test_zero_engine
TEST_RACE_SRC = tests/test_race_integration.c
TEST_RACE_BIN = build/test_race_integration
TEST_COLLISION_SRC = tests/test_race_collision.c
TEST_COLLISION_BIN = build/test_race_collision
TEST_AI_NN_SRC = tests/test_ai_nn.c
TEST_AI_NN_BIN = build/test_ai_nn
TRAIN_AI_SRC = tools/train_ai_racers.c
TRAIN_AI_BIN = build/train_ai_racers
DEMO_BIN = build/demo_3d_viewport
CAR_BIN = build/demo_car_game
STUDIO_BIN = build/demo_engine_editor
RACE_BIN = build/demo_race_game
LIB = build/libzero_engine.a

.PHONY: all clean test demo car_game studio race_game test_race test_collision test_ai_nn train_ai

all: $(LIB) $(TEST_BIN) $(DEMO_BIN) $(CAR_BIN) $(STUDIO_BIN)

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

$(CAR_BIN): examples/demo_car_game.c $(LIB) | build
	$(CC) $(CFLAGS) examples/demo_car_game.c $(LIB) $(LDFLAGS) -o $@

$(STUDIO_BIN): examples/demo_engine_editor.c $(LIB) | build
	$(CC) $(CFLAGS) examples/demo_engine_editor.c $(LIB) $(LDFLAGS) -o $@

$(RACE_BIN): examples/demo_race_game.c $(LIB) | build
	$(CC) $(CFLAGS) examples/demo_race_game.c $(LIB) $(LDFLAGS) -o $@

$(TEST_RACE_BIN): $(TEST_RACE_SRC) $(LIB) | build
	$(CC) $(CFLAGS) $(TEST_RACE_SRC) $(LIB) $(LDFLAGS) -o $@

$(TEST_COLLISION_BIN): $(TEST_COLLISION_SRC) $(LIB) | build
	$(CC) $(CFLAGS) $(TEST_COLLISION_SRC) $(LIB) $(LDFLAGS) -o $@

$(TEST_AI_NN_BIN): $(TEST_AI_NN_SRC) $(LIB) | build
	$(CC) $(CFLAGS) $(TEST_AI_NN_SRC) $(LIB) $(LDFLAGS) -o $@

$(TRAIN_AI_BIN): $(TRAIN_AI_SRC) $(LIB) | build
	$(CC) $(CFLAGS) $(TRAIN_AI_SRC) $(LIB) $(LDFLAGS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

test_race: $(TEST_RACE_BIN)
	./$(TEST_RACE_BIN)

test_collision: $(TEST_COLLISION_BIN)
	./$(TEST_COLLISION_BIN)

test_ai_nn: $(TEST_AI_NN_BIN)
	./$(TEST_AI_NN_BIN)

train_ai: $(TRAIN_AI_BIN)
	./$(TRAIN_AI_BIN)

demo: $(DEMO_BIN)
	./$(DEMO_BIN)

car_game: $(CAR_BIN)
	./$(CAR_BIN)

studio: $(STUDIO_BIN)
	./$(STUDIO_BIN)

race_game: $(RACE_BIN)
	./$(RACE_BIN)

clean:
	rm -rf build
