// tests/standalone_test.cpp
// Standalone test harness — NO APP required.
// Validates physical compliance and structural bounds of the procedural animation.

#include "arkheon/character/ICharacterController.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <chrono>

static int32_t mock_raycast(void*, arkheon_vec3, arkheon_vec3, float, arkheon_vec3* out_hit, arkheon_vec3* out_normal, int32_t* out_obj) {
    *out_hit = {0, 0, 0}; *out_normal = {0, 1, 0}; *out_obj = -1; return 0;
}
static int32_t mock_get_aabb(void*, int32_t, arkheon_vec3* out_min, arkheon_vec3* out_max) {
    *out_min = {-1, -1, -1}; *out_max = {1, 1, 1}; return 1;
}
static int32_t mock_find_obj(void*, const char*) { return 0; }
static int32_t mock_navmesh(void*, arkheon_vec3, arkheon_vec3 to, arkheon_vec3* out_path, int32_t) {
    out_path[0] = to; return 1;
}
static void mock_report_goal(void*, int32_t, int32_t) {}
static arkheon_vec3 mock_gravity(void*) { return {0, -9.81f, 0}; }

static void fill_identity(arkheon_bone_state bones[66]) {
    for (int i = 0; i < 66; i++) {
        bones[i].local_rotation = {0, 0, 0, 1};
        bones[i].local_translation = {0, 0, 0};
        bones[i].world_position = {0, 1.0f, 0};
    }
}

static bool is_valid_quaternion(const arkheon_quat &q) {
    float n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(n2) && n2 > 0.5f && n2 < 1.5f;
}

int main() {
    std::printf("========================================\n");
    std::printf("Standalone Test - Procedural Physics Controller\n");
    std::printf("========================================\n\n");

    uint32_t version = arkheon_character_sdk_version();
    std::printf("[TEST 1] SDK version = 0x%08x\n", version);
    if (version != ARKHEON_CHARACTER_SDK_VERSION) return 1;

    const char* name = arkheon_character_plugin_name();
    std::printf("[TEST 2] Plugin name: %s\n", name);
    if (!name || strlen(name) == 0) return 1;

    int32_t clip_ids[3] = {0};
    arkheon_character_get_motion_clips(nullptr, clip_ids);
    std::printf("[TEST 3] Motion clips: A=%d, B=%d, C=%d\n", clip_ids[0], clip_ids[1], clip_ids[2]);

    float segs[10] = {0.0f, 0.0f, 0.3f, 0.3f, 0.0f, 0.0f, 0.4f, 0.4f, 0.4f, 0.4f};
    void* handle = arkheon_character_create(segs);
    if (!handle) return 1;
    std::printf("[TEST 4] Character created successfully\n");

    std::printf("[TEST 5] Running 1000 physics ticks (phase evaluation)...\n");
    arkheon_bone_state bones[66]; fill_identity(bones);
    arkheon_env_api env = {nullptr, mock_raycast, mock_get_aabb, mock_find_obj, mock_navmesh, mock_report_goal, mock_gravity};
    arkheon_frame frame = {}; frame.delta_time_s = 0.02; frame.is_paused = 0;

    auto start_time = std::chrono::high_resolution_clock::now();
    double max_tick_time = 0.0;

    for (uint64_t i = 0; i < 1000; ++i) {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number = i;
        auto tick_start = std::chrono::high_resolution_clock::now();
        arkheon_bone_override out_overrides[10] = {};
        arkheon_vec3 root_delta = {0,0,0};
        arkheon_quat root_rot_delta = {0,0,0,1};
        arkheon_input_state input = {};

        int rc = arkheon_character_tick(handle, &frame, bones, out_overrides, &root_delta, &root_rot_delta, &input, nullptr, &env);
        
        auto tick_end = std::chrono::high_resolution_clock::now();
        double tick_time = std::chrono::duration<double, std::milli>(tick_end - tick_start).count();
        if (tick_time > max_tick_time) max_tick_time = tick_time;

        if (rc != 0) return 1;
        for (int j = 0; j < 10; j++) {
            if (!is_valid_quaternion(out_overrides[j].local_rotation)) return 1;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std::printf("  Avg tick: %.4f ms (Max: %.4f ms)\n", total_time / 1000.0, max_tick_time);
    std::printf("PASS\n\n");

    std::printf("[TEST 6] Stress test: 10000 ticks...\n");
    fill_identity(bones);
    for (uint64_t i = 0; i < 10000; ++i) {
        frame.simulation_time_s = i * 0.02;
        arkheon_bone_override out_overrides[10] = {};
        arkheon_vec3 root_delta = {0,0,0}; arkheon_quat root_rot_delta = {0,0,0,1};
        arkheon_input_state input = {};
        if (arkheon_character_tick(handle, &frame, bones, out_overrides, &root_delta, &root_rot_delta, &input, nullptr, &env) != 0) return 1;
    }
    std::printf("PASS\n\n");

    arkheon_character_destroy(handle);
    std::printf("========================================\n");
    std::printf("ALL TESTS PASSED!\n");
    std::printf("========================================\n");
    return 0;
}
