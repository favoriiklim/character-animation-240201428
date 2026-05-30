#include "arkheon/character/ICharacterController.h"
#include <cstring>
#include <cmath>
#include <new>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace {

arkheon_quat q_identity() {
    return {0.0f, 0.0f, 0.0f, 1.0f};
}

arkheon_quat q_from_axis_angle(float ax, float ay, float az, float angle_rad) {
    float half = angle_rad * 0.5f;
    float s = std::sin(half);
    float c = std::cos(half);
    return {ax * s, ay * s, az * s, c};
}

arkheon_quat q_normalize(arkheon_quat q) {
    float n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (n2 <= 0.000001f) return q_identity();
    float inv = 1.0f / std::sqrt(n2);
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

struct Controller {
    float seg_len[10] = {0};
    int phase = 1;
    float phaseTime = 0.0f;
    arkheon_quat current_pose[ARK_JOINT_COUNT];
    bool initialized = false;
};

void apply_pose(arkheon_quat target[ARK_JOINT_COUNT], 
                float thigh_l, float thigh_r, 
                float calf_l, float calf_r, 
                float foot_l, float foot_r, 
                float upper_l, float upper_r, 
                float lower_l, float lower_r,
                float upper_l_z = 0.0f, float upper_r_z = 0.0f) 
{
    float deg2rad = M_PI / 180.0f;
    target[ARK_JOINT_THIGH_L] = q_from_axis_angle(1, 0, 0, thigh_l * deg2rad);
    target[ARK_JOINT_THIGH_R] = q_from_axis_angle(1, 0, 0, thigh_r * deg2rad);
    target[ARK_JOINT_CALF_L]  = q_from_axis_angle(1, 0, 0, calf_l * deg2rad);
    target[ARK_JOINT_CALF_R]  = q_from_axis_angle(1, 0, 0, calf_r * deg2rad);
    target[ARK_JOINT_FOOT_L]  = q_from_axis_angle(1, 0, 0, foot_l * deg2rad);
    target[ARK_JOINT_FOOT_R]  = q_from_axis_angle(1, 0, 0, foot_r * deg2rad);
    
    if (upper_l_z != 0.0f || upper_r_z != 0.0f) {
        arkheon_quat qx_l = q_from_axis_angle(1, 0, 0, upper_l * deg2rad);
        arkheon_quat qz_l = q_from_axis_angle(0, 0, 1, upper_l_z * deg2rad);
        target[ARK_JOINT_UPPERARM_L] = {qx_l.x, qx_l.y, qx_l.z + qz_l.z, qx_l.w};
        
        arkheon_quat qx_r = q_from_axis_angle(1, 0, 0, upper_r * deg2rad);
        arkheon_quat qz_r = q_from_axis_angle(0, 0, 1, upper_r_z * deg2rad);
        target[ARK_JOINT_UPPERARM_R] = {
            qx_r.w * qz_r.x + qx_r.x * qz_r.w + qx_r.y * qz_r.z - qx_r.z * qz_r.y,
            qx_r.w * qz_r.y - qx_r.x * qz_r.z + qx_r.y * qz_r.w + qx_r.z * qz_r.x,
            qx_r.w * qz_r.z + qx_r.x * qz_r.y - qx_r.y * qz_r.x + qx_r.z * qz_r.w,
            qx_r.w * qz_r.w - qx_r.x * qz_r.x - qx_r.y * qz_r.y - qx_r.z * qz_r.z
        };
    } else {
        target[ARK_JOINT_UPPERARM_L] = q_from_axis_angle(1, 0, 0, upper_l * deg2rad);
        target[ARK_JOINT_UPPERARM_R] = q_from_axis_angle(1, 0, 0, upper_r * deg2rad);
    }
    
    target[ARK_JOINT_LOWERARM_L] = q_from_axis_angle(1, 0, 0, lower_l * deg2rad);
    target[ARK_JOINT_LOWERARM_R] = q_from_axis_angle(1, 0, 0, lower_r * deg2rad);
}

}

extern "C" {

ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void) {
    return ARKHEON_CHARACTER_SDK_VERSION;
}

ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void) {
    return "Procedural Animation Controller";
}

ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(void*, int32_t out[3]) {
    out[0] = 12;
    out[1] = 47;
    out[2] = 83;
}

ARKHEON_CHAR_EXPORT void* arkheon_character_create(const float segs[10]) {
    auto* c = new (std::nothrow) Controller();
    if (!c) return nullptr;
    if (segs) {
        std::memcpy(c->seg_len, segs, sizeof(c->seg_len));
    }
    for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
        c->current_pose[i] = q_identity();
    }
    return c;
}

ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* h) {
    delete static_cast<Controller*>(h);
}

ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
    void* h,
    const arkheon_frame* frame,
    const arkheon_bone_state[66],
    arkheon_bone_override out_overrides[10],
    arkheon_vec3* out_root_translation_delta,
    arkheon_quat* out_root_rotation_delta,
    const arkheon_input_state*,
    const arkheon_mission_goal*,
    const arkheon_env_api*)
{
    if (!h || !out_overrides || !out_root_translation_delta || !out_root_rotation_delta) return 1;
    if (frame && frame->is_paused) return 0;

    auto* c = static_cast<Controller*>(h);
    float dt = (frame && frame->delta_time_s > 0) ? (float)frame->delta_time_s : 0.02f;

    *out_root_translation_delta = {0, 0, 0};
    *out_root_rotation_delta    = {0, 0, 0, 1};

    c->phaseTime += dt;
    float phaseDurations[] = { 0.0f, 6.0f, 4.0f, 3.0f, 4.0f };
    if (c->phaseTime >= phaseDurations[c->phase]) {
        c->phaseTime = 0.0f;
        c->phase++;
        if (c->phase > 4) c->phase = 1;
    }

    arkheon_quat target[ARK_JOINT_COUNT];
    for (int i = 0; i < ARK_JOINT_COUNT; ++i) target[i] = q_identity();

    float t = c->phaseTime;

    if (c->phase == 1) {
        float spd = 5.5f;
        float lc = std::sin(t * spd);
        float rc = std::sin(t * spd + M_PI);
        float lk = (lc < 0) ? -38.0f * std::abs(lc) : -5.0f;
        float rk = (rc < 0) ? -38.0f * std::abs(rc) : -5.0f;
        float la = (lc < 0) ? 12.0f : 0.0f;
        float ra = (rc < 0) ? 12.0f : 0.0f;
        apply_pose(target, lc*28, rc*28, lk, rk, la, ra, rc*22, lc*22, 18, 18);
    }
    else if (c->phase == 2) {
        float b = std::fmin(t * 2.0f, 1.0f);
        apply_pose(target, b*55, b*55, b*-90, b*-90, b*20, b*20, b*-15, b*-15, b*35, b*35);
    }
    else if (c->phase == 3) {
        float phase_mod = std::fmod(t * 2.5f, 3.0f);
        if (phase_mod < 1.0f) {
            float b = phase_mod;
            apply_pose(target, b*40, b*40, b*-70, b*-70, b*15, b*15, b*-20, b*-20, 10, 10);
        } else if (phase_mod < 2.0f) {
            float e = phase_mod - 1.0f;
            apply_pose(target, 40-50*e, 40-50*e, -70+65*e, -70+65*e, 15-30*e, 15-30*e, -20+80*e, -20+80*e, 5, 5);
        } else {
            float r = 1.0f - (phase_mod - 2.0f);
            apply_pose(target, -10*r, -10*r, -20*r, -20*r, 0, 0, 60*r, 60*r, 10, 10);
        }
    }
    else if (c->phase == 4) {
        float w = std::sin(t * 4.0f);
        apply_pose(target, 0, 0, -3, -3, 0, 0, 0, 120, 8, 40 + w*25, 0.0f, -30.0f);
    }

    float alpha = 0.25f;
    for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
        if (!c->initialized) {
            c->current_pose[i] = target[i];
        } else {
            c->current_pose[i].x += (target[i].x - c->current_pose[i].x) * alpha;
            c->current_pose[i].y += (target[i].y - c->current_pose[i].y) * alpha;
            c->current_pose[i].z += (target[i].z - c->current_pose[i].z) * alpha;
            c->current_pose[i].w += (target[i].w - c->current_pose[i].w) * alpha;
            c->current_pose[i] = q_normalize(c->current_pose[i]);
        }
        
        out_overrides[i].local_rotation = c->current_pose[i];
        out_overrides[i].apply = 1;
    }
    c->initialized = true;

    return 0;
}

}
