#include "arkheon/character/ICharacterController.h"
#include "AnimationSampler.h"
#include "PIPDController.h"
#include "PhysicsEngine.h"
#include <new>
#include <cstring>

struct Controller {
    AnimationSampler sampler;
    PIPDController pipd;
    PhysicsEngine physics;
    arkheon_quat current_pose[10];
    arkheon_quat previous_pose[10];
    bool initialized = false;

    arkheon_quat q_identity() {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }
};

extern "C" {

ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void) {
    return ARKHEON_CHARACTER_SDK_VERSION;
}

ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void) {
    return "Procedural Physics Controller";
}

ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(void*, int32_t out[3]) {
    out[0] = 12;
    out[1] = 47;
    out[2] = 83;
}

ARKHEON_CHAR_EXPORT void* arkheon_character_create(const float segs[10]) {
    auto* c = new (std::nothrow) Controller();
    if (!c) return nullptr;
    
    c->physics.initialize(segs);
    c->pipd.initialize();
    
    for (int i = 0; i < 10; ++i) {
        c->current_pose[i] = c->q_identity();
        c->previous_pose[i] = c->q_identity();
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

    arkheon_quat target_quats[10];
    c->sampler.update(dt, target_quats);

    if (!c->initialized) {
        for (int i = 0; i < 10; i++) {
            c->current_pose[i] = target_quats[i];
            c->previous_pose[i] = target_quats[i];
        }
        c->initialized = true;
    }

    arkheon_vec3 torques[10];
    c->pipd.compute_torques(c->current_pose, target_quats, c->previous_pose, dt, torques);

    for (int i = 0; i < 10; i++) {
        c->previous_pose[i] = c->current_pose[i];
    }

    c->physics.step_physics(c->current_pose, torques, dt, out_overrides);

    for (int i = 0; i < 10; i++) {
        c->current_pose[i] = out_overrides[i].local_rotation;
    }

    return 0;
}

}
