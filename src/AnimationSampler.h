#pragma once
#include <arkheon/character/ICharacterController.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

struct AnimationSampler {
    int phase = 1;
    float phaseTime = 0.0f;

    arkheon_quat q_identity() {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }

    arkheon_quat q_from_axis_angle(float ax, float ay, float az, float angle_rad) {
        float half = angle_rad * 0.5f;
        float s = std::sin(half);
        float c = std::cos(half);
        return {ax * s, ay * s, az * s, c};
    }

    void apply_pose(arkheon_quat target[10], float tl, float tr, float cl, float cr, float fl, float fr, float ul, float ur, float ll, float lr, float ul_z = 0.0f, float ur_z = 0.0f) {
        float deg2rad = M_PI / 180.0f;
        target[ARK_JOINT_THIGH_L] = q_from_axis_angle(1, 0, 0, tl * deg2rad);
        target[ARK_JOINT_THIGH_R] = q_from_axis_angle(1, 0, 0, tr * deg2rad);
        target[ARK_JOINT_CALF_L]  = q_from_axis_angle(1, 0, 0, cl * deg2rad);
        target[ARK_JOINT_CALF_R]  = q_from_axis_angle(1, 0, 0, cr * deg2rad);
        target[ARK_JOINT_FOOT_L]  = q_from_axis_angle(1, 0, 0, fl * deg2rad);
        target[ARK_JOINT_FOOT_R]  = q_from_axis_angle(1, 0, 0, fr * deg2rad);
        
        if (ul_z != 0.0f || ur_z != 0.0f) {
            arkheon_quat qx_l = q_from_axis_angle(1, 0, 0, ul * deg2rad);
            arkheon_quat qz_l = q_from_axis_angle(0, 0, 1, ul_z * deg2rad);
            target[ARK_JOINT_UPPERARM_L] = {qx_l.x, qx_l.y, qx_l.z + qz_l.z, qx_l.w};
            arkheon_quat qx_r = q_from_axis_angle(1, 0, 0, ur * deg2rad);
            arkheon_quat qz_r = q_from_axis_angle(0, 0, 1, ur_z * deg2rad);
            target[ARK_JOINT_UPPERARM_R] = {
                qx_r.w * qz_r.x + qx_r.x * qz_r.w + qx_r.y * qz_r.z - qx_r.z * qz_r.y,
                qx_r.w * qz_r.y - qx_r.x * qz_r.z + qx_r.y * qz_r.w + qx_r.z * qz_r.x,
                qx_r.w * qz_r.z + qx_r.x * qz_r.y - qx_r.y * qz_r.x + qx_r.z * qz_r.w,
                qx_r.w * qz_r.w - qx_r.x * qz_r.x - qx_r.y * qz_r.y - qx_r.z * qz_r.z
            };
        } else {
            target[ARK_JOINT_UPPERARM_L] = q_from_axis_angle(1, 0, 0, ul * deg2rad);
            target[ARK_JOINT_UPPERARM_R] = q_from_axis_angle(1, 0, 0, ur * deg2rad);
        }
        target[ARK_JOINT_LOWERARM_L] = q_from_axis_angle(1, 0, 0, ll * deg2rad);
        target[ARK_JOINT_LOWERARM_R] = q_from_axis_angle(1, 0, 0, lr * deg2rad);
    }

    void update(float dt, arkheon_quat target_quats[10]) {
        phaseTime += dt;
        float phaseDurations[] = { 0.0f, 6.0f, 4.0f, 3.0f, 4.0f };
        if (phaseTime >= phaseDurations[phase]) {
            phaseTime = 0.0f;
            phase++;
            if (phase > 4) phase = 1;
        }

        for (int i = 0; i < 10; ++i) target_quats[i] = q_identity();
        float t = phaseTime;

        if (phase == 1) {
            float spd = 5.5f;
            float lc = std::sin(t * spd);
            float rc = std::sin(t * spd + M_PI);
            float lk = (lc < 0) ? -38.0f * std::abs(lc) : -5.0f;
            float rk = (rc < 0) ? -38.0f * std::abs(rc) : -5.0f;
            float la = (lc < 0) ? 12.0f : 0.0f;
            float ra = (rc < 0) ? 12.0f : 0.0f;
            apply_pose(target_quats, lc*28, rc*28, lk, rk, la, ra, rc*22, lc*22, 18, 18);
        }
        else if (phase == 2) {
            float b = std::fmin(t * 2.0f, 1.0f);
            apply_pose(target_quats, b*55, b*55, b*-90, b*-90, b*20, b*20, b*-15, b*-15, b*35, b*35);
        }
        else if (phase == 3) {
            float phase_mod = std::fmod(t * 2.5f, 3.0f);
            if (phase_mod < 1.0f) {
                apply_pose(target_quats, phase_mod*40, phase_mod*40, phase_mod*-70, phase_mod*-70, phase_mod*15, phase_mod*15, phase_mod*-20, phase_mod*-20, 10, 10);
            } else if (phase_mod < 2.0f) {
                float e = phase_mod - 1.0f;
                apply_pose(target_quats, 40-50*e, 40-50*e, -70+65*e, -70+65*e, 15-30*e, 15-30*e, -20+80*e, -20+80*e, 5, 5);
            } else {
                float r = 1.0f - (phase_mod - 2.0f);
                apply_pose(target_quats, -10*r, -10*r, -20*r, -20*r, 0, 0, 60*r, 60*r, 10, 10);
            }
        }
        else if (phase == 4) {
            float w = std::sin(t * 4.0f);
            apply_pose(target_quats, 0, 0, -3, -3, 0, 0, 0, 120, 8, 40 + w*25, 0.0f, -30.0f);
        }
    }
};
