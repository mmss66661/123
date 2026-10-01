//
// arm_math.cpp — 4x4 齐次矩阵 / 旋转矩阵 / 位姿换算实现
//

#include "../Inc/arm_math.h"

#include <cmath>

namespace arm {

Mat4 mat4Identity() {
    Mat4 r{};
    r.m[0][0] = 1.0f;
    r.m[1][1] = 1.0f;
    r.m[2][2] = 1.0f;
    r.m[3][3] = 1.0f;
    return r;
}

Mat4 mat4Multiply(const Mat4& a, const Mat4& b) {
    Mat4 c{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a.m[i][k] * b.m[k][j];
            }
            c.m[i][j] = sum;
        }
    }
    return c;
}

void mat4ToPose(const Mat4& T, Pose& pose) {
    // R = Rz(yaw)*Ry(pitch)*Rx(roll) 的元素对应关系（见 arm_math.h 注释）
    const float sp = -T.m[2][0];
    const float pitch = asinf(sp < -1.0f ? -1.0f : (sp > 1.0f ? 1.0f : sp));
    const float cp = cosf(pitch);

    pose.pitch = pitch;
    pose.roll = atan2f(T.m[2][1], T.m[2][2]);
    pose.yaw = atan2f(T.m[1][0], T.m[0][0]);
    pose.x = T.m[0][3];
    pose.y = T.m[1][3];
    pose.z = T.m[2][3];

    // 万向锁附近 yaw/roll 退化，仅影响姿态表达不影响矩阵本身
    (void)cp;
}

Mat4 poseToMat4(const Pose& pose) {
    const float cr = cosf(pose.roll), sr = sinf(pose.roll);
    const float cp = cosf(pose.pitch), sp = sinf(pose.pitch);
    const float cy = cosf(pose.yaw), sy = sinf(pose.yaw);

    Mat4 T = mat4Identity();
    T.m[0][0] = cy * cp;
    T.m[0][1] = -sy * cr + cy * sp * sr;
    T.m[0][2] = sy * sr + cy * sp * cr;
    T.m[1][0] = sy * cp;
    T.m[1][1] = cy * cr + sy * sp * sr;
    T.m[1][2] = -cy * sr + sy * sp * cr;
    T.m[2][0] = -sp;
    T.m[2][1] = cp * sr;
    T.m[2][2] = cp * cr;
    T.m[0][3] = pose.x;
    T.m[1][3] = pose.y;
    T.m[2][3] = pose.z;
    return T;
}

Mat3 mat3Transpose(const Mat3& r) {
    Mat3 t{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            t.m[i][j] = r.m[j][i];
        }
    }
    return t;
}

Mat3 mat3Multiply(const Mat3& a, const Mat3& b) {
    Mat3 c{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += a.m[i][k] * b.m[k][j];
            }
            c.m[i][j] = sum;
        }
    }
    return c;
}

}  // namespace arm
