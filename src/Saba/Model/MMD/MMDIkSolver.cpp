//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#include "MMDIkSolver.h"
#include <algorithm>
#include <cmath>

extern void ik_one_joint_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joint_local_space, DirectX::XMFLOAT4X4 *const inout_joint_model_space);
extern void ik_two_joints_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_hinge_joint_axis_local_space, float const in_cosine_max_hinge_joint_angle, float const in_cosine_min_hinge_joint_angle, float const in_hinge_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space);
extern void ik_ccd_solve(uint32_t const in_iteration, float const in_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space);
extern void ik_reaching_solve(DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space);

namespace saba
{
    void MMDIkSolver::Solve(DirectX::XMFLOAT3 const &in_two_joints_hinge_joint_axis_local_space, float const in_two_joints_cosine_max_hinge_joint_angle, float const in_two_joints_cosine_min_hinge_joint_angle, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *inout_joints_local_space, DirectX::XMFLOAT4X4 *inout_joints_model_space)
    {
#if 0
        ik_reaching_solve(in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
#else
        if (1U == in_joint_count)
        {
            ik_one_joint_solve(1.0F, in_target_position_model_space, in_end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);
        }
        else if (2U == in_joint_count)
        {
            ik_two_joints_solve(1.0F, in_two_joints_hinge_joint_axis_local_space, in_two_joints_cosine_max_hinge_joint_angle, in_two_joints_cosine_min_hinge_joint_angle, 1.0F, in_target_position_model_space, in_end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);
        }
        else if (3U == in_joint_count)
        {
            // TODO: three joints IK
            ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
        }
        else
        {

            assert(in_joint_count >= 4U);
            ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
        }
#endif
    }
}

static inline float internal_sqrt(float x)
{
    constexpr float const epsilon = 1E-6F;
    return ((x > epsilon) ? std::sqrt(x) : 0.0F);
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_compute_rotation_axis_damped(DirectX::XMVECTOR axis, float cos_angle, float sin_angle, float gain)
{
    constexpr float const epsilon = 1E-6F;
    assert(std::abs(DirectX::XMVectorGetX(DirectX::XMVector3Dot(axis, axis)) - 1.0F) < epsilon);

    constexpr float const one = 1.0F;
    constexpr float const half = 0.5F;
    constexpr float const zero = 0.0F;
    constexpr float const nearly_one = one - epsilon;

    float const damped_dot = one - gain + gain * cos_angle;

    float const cos_angle_div_2_square = (damped_dot + one) * half;

    if (cos_angle_div_2_square > zero && cos_angle >= (-nearly_one) && cos_angle <= nearly_one)
    {
        // cos(angle/2) = sqrt((1+cos(angle))/2)
        float cos_angle_div_2 = std::sqrt(cos_angle_div_2_square);

        // sin(angle/2) = sin(angle)/(2*cos(angle/2))
        float sin_angle_div_2 = sin_angle * ((gain * half) / cos_angle_div_2);

        // "cos_angle >= (-nearly_one) && cos_angle <= nearly_one" to avoid zero vector which can NOT be normalized
        return DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(DirectX::XMVectorScale(axis, sin_angle_div_2), cos_angle_div_2));
    }
    else if (cos_angle_div_2_square > zero && cos_angle > nearly_one)
    {
        return DirectX::XMQuaternionIdentity();
    }
    else
    {
        return DirectX::XMVectorSetW(axis, 0.0F);
    }
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_calculate_perpendicular_vector(DirectX::XMVECTOR simd_in_v)
{
    int min = 0;
    int ok1 = 1;
    int ok2 = 2;

    float in_v[3];
    DirectX::XMStoreFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&in_v[0]), simd_in_v);

    float a0 = in_v[0];
    float a1 = in_v[1];
    float a2 = in_v[2];

    if (a1 < a0)
    {
        ok1 = 0;
        min = 1;
        a0 = a1;
    }

    if (a2 < a0)
    {
        ok2 = min;
        min = 2;
    }

    float out_v[3] = {0.0F, 0.0F, 0.0F};
    out_v[ok1] = in_v[ok2];
    out_v[ok2] = -in_v[ok1];
    return DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&out_v[0]));
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_compute_shortest_rotation_damped(DirectX::XMVECTOR from, DirectX::XMVECTOR to, float gain)
{
    constexpr float const epsilon = 1E-6F;
    assert(std::abs(DirectX::XMVectorGetX(DirectX::XMVector3Dot(from, from)) - 1.0F) < epsilon);
    assert(std::abs(DirectX::XMVectorGetX(DirectX::XMVector3Dot(to, to)) - 1.0F) < epsilon);

    constexpr float const one = 1.0F;
    constexpr float const half = 0.5F;
    constexpr float const zero = 0.0F;
    constexpr float const nearly_one = one - epsilon;

    // cos(theta)
    float const cos_theta = DirectX::XMVectorGetX(DirectX::XMVector3Dot(from, to));

    float const damped_dot = one - gain + gain * cos_theta;

    float const cos_theta_div_2_square = (damped_dot + one) * half;

    if (cos_theta_div_2_square > zero && cos_theta >= (-nearly_one) && cos_theta <= nearly_one)
    {
        // cos(theta/2) = sqrt((1+cos(theta))/2)
        float cos_theta_div_2 = std::sqrt(cos_theta_div_2_square);

        // sin(theta)
        DirectX::XMVECTOR cross = DirectX::XMVector3Cross(from, to);

        // sin(theta/2) = sin(theta)/(2*cos(theta/2))
        DirectX::XMVECTOR sin_theta_div_2 = DirectX::XMVectorScale(cross, ((gain * half) / cos_theta_div_2));

        // "cos_theta >= (-nearly_one) && cos_theta <= nearly_one" to avoid zero vector which can NOT be normalized
        return DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(sin_theta_div_2, cos_theta_div_2));
    }
    else if (cos_theta_div_2_square > zero && cos_theta > nearly_one)
    {
        return DirectX::XMQuaternionIdentity();
    }
    else
    {
        return DirectX::XMVectorSetW(DirectX::XMVector3Normalize(internal_calculate_perpendicular_vector(from)), 0.0F);
    }
}

extern void ik_one_joint_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joint_local_space, DirectX::XMFLOAT4X4 *const inout_joint_model_space)
{
    constexpr uint32_t const ball_and_socket_joint_index = 0U;
    constexpr uint32_t const end_effector_parent_joint_index = 0U;

    constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;
    constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;
    constexpr float const INTERNAL_LENGTH_EPSILON = 1E-6F;

    DirectX::XMMATRIX ball_and_socket_parent_transform_model_space;
    {
        DirectX::XMVECTOR unused_determinant;
        ball_and_socket_parent_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joint_local_space[ball_and_socket_joint_index])), DirectX::XMLoadFloat4x4(&inout_joint_model_space[ball_and_socket_joint_index]));
    }

    DirectX::XMVECTOR ball_and_socket_joint_model_space_translation;
    DirectX::XMVECTOR ball_and_socket_joint_model_space_rotation;
    {
        DirectX::XMMATRIX ball_and_socket_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joint_model_space[ball_and_socket_joint_index]);

        DirectX::XMVECTOR ball_and_socket_joint_model_space_scale;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_model_space_scale, &ball_and_socket_joint_model_space_rotation, &ball_and_socket_joint_model_space_translation, ball_and_socket_joint_transform_model_space);
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMVECTOR from_end_effector_to_target_model_space_rotation;
    {
        DirectX::XMVECTOR end_effector_model_space_translation;
        {
            DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joint_model_space[end_effector_parent_joint_index]));

            DirectX::XMVECTOR end_effector_model_space_scale;
            DirectX::XMVECTOR end_effector_model_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR end_effector_model_space_displacement = DirectX::XMVectorSubtract(end_effector_model_space_translation, ball_and_socket_joint_model_space_translation);

        DirectX::XMVECTOR target_model_space_displacement = DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&in_target_position_model_space), ball_and_socket_joint_model_space_translation);

        float end_effector_model_space_displacement_length = DirectX::XMVectorGetX(DirectX::XMVector3Length(end_effector_model_space_displacement));

        float target_model_space_displacement_length = DirectX::XMVectorGetX(DirectX::XMVector3Length(target_model_space_displacement));

        if ((end_effector_model_space_displacement_length > INTERNAL_LENGTH_EPSILON) && (target_model_space_displacement_length > INTERNAL_LENGTH_EPSILON))
        {
            assert((in_ball_and_socket_joint_gain >= 0.0F) && (in_ball_and_socket_joint_gain <= 1.0F));

            DirectX::XMVECTOR end_effector_model_space_direction = DirectX::XMVectorScale(end_effector_model_space_displacement, 1.0F / end_effector_model_space_displacement_length);

            DirectX::XMVECTOR target_model_space_direction = DirectX::XMVectorScale(target_model_space_displacement, 1.0F / target_model_space_displacement_length);

            from_end_effector_to_target_model_space_rotation = internal_compute_shortest_rotation_damped(end_effector_model_space_direction, target_model_space_direction, in_ball_and_socket_joint_gain);
        }
        else
        {
            from_end_effector_to_target_model_space_rotation = DirectX::XMQuaternionIdentity();
        }
    }

    DirectX::XMVECTOR updated_ball_and_socket_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(ball_and_socket_joint_model_space_rotation, from_end_effector_to_target_model_space_rotation));

    DirectX::XMMATRIX updated_ball_and_socket_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_model_space_translation));

    DirectX::XMStoreFloat4x4(&inout_joint_model_space[ball_and_socket_joint_index], updated_ball_and_socket_joint_model_space_transform);

    DirectX::XMVECTOR ball_and_socket_joint_local_space_translation;
    {
        DirectX::XMVECTOR ball_and_socket_joint_local_space_scale;
        DirectX::XMVECTOR ball_and_socket_joint_local_space_rotation;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_local_space_scale, &ball_and_socket_joint_local_space_rotation, &ball_and_socket_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joint_local_space[ball_and_socket_joint_index]));
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_rotation;
    {
        DirectX::XMVECTOR unused_determinant;
        DirectX::XMMATRIX unused_updated_ball_and_socket_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_ball_and_socket_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, ball_and_socket_parent_transform_model_space));

        DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_scale;
        DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_translation;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_ball_and_socket_joint_local_space_scale, &updated_ball_and_socket_joint_local_space_rotation, &updated_ball_and_socket_joint_local_space_translation, unused_updated_ball_and_socket_joint_local_space_transform);
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_ball_and_socket_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_ball_and_socket_joint_local_space_translation, ball_and_socket_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
    }

    DirectX::XMMATRIX updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_local_space_translation));

    DirectX::XMStoreFloat4x4(&inout_joint_local_space[ball_and_socket_joint_index], updated_current_joint_local_space_transform);
}

extern void ik_two_joints_solve(float const in_ball_and_socket_joint_gain, DirectX::XMFLOAT3 const &in_hinge_joint_axis_local_space, float const in_cosine_max_hinge_joint_angle, float const in_cosine_min_hinge_joint_angle, float const in_hinge_joint_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    constexpr uint32_t const ball_and_socket_joint_index = 0U;
    constexpr uint32_t const hinge_joint_index = 1U;
    constexpr uint32_t const end_effector_parent_joint_index = 1U;

    constexpr float const INTERNAL_SCALE_EPSILON = 1E-4F;
    constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;

    DirectX::XMMATRIX ball_and_socket_parent_transform_model_space;
    {
        DirectX::XMVECTOR unused_determinant;
        ball_and_socket_parent_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joints_local_space[ball_and_socket_joint_index])), DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]));
    }

    DirectX::XMVECTOR ball_and_socket_joint_model_space_translation;
    DirectX::XMVECTOR ball_and_socket_joint_model_space_rotation;
    {
        DirectX::XMMATRIX ball_and_socket_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]);

        DirectX::XMVECTOR ball_and_socket_joint_model_space_scale;
        bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_model_space_scale, &ball_and_socket_joint_model_space_rotation, &ball_and_socket_joint_model_space_translation, ball_and_socket_joint_transform_model_space);
        assert(directx_xm_matrix_decompose);

        assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
    }

    DirectX::XMVECTOR from_ball_and_socket_joint_to_target_model_space_translation = DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&in_target_position_model_space), ball_and_socket_joint_model_space_translation);

    {
        DirectX::XMVECTOR hinge_joint_model_space_translation;
        DirectX::XMVECTOR hinge_joint_model_space_rotation;
        DirectX::XMVECTOR hinge_joint_axis_model_space;
        {
            DirectX::XMMATRIX hinge_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joints_model_space[hinge_joint_index]);

            DirectX::XMVECTOR hinge_joint_model_space_scale;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&hinge_joint_model_space_scale, &hinge_joint_model_space_rotation, &hinge_joint_model_space_translation, hinge_joint_transform_model_space);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(hinge_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            DirectX::XMVECTOR hinge_joint_axis_local_space = DirectX::XMLoadFloat3(&in_hinge_joint_axis_local_space);

            hinge_joint_axis_model_space = DirectX::XMVector3Normalize(DirectX::XMVector3TransformNormal(hinge_joint_axis_local_space, hinge_joint_transform_model_space));
        }

        DirectX::XMVECTOR different_hinge_joint_model_space_rotation;
        {
            DirectX::XMVECTOR end_effector_model_space_translation;
            {
                DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joints_model_space[end_effector_parent_joint_index]));

                DirectX::XMVECTOR end_effector_model_space_scale;
                DirectX::XMVECTOR end_effector_model_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR v1 = DirectX::XMVectorSubtract(hinge_joint_model_space_translation, ball_and_socket_joint_model_space_translation);

            DirectX::XMVECTOR v1_in_normal = DirectX::XMVectorScale(hinge_joint_axis_model_space, DirectX::XMVectorGetX(DirectX::XMVector3Dot(v1, hinge_joint_axis_model_space)));

            DirectX::XMVECTOR v1_in_plane = DirectX::XMVectorSubtract(v1, v1_in_normal);

            float v1_in_plane_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(v1_in_plane, v1_in_plane));

            float v1_in_plane_length = internal_sqrt(v1_in_plane_length_square);

            DirectX::XMVECTOR v2 = DirectX::XMVectorSubtract(end_effector_model_space_translation, hinge_joint_model_space_translation);

            DirectX::XMVECTOR v2_in_normal = DirectX::XMVectorScale(hinge_joint_axis_model_space, DirectX::XMVectorGetX(DirectX::XMVector3Dot(v2, hinge_joint_axis_model_space)));

            DirectX::XMVECTOR v2_in_plane = DirectX::XMVectorSubtract(v2, v2_in_normal);

            float v2_in_plane_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(v2_in_plane, v2_in_plane));

            float v2_in_plane_length = internal_sqrt(v2_in_plane_length_square);

            DirectX::XMVECTOR h_in_normal = DirectX::XMVectorAdd(v1_in_normal, v2_in_normal);

            float h_in_normal_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(h_in_normal, h_in_normal));

            float d_length_square = DirectX::XMVectorGetX(DirectX::XMVector3Dot(from_ball_and_socket_joint_to_target_model_space_translation, from_ball_and_socket_joint_to_target_model_space_translation));

            float d_in_plane_length_square = d_length_square - h_in_normal_length_square;

            float cos_v1_v2_in_plane = (v1_in_plane_length_square + v2_in_plane_length_square - d_in_plane_length_square) * 0.5F / (v1_in_plane_length * v2_in_plane_length);

            assert(in_cosine_max_hinge_joint_angle < in_cosine_min_hinge_joint_angle);
            assert(in_cosine_max_hinge_joint_angle >= -1.0F);
            assert(in_cosine_max_hinge_joint_angle <= 1.0F);
            assert(in_cosine_min_hinge_joint_angle >= -1.0F);
            assert(in_cosine_min_hinge_joint_angle <= 1.0F);
            float cos_updated = std::min(std::max(in_cosine_max_hinge_joint_angle, cos_v1_v2_in_plane), in_cosine_min_hinge_joint_angle);

            float cos_current = DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMVectorNegate(v1_in_plane), v2_in_plane)) / (v1_in_plane_length * v2_in_plane_length);

            assert(DirectX::XMVectorGetX(DirectX::XMVector3Dot(hinge_joint_axis_model_space, DirectX::XMVector3Normalize(DirectX::XMVector3Cross(DirectX::XMVectorNegate(v1_in_plane), v2_in_plane)))) > (1.0F - 1E-6F));

            float cos_different;
            float sin_different;
            {
                float const cos_b = cos_updated;
                float const cos_a = cos_current;

                float sin_b_sqr = 1.0F - cos_b * cos_b;
                float sin_a_sqr = 1.0F - cos_a * cos_a;

                float sin_b = internal_sqrt(sin_b_sqr);
                float sin_a = internal_sqrt(sin_a_sqr);

                float cos_b_a = cos_b * cos_a + sin_b * sin_a;
                float sin_b_a = sin_b * cos_a - sin_a * cos_b;

                cos_different = cos_b_a;
                sin_different = sin_b_a;
            }

            assert((in_hinge_joint_gain >= 0.0F) && (in_hinge_joint_gain <= 1.0F));
            different_hinge_joint_model_space_rotation = internal_compute_rotation_axis_damped(hinge_joint_axis_model_space, cos_different, sin_different, in_hinge_joint_gain);
        }

        DirectX::XMVECTOR updated_hinge_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(hinge_joint_model_space_rotation, different_hinge_joint_model_space_rotation));

        DirectX::XMMATRIX updated_hinge_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_hinge_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(hinge_joint_model_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[hinge_joint_index], updated_hinge_joint_model_space_transform);

        DirectX::XMVECTOR hinge_joint_local_space_translation;
        {
            DirectX::XMVECTOR hinge_joint_local_space_scale;
            DirectX::XMVECTOR hinge_joint_local_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&hinge_joint_local_space_scale, &hinge_joint_local_space_rotation, &hinge_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_local_space[hinge_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR updated_hinge_joint_local_space_rotation;
        {
            DirectX::XMMATRIX ball_and_socket_joint_transform_model_space = DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]);

            DirectX::XMVECTOR unused_determinant;
            DirectX::XMMATRIX unused_updated_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_hinge_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, ball_and_socket_joint_transform_model_space));

            DirectX::XMVECTOR updated_hinge_joint_local_space_scale;
            DirectX::XMVECTOR updated_hinge_joint_local_space_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_hinge_joint_local_space_scale, &updated_hinge_joint_local_space_rotation, &updated_hinge_joint_local_space_translation, unused_updated_hinge_joint_local_space_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;
            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_hinge_joint_local_space_translation, hinge_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
        }

        DirectX::XMMATRIX updated_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_hinge_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(hinge_joint_local_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_local_space[hinge_joint_index], updated_hinge_joint_local_space_transform);
    }

    {
        DirectX::XMVECTOR from_end_effector_to_target_model_space_rotation;
        {
            DirectX::XMVECTOR end_effector_model_space_translation;
            {
                DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joints_model_space[end_effector_parent_joint_index]));

                DirectX::XMVECTOR end_effector_model_space_scale;
                DirectX::XMVECTOR end_effector_model_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR end_effector_model_space_direction = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(end_effector_model_space_translation, ball_and_socket_joint_model_space_translation));

            DirectX::XMVECTOR target_model_space_direction = DirectX::XMVector3Normalize(from_ball_and_socket_joint_to_target_model_space_translation);

            assert((in_ball_and_socket_joint_gain >= 0.0F) && (in_ball_and_socket_joint_gain <= 1.0F));
            from_end_effector_to_target_model_space_rotation = internal_compute_shortest_rotation_damped(end_effector_model_space_direction, target_model_space_direction, in_ball_and_socket_joint_gain);
        }

        DirectX::XMVECTOR updated_ball_and_socket_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(ball_and_socket_joint_model_space_rotation, from_end_effector_to_target_model_space_rotation));

        DirectX::XMMATRIX updated_ball_and_socket_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_model_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index], updated_ball_and_socket_joint_model_space_transform);

        DirectX::XMVECTOR ball_and_socket_joint_local_space_translation;
        {
            DirectX::XMVECTOR ball_and_socket_joint_local_space_scale;
            DirectX::XMVECTOR ball_and_socket_joint_local_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_local_space_scale, &ball_and_socket_joint_local_space_rotation, &ball_and_socket_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_local_space[ball_and_socket_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_rotation;
        {
            DirectX::XMVECTOR unused_determinant;
            DirectX::XMMATRIX unused_updated_ball_and_socket_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_ball_and_socket_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, ball_and_socket_parent_transform_model_space));

            DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_scale;
            DirectX::XMVECTOR updated_ball_and_socket_joint_local_space_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_ball_and_socket_joint_local_space_scale, &updated_ball_and_socket_joint_local_space_rotation, &updated_ball_and_socket_joint_local_space_translation, unused_updated_ball_and_socket_joint_local_space_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_ball_and_socket_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_ball_and_socket_joint_local_space_translation, ball_and_socket_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
        }

        DirectX::XMMATRIX updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_ball_and_socket_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(ball_and_socket_joint_local_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_local_space[ball_and_socket_joint_index], updated_current_joint_local_space_transform);

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[hinge_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&inout_joints_local_space[hinge_joint_index]), DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index])));
    }
}

static inline void internal_ik_ccd_solve_iteration(float const in_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, DirectX::XMFLOAT4X4 const &in_base_parent_transform_model_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;
    constexpr float const INTERNAL_TRANSLATION_EPSILON = 7E-5F;

    for (uint32_t current_joint_index_plus_1 = in_joint_count; current_joint_index_plus_1 >= 1U; --current_joint_index_plus_1)
    {
        uint32_t const current_joint_index = current_joint_index_plus_1 - 1U;

        DirectX::XMVECTOR current_joint_model_space_rotation;
        DirectX::XMVECTOR current_joint_model_space_translation;
        {
            DirectX::XMVECTOR current_joint_model_space_scale;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&current_joint_model_space_scale, &current_joint_model_space_rotation, &current_joint_model_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_model_space[current_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(current_joint_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR from_end_effector_to_target_model_space_rotation;
        {
            DirectX::XMVECTOR end_effector_model_space_translation;
            {
                uint32_t const end_effector_parent_joint_index = in_joint_count - 1U;

                DirectX::XMMATRIX end_effector_transform_model_space = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space), DirectX::XMLoadFloat4x4(&inout_joints_model_space[end_effector_parent_joint_index]));

                DirectX::XMVECTOR end_effector_model_space_scale;
                DirectX::XMVECTOR end_effector_model_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_model_space_scale, &end_effector_model_space_rotation, &end_effector_model_space_translation, end_effector_transform_model_space);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_model_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR end_effector_model_space_direction = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(end_effector_model_space_translation, current_joint_model_space_translation));

            DirectX::XMVECTOR target_model_space_direction = DirectX::XMVector3Normalize(DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&in_target_position_model_space), current_joint_model_space_translation));

            assert((in_gain >= 0.0F) && (in_gain <= 1.0F));
            from_end_effector_to_target_model_space_rotation = internal_compute_shortest_rotation_damped(end_effector_model_space_direction, target_model_space_direction, in_gain);
        }

        DirectX::XMVECTOR updated_current_joint_model_space_rotation = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(current_joint_model_space_rotation, from_end_effector_to_target_model_space_rotation));

        DirectX::XMMATRIX updated_current_joint_model_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_current_joint_model_space_rotation), DirectX::XMMatrixTranslationFromVector(current_joint_model_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_model_space[current_joint_index], updated_current_joint_model_space_transform);

        DirectX::XMVECTOR current_joint_local_space_translation;
        {
            DirectX::XMVECTOR current_joint_local_space_scale;
            DirectX::XMVECTOR current_joint_local_space_rotation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&current_joint_local_space_scale, &current_joint_local_space_rotation, &current_joint_local_space_translation, DirectX::XMLoadFloat4x4(&inout_joints_local_space[current_joint_index]));
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(current_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
        }

        DirectX::XMVECTOR updated_current_joint_local_space_rotation;
        {
            DirectX::XMMATRIX current_parent_joint_model_space = (current_joint_index >= 1U) ? DirectX::XMLoadFloat4x4(&inout_joints_model_space[current_joint_index - 1U]) : DirectX::XMLoadFloat4x4(&in_base_parent_transform_model_space);

            DirectX::XMVECTOR unused_determinant;

            DirectX::XMMATRIX unused_updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(updated_current_joint_model_space_transform, DirectX::XMMatrixInverse(&unused_determinant, current_parent_joint_model_space));

            DirectX::XMVECTOR updated_current_joint_local_space_scale;
            DirectX::XMVECTOR updated_current_joint_local_space_translation;
            bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&updated_current_joint_local_space_scale, &updated_current_joint_local_space_rotation, &updated_current_joint_local_space_translation, unused_updated_current_joint_local_space_transform);
            assert(directx_xm_matrix_decompose);

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_current_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));

            assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(updated_current_joint_local_space_translation, current_joint_local_space_translation)), DirectX::XMVectorReplicate(INTERNAL_TRANSLATION_EPSILON))));
        }

        DirectX::XMMATRIX updated_current_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(updated_current_joint_local_space_rotation), DirectX::XMMatrixTranslationFromVector(current_joint_local_space_translation));

        DirectX::XMStoreFloat4x4(&inout_joints_local_space[current_joint_index], updated_current_joint_local_space_transform);

        for (uint32_t child_joint_index_plus_1 = (current_joint_index_plus_1 + 1U); child_joint_index_plus_1 <= in_joint_count; ++child_joint_index_plus_1)
        {
            uint32_t const parent_joint_index = child_joint_index_plus_1 - 1U - 1U;
            uint32_t const child_joint_index = child_joint_index_plus_1 - 1U;
            DirectX::XMStoreFloat4x4(&inout_joints_model_space[child_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&inout_joints_local_space[child_joint_index]), DirectX::XMLoadFloat4x4(&inout_joints_model_space[parent_joint_index])));
        }
    }
}

extern void ik_ccd_solve(uint32_t const in_iteration, float const in_gain, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    DirectX::XMFLOAT4X4 in_base_parent_transform_model_space;
    assert(in_joint_count >= 1U);
    {
        DirectX::XMVECTOR unused_determinant;
        DirectX::XMStoreFloat4x4(&in_base_parent_transform_model_space, DirectX::XMMatrixMultiply(DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joints_local_space[0])), DirectX::XMLoadFloat4x4(&inout_joints_model_space[0])));
    }

    for (uint32_t iteration_index = 0U; iteration_index < in_iteration; ++iteration_index)
    {
        internal_ik_ccd_solve_iteration(in_gain, in_target_position_model_space, in_end_effector_transform_local_space, in_base_parent_transform_model_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
    }
}

extern void ik_reaching_solve(DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *const inout_joints_local_space, DirectX::XMFLOAT4X4 *const inout_joints_model_space)
{
    constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;

    // NOTE: the model space of all children of the end effector should also be marked as invalid

    if (1U == in_joint_count)
    {
        ik_one_joint_solve(1.0F, in_target_position_model_space, in_end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);
    }
    else if (2U == in_joint_count)
    {
        DirectX::XMFLOAT3 hinge_joint_axis_local_space;
        {
            DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_translation;
            {
                constexpr uint32_t const ball_and_socket_joint_index = 0U;
                constexpr uint32_t const hinge_joint_index = 1U;

                DirectX::XMVECTOR unused_determinant;
                DirectX::XMMATRIX ball_and_socket_joint_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&inout_joints_model_space[ball_and_socket_joint_index]), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&inout_joints_model_space[hinge_joint_index])));

                DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_scale;
                DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&ball_and_socket_joint_hinge_joint_local_space_scale, &ball_and_socket_joint_hinge_joint_local_space_rotation, &ball_and_socket_joint_hinge_joint_local_space_translation, ball_and_socket_joint_hinge_joint_local_space_transform);
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(ball_and_socket_joint_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMVECTOR end_effector_hinge_joint_local_space_translation;
            {
                DirectX::XMVECTOR end_effector_hinge_joint_local_space_scale;
                DirectX::XMVECTOR end_effector_hinge_joint_local_space_rotation;
                bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_hinge_joint_local_space_scale, &end_effector_hinge_joint_local_space_rotation, &end_effector_hinge_joint_local_space_translation, DirectX::XMLoadFloat4x4(&in_end_effector_transform_local_space));
                assert(directx_xm_matrix_decompose);

                assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
            }

            DirectX::XMStoreFloat3(&hinge_joint_axis_local_space, DirectX::XMVector3Normalize(DirectX::XMVector3Cross(ball_and_socket_joint_hinge_joint_local_space_translation, end_effector_hinge_joint_local_space_translation)));
        }

        ik_two_joints_solve(1.0F, hinge_joint_axis_local_space, -1.0F, 1.0F, 1.0F, in_target_position_model_space, in_end_effector_transform_local_space, inout_joints_local_space, inout_joints_model_space);
    }
    else if (3U == in_joint_count)
    {
        // TODO: three joints IK
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
    }
    else
    {
        assert(in_joint_count >= 4U);
        ik_ccd_solve(8U, 0.5F, in_target_position_model_space, in_end_effector_transform_local_space, in_joint_count, inout_joints_local_space, inout_joints_model_space);
    }
}
