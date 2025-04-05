//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_MMD_MMDIKSOLVER_H_
#define SABA_MODEL_MMD_MMDIKSOLVER_H_

#include <DirectXMath.h>

namespace saba
{
	class MMDIkSolver
	{
	public:
		static void Solve(DirectX::XMFLOAT3 const &in_two_joints_hinge_joint_axis_local_space, float const in_two_joints_cosine_max_hinge_joint_angle, float const in_two_joints_cosine_min_hinge_joint_angle, DirectX::XMFLOAT3 const &in_target_position_model_space, DirectX::XMFLOAT4X4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, DirectX::XMFLOAT4X4 *inout_joints_local_space, DirectX::XMFLOAT4X4 *inout_joints_model_space);
	};
}

#endif // !SABA_MODEL_MMD_MMDIKSOLVER_H_
