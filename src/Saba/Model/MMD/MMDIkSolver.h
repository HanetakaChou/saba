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

#include "MMDNode.h"

#include <vector>
#include <string>
#include <glm/vec3.hpp>

enum BRX_ASSET_IMPORT_IK_NAME : uint32_t
{
	// 右足ＩＫ
	BRX_ASSET_IMPORT_IK_NAME_MMD_RIGHT_FOOT = 0,
	// 右つま先ＩＫ
	BRX_ASSET_IMPORT_IK_NAME_MMD_RIGHT_TOE = 1,
	// 左足ＩＫ
	BRX_ASSET_IMPORT_IK_NAME_MMD_LEFT_FOOT = 2,
	// 左つま先ＩＫ
	BRX_ASSET_IMPORT_IK_NAME_MMD_LEFT_TOE = 3,
	//
	BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT = 4
};

namespace saba
{
	class MMDIkSolver
	{
	public:
		MMDIkSolver(std::string const &name);

		inline std::string GetName() const { return m_name; }
		inline void Enable(bool enable) { m_enable = enable; }
		inline bool Enabled() const { return m_enable; }
		inline void SaveBaseAnimation() { m_baseAnimEnable = m_enable; }
		inline void LoadBaseAnimation() { m_enable = m_baseAnimEnable; }
		inline void ClearBaseAnimation() { m_baseAnimEnable = true; }
		inline bool GetBaseAnimationEnabled() const { return m_baseAnimEnable; }

		static void Solve(glm::vec3 const &in_two_joints_hinge_joint_axis_local_space, float const in_two_joints_cosine_max_hinge_joint_angle, float const in_two_joints_cosine_min_hinge_joint_angle, glm::vec3 const &in_target_position_model_space, glm::mat4 const &in_end_effector_transform_local_space, uint32_t const in_joint_count, glm::mat4x4 *inout_joints_local_space, glm::mat4x4 *inout_joints_model_space);

	private:
		std::string m_name;
		bool m_enable;
		bool m_baseAnimEnable;
	};
}

#endif // !SABA_MODEL_MMD_MMDIKSOLVER_H_
