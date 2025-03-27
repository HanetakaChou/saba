//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMDMODEL_MMDPHYSICS_H_
#define SABA_MODEL_MMDMODEL_MMDPHYSICS_H_

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Physics/thirdparty/Brioche-Physics/include/brx_physics.h"

#include "PMDFile.h"
#include "PMXFile.h"

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include <vector>
#include <memory>
#include <cinttypes>

struct brx_motion_ragdoll_direct_mapping
{
	uint32_t m_joint_index_a;
	uint32_t m_joint_index_b;
	float m_a_to_b_transform_model_space[4][4];
};

namespace saba
{
	class MMDPhysics;
	class MMDModel;
	class MMDNode;

	class MMDPhysics
	{
		brx_physics_context *m_jph_physics_context;
		brx_physics_world *m_jph_physics_world;

		std::vector<brx_physics_rigid_body *> m_jph_physics_rigid_bodies;
		std::vector<brx_physics_constraint *> m_jph_physics_constraints;
		std::vector<brx_motion_ragdoll_direct_mapping> m_animation_to_ragdoll_mapping;
		std::vector<brx_motion_ragdoll_direct_mapping> m_ragdoll_to_animation_mapping;

	public:
		MMDPhysics();
		~MMDPhysics();

		MMDPhysics(const MMDPhysics &rhs) = delete;
		MMDPhysics &operator=(const MMDPhysics &rhs) = delete;

		bool Create();
		void Destroy();

		void InitRagdoll(mcrt_vector<mmd_pmx_rigid_body_t> const &in_mmd_rigid_bodies, mcrt_vector<mmd_pmx_constraint_t> const &in_mmd_joints, uint32_t const *const in_model_node_to_animation_skeleton_joint_map, glm::mat4 const *const in_animation_pose_model_space);
		void AnimationToRagdoll(glm::mat4x4 const * const in_animation_skeleton_model_space);
		void RagdollToAnimation(glm::mat4x4 * const out_animation_skeleton_model_space);

		void Update(float time);
	};

}
#endif // SABA_MODEL_MMDMODEL_MMDPHYSICS_H_
