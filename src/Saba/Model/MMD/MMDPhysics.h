//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_MMDMODEL_MMDPHYSICS_H_
#define SABA_MODEL_MMDMODEL_MMDPHYSICS_H_

#include <cstddef>
#include <cstdint>

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_input_stream.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_scene.h"

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Physics/include/brx_physics.h"

#include "PMDFile.h"
#include "PMXFile.h"
#include "MMDNode.h"

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include <vector>
#include <memory>
#include <cinttypes>

namespace saba
{
	class MMDPhysics;
	class MMDModel;
	class MMDNode;

	class MMDPhysics
	{
		brx_physics_context *m_physics_context;
		brx_physics_world *m_physics_world;

		std::vector<brx_physics_rigid_body *> m_physics_rigid_bodies;
		std::vector<brx_physics_constraint *> m_physics_constraints;
		std::vector<brx_asset_import_ragdoll_direct_mapping> m_animation_to_ragdoll_mapping;
		std::vector<brx_asset_import_ragdoll_direct_mapping> m_ragdoll_to_animation_mapping;

	public:
		MMDPhysics();
		~MMDPhysics();

		MMDPhysics(const MMDPhysics &rhs) = delete;
		MMDPhysics &operator=(const MMDPhysics &rhs) = delete;

		bool Create();
		void Destroy();

		void InitRagdoll(brx_asset_import_surface_group const *surface_group);
		void AnimationToRagdoll(glm::mat4x4 const *const in_animation_skeleton_model_space);
		void RagdollToAnimation(glm::mat4x4 *const out_animation_skeleton_model_space);

		void Update(float time);
	};

}
#endif // SABA_MODEL_MMDMODEL_MMDPHYSICS_H_
