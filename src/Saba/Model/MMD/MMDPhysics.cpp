//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#include "MMDPhysics.h"

#include "MMDNode.h"
#include "MMDModel.h"
#include "Saba/Base/Log.h"

#include <glm/gtc/matrix_transform.hpp>

#include <DirectXMath.h>

namespace brx
{
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_input_stream.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_scene.h"
}

static inline void *_internal_dynamic_link_open(wchar_t const *filename);
static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol);

#ifndef NDEBUG
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/McRT-Malloc");
void *const dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Physics-BT");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/McRT-Malloc");
void *const dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Physics-BT");
#endif
decltype(brx_physics_create_context) *const physics_create_context = reinterpret_cast<decltype(brx_physics_create_context) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_create_context"));
decltype(brx_physics_destory_context) *const physics_destory_context = reinterpret_cast<decltype(brx_physics_destory_context) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_destory_context"));
decltype(brx_physics_create_world) *const physics_create_world = reinterpret_cast<decltype(brx_physics_create_world) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_create_world"));
decltype(brx_physics_destory_world) *const physics_destory_world = reinterpret_cast<decltype(brx_physics_destory_world) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_destory_world"));
decltype(brx_physics_world_add_rigid_body) *const physics_world_add_rigid_body = reinterpret_cast<decltype(brx_physics_world_add_rigid_body) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_world_add_rigid_body"));
decltype(brx_physics_world_remove_rigid_body) *const physics_world_remove_rigid_body = reinterpret_cast<decltype(brx_physics_world_remove_rigid_body) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_world_remove_rigid_body"));
decltype(brx_physics_world_add_constraint) *const physics_world_add_constraint = reinterpret_cast<decltype(brx_physics_world_add_constraint) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_world_add_constraint"));
decltype(brx_physics_world_remove_constraint) *const physics_world_remove_constraint = reinterpret_cast<decltype(brx_physics_world_remove_constraint) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_world_remove_constraint"));
decltype(brx_physics_world_step) *const physics_world_step = reinterpret_cast<decltype(brx_physics_world_step) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_world_step"));
decltype(brx_physics_create_rigid_body) *const physics_create_rigid_body = reinterpret_cast<decltype(brx_physics_create_rigid_body) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_create_rigid_body"));
decltype(brx_physics_destory_rigid_body) *const physics_destory_rigid_body = reinterpret_cast<decltype(brx_physics_destory_rigid_body) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_destory_rigid_body"));
decltype(brx_physics_rigid_body_apply_key_frame) *const physics_rigid_body_apply_key_frame = reinterpret_cast<decltype(brx_physics_rigid_body_apply_key_frame) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_rigid_body_apply_key_frame"));
decltype(brx_physics_rigid_body_set_transform) *const physics_rigid_body_set_transform = reinterpret_cast<decltype(brx_physics_rigid_body_set_transform) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_rigid_body_set_transform"));
decltype(brx_physics_rigid_body_get_transform) *const physics_rigid_body_get_transform = reinterpret_cast<decltype(brx_physics_rigid_body_get_transform) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_rigid_body_get_transform"));
decltype(brx_physics_create_constraint) *const physics_create_constraint = reinterpret_cast<decltype(brx_physics_create_constraint) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_create_constraint"));
decltype(brx_physics_destory_constraint) *const physics_destory_constraint = reinterpret_cast<decltype(brx_physics_destory_constraint) *>(_internal_dynamic_link_symbol(dynamic_link_handle, "brx_physics_destory_constraint"));

namespace saba
{
	MMDPhysics::MMDPhysics() : m_physics_context(NULL), m_physics_world(NULL)
	{
	}

	MMDPhysics::~MMDPhysics()
	{
		Destroy();
	}

	bool MMDPhysics::Create()
	{
		float gravity[3] = {0, -9.8F, 0};

		assert(NULL == m_physics_context);
		assert(NULL == m_physics_world);
		m_physics_context = physics_create_context();
		m_physics_world = physics_create_world(m_physics_context, gravity);

		return true;
	}

	void MMDPhysics::Destroy()
	{
		for (brx_physics_constraint *ragdoll_constraint : m_physics_constraints)
		{
			physics_world_remove_constraint(m_physics_context, m_physics_world, ragdoll_constraint);
		}

		for (brx_physics_rigid_body *ragdoll_rigid_body : m_physics_rigid_bodies)
		{
			physics_world_remove_rigid_body(m_physics_context, m_physics_world, ragdoll_rigid_body);
		}

		for (brx_physics_constraint *ragdoll_constraint : m_physics_constraints)
		{
			physics_destory_constraint(m_physics_context, m_physics_world, ragdoll_constraint);
		}

		for (brx_physics_rigid_body *ragdoll_rigid_body : m_physics_rigid_bodies)
		{
			physics_destory_rigid_body(m_physics_context, m_physics_world, ragdoll_rigid_body);
		}

		assert(NULL != m_physics_context);
		assert(NULL != m_physics_world);
		physics_destory_world(m_physics_context, m_physics_world);
		physics_destory_context(m_physics_context);
		m_physics_context = NULL;
		m_physics_world = NULL;
	}

	static inline BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE unwrap(BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type)
	{
		static_assert(BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_SPHERE, "");
		static_assert(BRX_PHYSICS_RIGID_BODY_SHAPE_BOX == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_BOX, "");
		static_assert(BRX_PHYSICS_RIGID_BODY_SHAPE_CAPSULE == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_CAPSULE, "");
		assert(BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_SPHERE == shape_type || BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_BOX == shape_type || BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_CAPSULE == shape_type);
		return static_cast<BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE>(shape_type);
	}

	static inline BRX_PHYSICS_RIGID_BODY_MOTION_TYPE unwrap(BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type)
	{
		static_assert(BRX_PHYSICS_RIGID_BODY_MOTION_FIXED == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_FIXED, "");
		static_assert(BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_KEYFRAME, "");
		static_assert(BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_DYNAMIC, "");
		assert(BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_FIXED == motion_type || BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_KEYFRAME == motion_type || BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_DYNAMIC == motion_type);
		return static_cast<BRX_PHYSICS_RIGID_BODY_MOTION_TYPE>(motion_type);
	}

	static inline BRX_PHYSICS_CONSTRAINT_TYPE unwrap(BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_TYPE constraint_type)
	{
		static_assert(BRX_PHYSICS_CONSTRAINT_FIXED == BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_FIXED, "");
		static_assert(BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET == BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_BALL_AND_SOCKET, "");
		static_assert(BRX_PHYSICS_CONSTRAINT_HINGE == BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_HINGE, "");
		static_assert(BRX_PHYSICS_CONSTRAINT_PRISMATIC == BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_PRISMATIC, "");
		static_assert(BRX_PHYSICS_CONSTRAINT_RAGDOLL == BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_RAGDOLL, "");
		assert(BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_FIXED == constraint_type || BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_BALL_AND_SOCKET == constraint_type || BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_HINGE == constraint_type || BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_PRISMATIC == constraint_type || BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_RAGDOLL == constraint_type);
		return static_cast<BRX_PHYSICS_CONSTRAINT_TYPE>(constraint_type);
	}

	void MMDPhysics::InitRagdoll(brx_asset_import_surface_group const *surface_group)
	{
		uint32_t const ragdoll_skeleton_rigid_body_count = surface_group->get_ragdoll_skeleton_rigid_body_count();
		assert(this->m_physics_rigid_bodies.empty());
		for (uint32_t ragdoll_skeleton_rigid_body_index = 0U; ragdoll_skeleton_rigid_body_index < ragdoll_skeleton_rigid_body_count; ++ragdoll_skeleton_rigid_body_index)
		{
			brx_asset_import_physics_rigid_body const *const ragdoll_skeleton_rigid_body = surface_group->get_ragdoll_skeleton_rigid_body(ragdoll_skeleton_rigid_body_index);

			brx_physics_rigid_body *physics_rigid_body = physics_create_rigid_body(this->m_physics_context, this->m_physics_world, ragdoll_skeleton_rigid_body->m_model_space_transform.m_rotation, ragdoll_skeleton_rigid_body->m_model_space_transform.m_translation, unwrap(ragdoll_skeleton_rigid_body->m_shape_type), ragdoll_skeleton_rigid_body->m_shape_size, unwrap(ragdoll_skeleton_rigid_body->m_motion_type), ragdoll_skeleton_rigid_body->m_collision_filter_group, ragdoll_skeleton_rigid_body->m_collision_filter_mask, ragdoll_skeleton_rigid_body->m_mass, ragdoll_skeleton_rigid_body->m_linear_damping, ragdoll_skeleton_rigid_body->m_angular_damping, ragdoll_skeleton_rigid_body->m_friction, ragdoll_skeleton_rigid_body->m_restitution);

			physics_world_add_rigid_body(this->m_physics_context, this->m_physics_world, physics_rigid_body);

			this->m_physics_rigid_bodies.push_back(physics_rigid_body);
		}
		assert(this->m_physics_rigid_bodies.size() == ragdoll_skeleton_rigid_body_count);

		uint32_t const ragdoll_skeleton_constraint_count = surface_group->get_ragdoll_skeleton_constraint_count();
		assert(this->m_physics_constraints.empty());
		for (uint32_t ragdoll_skeleton_constraint_index = 0U; ragdoll_skeleton_constraint_index < ragdoll_skeleton_constraint_count; ++ragdoll_skeleton_constraint_index)
		{
			brx_asset_import_physics_constraint const *const ragdoll_skeleton_constraint = surface_group->get_ragdoll_skeleton_constraint(ragdoll_skeleton_constraint_index);

			brx_physics_constraint *physics_constraint = physics_create_constraint(this->m_physics_context, this->m_physics_world, this->m_physics_rigid_bodies[ragdoll_skeleton_constraint->m_rigid_body_a_index], this->m_physics_rigid_bodies[ragdoll_skeleton_constraint->m_rigid_body_b_index], unwrap(ragdoll_skeleton_constraint->m_constraint_type), ragdoll_skeleton_constraint->m_pivot, ragdoll_skeleton_constraint->m_twist_axis, ragdoll_skeleton_constraint->m_plane_axis, ragdoll_skeleton_constraint->m_normal_axis, ragdoll_skeleton_constraint->m_twist_limit, ragdoll_skeleton_constraint->m_plane_limit, ragdoll_skeleton_constraint->m_normal_limit);

			physics_world_add_constraint(this->m_physics_context, this->m_physics_world, physics_constraint);

			this->m_physics_constraints.push_back(physics_constraint);
		}
		assert(this->m_physics_constraints.size() == ragdoll_skeleton_constraint_count);

		uint32_t const animation_to_ragdoll_direct_mapping_count = surface_group->get_animation_to_ragdoll_direct_mapping_count();
		assert(this->m_animation_to_ragdoll_mapping.empty());
		for (uint32_t animation_to_ragdoll_direct_mapping_index = 0U; animation_to_ragdoll_direct_mapping_index < animation_to_ragdoll_direct_mapping_count; ++animation_to_ragdoll_direct_mapping_index)
		{
			brx_asset_import_ragdoll_direct_mapping const *const animation_to_ragdoll_direct_mapping = surface_group->get_animation_to_ragdoll_direct_mapping(animation_to_ragdoll_direct_mapping_index);

			this->m_animation_to_ragdoll_mapping.push_back(*animation_to_ragdoll_direct_mapping);
		}
		assert(this->m_animation_to_ragdoll_mapping.size() == animation_to_ragdoll_direct_mapping_count);

		uint32_t const ragdoll_to_animation_direct_mapping_count = surface_group->get_ragdoll_to_animation_direct_mapping_count();
		assert(this->m_ragdoll_to_animation_mapping.empty());
		for (uint32_t ragdoll_to_animation_direct_mapping_index = 0U; ragdoll_to_animation_direct_mapping_index < ragdoll_to_animation_direct_mapping_count; ++ragdoll_to_animation_direct_mapping_index)
		{
			brx_asset_import_ragdoll_direct_mapping const *const ragdoll_to_animation_direct_mapping = surface_group->get_ragdoll_to_animation_direct_mapping(ragdoll_to_animation_direct_mapping_index);

			this->m_ragdoll_to_animation_mapping.push_back(*ragdoll_to_animation_direct_mapping);
		}
		assert(this->m_ragdoll_to_animation_mapping.size() == ragdoll_to_animation_direct_mapping_count);
	}

	void MMDPhysics::AnimationToRagdoll(glm::mat4x4 const *const in_animation_skeleton_pose_model_space)
	{
		for (brx_asset_import_ragdoll_direct_mapping const &ragdoll_direct_mapping : this->m_animation_to_ragdoll_mapping)
		{
			glm::mat4 animation_transform = in_animation_skeleton_pose_model_space[ragdoll_direct_mapping.m_joint_index_a];

			glm::mat4 ragdoll_transform = animation_transform * (*reinterpret_cast<glm::mat4 const *>(&ragdoll_direct_mapping.m_a_to_b_transform_model_space[0][0]));

			float brx_rotation[4];
			float brx_translation[3];
			{
				glm::vec3 translate = glm::vec3(ragdoll_transform[3]);
				brx_translation[0] = translate.x;
				brx_translation[1] = translate.y;
				brx_translation[2] = translate.z;

				glm::vec3 scale = glm::vec3(
					glm::length(glm::vec3(ragdoll_transform[0])),
					glm::length(glm::vec3(ragdoll_transform[1])),
					glm::length(glm::vec3(ragdoll_transform[2])));
				assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

				glm::quat rotate = glm::quat_cast(glm::mat3(
					glm::vec3(ragdoll_transform[0]) / scale.x,
					glm::vec3(ragdoll_transform[1]) / scale.y,
					glm::vec3(ragdoll_transform[2]) / scale.z));
				brx_rotation[0] = rotate.x;
				brx_rotation[1] = rotate.y;
				brx_rotation[2] = rotate.z;
				brx_rotation[3] = rotate.w;
			}

			physics_rigid_body_set_transform(this->m_physics_context, this->m_physics_world, this->m_physics_rigid_bodies[ragdoll_direct_mapping.m_joint_index_b], brx_rotation, brx_translation);
		}
	}

	void MMDPhysics::RagdollToAnimation(glm::mat4x4 *const out_animation_skeleton_pose_model_space)
	{
		for (brx_asset_import_ragdoll_direct_mapping const &ragdoll_direct_mapping : this->m_ragdoll_to_animation_mapping)
		{
			float brx_rotation[4];
			float brx_position[3];
			physics_rigid_body_get_transform(this->m_physics_context, this->m_physics_world, this->m_physics_rigid_bodies[ragdoll_direct_mapping.m_joint_index_a], brx_rotation, brx_position);

			glm::mat4 ragdoll_transform = glm::translate(glm::mat4(1), glm::vec3(brx_position[0], brx_position[1], brx_position[2])) * glm::mat4_cast(glm::quat(brx_rotation[3], brx_rotation[0], brx_rotation[1], brx_rotation[2]));

			glm::mat4 animation_transform = ragdoll_transform * (*reinterpret_cast<glm::mat4 const *>(&ragdoll_direct_mapping.m_a_to_b_transform_model_space[0][0]));

			out_animation_skeleton_pose_model_space[ragdoll_direct_mapping.m_joint_index_b] = animation_transform;

			// check unmapped

			// order?
			// animation_nodes[ragdoll_direct_mapping.m_joint_index_b]->UpdateChildTransform();
		}
	}

	void MMDPhysics::Update(float time)
	{
		if (NULL != this->m_physics_world)
		{
			physics_world_step(this->m_physics_context, this->m_physics_world, time);
		}
	}
}

#if defined(__GNUC__)
#error 1
#elif defined(_MSC_VER)
#define WIN32_LEAN_AND_MEAN 1
#include <Windows.h>
static inline void *_internal_dynamic_link_open(wchar_t const *filename)
{
	HMODULE dynamic_link_handle = GetModuleHandleW(filename);
	if (NULL == dynamic_link_handle)
	{
		assert(ERROR_MOD_NOT_FOUND == GetLastError());

		dynamic_link_handle = LoadLibraryW(filename);
		if (NULL == dynamic_link_handle)
		{
			assert(ERROR_MOD_NOT_FOUND == GetLastError());
			assert(false);
		}
	}

	return dynamic_link_handle;
}

static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol)
{
	return reinterpret_cast<void *>(GetProcAddress(static_cast<HINSTANCE>(handle), symbol));
}
#endif
