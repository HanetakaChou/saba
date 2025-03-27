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

static inline void *_internal_dynamic_link_open(wchar_t const *filename);
static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol);

#ifndef NDEBUG
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Physics/build-windows/bin/x64/Debug/McRT-Malloc");
void *const jph_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Physics/build-windows/bin/x64/Debug/BRX-Physics-BT");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Physics/build-windows/bin/x64/Release/McRT-Malloc");
void *const jph_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Physics/build-windows/bin/x64/Release/BRX-Physics-BT");
#endif
decltype(brx_physics_create_context) *const jph_physics_create_context = reinterpret_cast<decltype(brx_physics_create_context) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_context"));
decltype(brx_physics_destory_context) *const jph_physics_destory_context = reinterpret_cast<decltype(brx_physics_destory_context) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_context"));
decltype(brx_physics_create_world) *const jph_physics_create_world = reinterpret_cast<decltype(brx_physics_create_world) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_world"));
decltype(brx_physics_destory_world) *const jph_physics_destory_world = reinterpret_cast<decltype(brx_physics_destory_world) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_world"));
decltype(brx_physics_world_add_rigid_body) *const jph_physics_world_add_rigid_body = reinterpret_cast<decltype(brx_physics_world_add_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_add_rigid_body"));
decltype(brx_physics_world_remove_rigid_body) *const jph_physics_world_remove_rigid_body = reinterpret_cast<decltype(brx_physics_world_remove_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_remove_rigid_body"));
decltype(brx_physics_world_add_constraint) *const jph_physics_world_add_constraint = reinterpret_cast<decltype(brx_physics_world_add_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_add_constraint"));
decltype(brx_physics_world_remove_constraint) *const jph_physics_world_remove_constraint = reinterpret_cast<decltype(brx_physics_world_remove_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_remove_constraint"));
decltype(brx_physics_world_step) *const jph_physics_world_step = reinterpret_cast<decltype(brx_physics_world_step) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_step"));
decltype(brx_physics_create_rigid_body) *const jph_physics_create_rigid_body = reinterpret_cast<decltype(brx_physics_create_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_rigid_body"));
decltype(brx_physics_destory_rigid_body) *const jph_physics_destory_rigid_body = reinterpret_cast<decltype(brx_physics_destory_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_rigid_body"));
decltype(brx_physics_rigid_body_apply_key_frame) *const jph_physics_rigid_body_apply_key_frame = reinterpret_cast<decltype(brx_physics_rigid_body_apply_key_frame) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_apply_key_frame"));
decltype(brx_physics_rigid_body_set_transform) *const jph_physics_rigid_body_set_transform = reinterpret_cast<decltype(brx_physics_rigid_body_set_transform) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_set_transform"));
decltype(brx_physics_rigid_body_get_transform) *const jph_physics_rigid_body_get_transform = reinterpret_cast<decltype(brx_physics_rigid_body_get_transform) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_get_transform"));
decltype(brx_physics_create_constraint) *const jph_physics_create_constraint = reinterpret_cast<decltype(brx_physics_create_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_constraint"));
decltype(brx_physics_destory_constraint) *const jph_physics_destory_constraint = reinterpret_cast<decltype(brx_physics_destory_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_constraint"));

static inline float internal_asin(float y, float z)
{
	constexpr float const INTERNAL_EPSILON = 1E-6F;
	constexpr float const INTERNAL_PI = 3.14159265358979323846264338327950288F;

	if (z > INTERNAL_EPSILON)
	{
		if (std::abs(y) > INTERNAL_EPSILON)
		{
			float const sin = (y / z);
			return DirectX::XMScalarASin(sin);
		}
		else
		{
			return 0.0F;
		}
	}
	else
	{
		if (y > INTERNAL_EPSILON)
		{
			return (INTERNAL_PI * 0.5F);
		}
		else if (y < INTERNAL_EPSILON)
		{
			return (0.0F - (INTERNAL_PI * 0.5F));
		}
		else
		{
			return 0.0F;
		}
	}
}

static inline void internal_import_ragdoll_physics(mcrt_vector<mmd_pmx_rigid_body_t> const &in_mmd_rigid_bodies, mcrt_vector<mmd_pmx_constraint_t> const &in_mmd_constraints, uint32_t const *const in_model_node_to_animation_skeleton_joint_map, DirectX::XMFLOAT4X4 const *const in_animation_pose_model_space, mcrt_vector<brx_asset_import_physics_rigid_body> &out_ragdoll_rigid_bodies, mcrt_vector<brx_asset_import_physics_constraint> &out_ragdoll_constraints, mcrt_vector<uint32_t> &out_ragdoll_skeleton_joint_parent_indices, mcrt_vector<brx_asset_import_ragdoll_direct_mapping> &out_animation_to_ragdoll_mapping, mcrt_vector<brx_asset_import_ragdoll_direct_mapping> &out_ragdoll_to_animation_mapping);

namespace saba
{
	namespace
	{
		glm::mat4 InvZ(const glm::mat4 &m)
		{
			return m;
		}
	}

	MMDPhysics::MMDPhysics() : m_jph_physics_context(NULL), m_jph_physics_world(NULL)
	{
	}

	MMDPhysics::~MMDPhysics()
	{
		Destroy();
	}

	bool MMDPhysics::Create()
	{
		// unit of mmd model is dm (decimeter)?
		float gravity[3] = {0, -9.8F * 10.0F, 0};

		assert(NULL == m_jph_physics_context);
		assert(NULL == m_jph_physics_world);
		m_jph_physics_context = jph_physics_create_context();
		m_jph_physics_world = jph_physics_create_world(m_jph_physics_context, gravity);

		return true;
	}

	void MMDPhysics::Destroy()
	{
		for (brx_physics_constraint *jph_ragdoll_constraint : m_jph_physics_constraints)
		{
			jph_physics_world_remove_constraint(m_jph_physics_context, m_jph_physics_world, jph_ragdoll_constraint);
		}

		for (brx_physics_rigid_body *jph_ragdoll_rigid_body : m_jph_physics_rigid_bodies)
		{
			jph_physics_world_remove_rigid_body(m_jph_physics_context, m_jph_physics_world, jph_ragdoll_rigid_body);
		}

		for (brx_physics_constraint *jph_ragdoll_constraint : m_jph_physics_constraints)
		{
			jph_physics_destory_constraint(m_jph_physics_context, m_jph_physics_world, jph_ragdoll_constraint);
		}

		for (brx_physics_rigid_body *jph_ragdoll_rigid_body : m_jph_physics_rigid_bodies)
		{
			jph_physics_destory_rigid_body(m_jph_physics_context, m_jph_physics_world, jph_ragdoll_rigid_body);
		}

		assert(NULL != m_jph_physics_context);
		assert(NULL != m_jph_physics_world);
		jph_physics_destory_world(m_jph_physics_context, m_jph_physics_world);
		jph_physics_destory_context(m_jph_physics_context);
		m_jph_physics_context = NULL;
		m_jph_physics_world = NULL;
	}

	void MMDPhysics::InitRagdoll(mcrt_vector<mmd_pmx_rigid_body_t> const &in_mmd_rigid_bodies, mcrt_vector<mmd_pmx_constraint_t> const &in_mmd_constraints, uint32_t const *const in_model_node_to_animation_skeleton_joint_map, glm::mat4 const *const in_animation_pose_model_space)
	{
		mcrt_vector<uint32_t> ragdoll_skeleton_joint_parent_indices;
		mcrt_vector<brx_asset_import_physics_rigid_body> ragdoll_rigid_bodies;
		mcrt_vector<brx_asset_import_physics_constraint> ragdoll_constraints;
		internal_import_ragdoll_physics(in_mmd_rigid_bodies, in_mmd_constraints, in_model_node_to_animation_skeleton_joint_map, reinterpret_cast<DirectX::XMFLOAT4X4 const *>(in_animation_pose_model_space), ragdoll_rigid_bodies, ragdoll_constraints, ragdoll_skeleton_joint_parent_indices, m_animation_to_ragdoll_mapping, m_ragdoll_to_animation_mapping);

		assert(m_jph_physics_rigid_bodies.empty());
		for (brx_asset_import_physics_rigid_body const &ragdoll_rigid_body : ragdoll_rigid_bodies)
		{
			brx_physics_rigid_body *jph_physics_rigid_body = jph_physics_create_rigid_body(m_jph_physics_context, m_jph_physics_world, ragdoll_rigid_body.m_model_space_transform.m_rotation, ragdoll_rigid_body.m_model_space_transform.m_translation, static_cast<BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE>(ragdoll_rigid_body.m_shape_type), ragdoll_rigid_body.m_shape_size, static_cast<BRX_PHYSICS_RIGID_BODY_MOTION_TYPE>(ragdoll_rigid_body.m_motion_type), ragdoll_rigid_body.m_collision_filter_group, ragdoll_rigid_body.m_collision_filter_mask, ragdoll_rigid_body.m_mass, ragdoll_rigid_body.m_linear_damping, ragdoll_rigid_body.m_angular_damping, ragdoll_rigid_body.m_friction, ragdoll_rigid_body.m_restitution);

			jph_physics_world_add_rigid_body(m_jph_physics_context, m_jph_physics_world, jph_physics_rigid_body);

			m_jph_physics_rigid_bodies.push_back(jph_physics_rigid_body);
		}
		assert(m_jph_physics_rigid_bodies.size() == ragdoll_rigid_bodies.size());

		assert(m_jph_physics_constraints.empty());
		for (brx_asset_import_physics_constraint const &ragdoll_constraint : ragdoll_constraints)
		{
			brx_physics_constraint *jph_physics_constraint = jph_physics_create_constraint(m_jph_physics_context, m_jph_physics_world, m_jph_physics_rigid_bodies[ragdoll_constraint.m_rigid_body_a_index], m_jph_physics_rigid_bodies[ragdoll_constraint.m_rigid_body_b_index], static_cast<BRX_PHYSICS_CONSTRAINT_TYPE>(ragdoll_constraint.m_constraint_type), ragdoll_constraint.m_pivot, ragdoll_constraint.m_twist_axis, ragdoll_constraint.m_plane_axis, ragdoll_constraint.m_normal_axis, ragdoll_constraint.m_twist_limit, ragdoll_constraint.m_plane_limit, ragdoll_constraint.m_normal_limit);

			jph_physics_world_add_constraint(m_jph_physics_context, m_jph_physics_world, jph_physics_constraint);

			m_jph_physics_constraints.push_back(jph_physics_constraint);
		}
		assert(m_jph_physics_constraints.size() == ragdoll_constraints.size());
	}

	void MMDPhysics::AnimationToRagdoll(glm::mat4x4 const *const in_animation_skeleton_pose_model_space)
	{
		for (brx_asset_import_ragdoll_direct_mapping const &ragdoll_direct_mapping : m_animation_to_ragdoll_mapping)
		{
			glm::mat4 animation_transform = in_animation_skeleton_pose_model_space[ragdoll_direct_mapping.m_joint_index_a];

			glm::mat4 ragdoll_transform = animation_transform * (*reinterpret_cast<glm::mat4 const *>(&ragdoll_direct_mapping.m_a_to_b_transform_model_space[0][0]));

			// TODO: remove this
			ragdoll_transform = InvZ(ragdoll_transform);

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

			jph_physics_rigid_body_set_transform(m_jph_physics_context, m_jph_physics_world, m_jph_physics_rigid_bodies[ragdoll_direct_mapping.m_joint_index_b], brx_rotation, brx_translation);
		}
	}

	void MMDPhysics::RagdollToAnimation(glm::mat4x4 *const out_animation_skeleton_pose_model_space)
	{
		for (brx_asset_import_ragdoll_direct_mapping const &ragdoll_direct_mapping : m_ragdoll_to_animation_mapping)
		{
			float brx_rotation[4];
			float brx_position[3];
			jph_physics_rigid_body_get_transform(m_jph_physics_context, m_jph_physics_world, m_jph_physics_rigid_bodies[ragdoll_direct_mapping.m_joint_index_a], brx_rotation, brx_position);

			glm::mat4 ragdoll_transform = glm::translate(glm::mat4(1), glm::vec3(brx_position[0], brx_position[1], brx_position[2])) * glm::mat4_cast(glm::quat(brx_rotation[3], brx_rotation[0], brx_rotation[1], brx_rotation[2]));

			// TODO: remove this
			ragdoll_transform = InvZ(ragdoll_transform);

			glm::mat4 animation_transform = ragdoll_transform * (*reinterpret_cast<glm::mat4 const *>(&ragdoll_direct_mapping.m_a_to_b_transform_model_space[0][0]));

			out_animation_skeleton_pose_model_space[ragdoll_direct_mapping.m_joint_index_b] = animation_transform;

			// check unmapped

			// order?
			// animation_nodes[ragdoll_direct_mapping.m_joint_index_b]->UpdateChildTransform();
		}
	}

	void MMDPhysics::Update(float time)
	{
		if (NULL != m_jph_physics_world)
		{
			jph_physics_world_step(m_jph_physics_context, m_jph_physics_world, time);
		}
	}
}

static inline void internal_import_ragdoll_physics(mcrt_vector<mmd_pmx_rigid_body_t> const &in_mmd_rigid_bodies, mcrt_vector<mmd_pmx_constraint_t> const &in_mmd_constraints, uint32_t const *const in_model_node_to_animation_skeleton_joint_map, DirectX::XMFLOAT4X4 const *const in_animation_pose_model_space, mcrt_vector<brx_asset_import_physics_rigid_body> &out_ragdoll_rigid_bodies, mcrt_vector<brx_asset_import_physics_constraint> &out_ragdoll_constraints, mcrt_vector<uint32_t> &out_ragdoll_skeleton_joint_parent_indices, mcrt_vector<brx_asset_import_ragdoll_direct_mapping> &out_animation_to_ragdoll_mapping, mcrt_vector<brx_asset_import_ragdoll_direct_mapping> &out_ragdoll_to_animation_mapping)
{
	uint32_t const rigid_body_count = in_mmd_rigid_bodies.size();

	mcrt_vector<uint32_t> ragdoll_skeleton_joint_to_rigid_body_map(static_cast<size_t>(rigid_body_count), BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
	mcrt_vector<uint32_t> rigid_body_to_ragdoll_skeleton_joint_map(static_cast<size_t>(rigid_body_count), BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
	{
		assert(out_ragdoll_skeleton_joint_parent_indices.empty());
		out_ragdoll_skeleton_joint_parent_indices = {};

		mcrt_vector<uint32_t> rigid_body_parent_indices(static_cast<size_t>(rigid_body_count), BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
		for (mmd_pmx_constraint_t const &mmd_constraint : in_mmd_constraints)
		{
			uint32_t const parent_index = mmd_constraint.m_rigid_body_a_index;
			uint32_t const child_index = mmd_constraint.m_rigid_body_b_index;

			if (parent_index < child_index)
			{
				if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == rigid_body_parent_indices[child_index])
				{
					rigid_body_parent_indices[child_index] = parent_index;
				}
				else
				{
					if (rigid_body_parent_indices[child_index] < parent_index)
					{
						rigid_body_parent_indices[child_index] = parent_index;
					}
				}
			}
		}

		mcrt_vector<uint32_t> rigid_body_depth_first_search_stack;
		mcrt_vector<mcrt_vector<uint32_t>> rigid_body_children_indices(static_cast<size_t>(rigid_body_count));
		for (uint32_t rigid_body_index_plus_1 = rigid_body_count; rigid_body_index_plus_1 > 0U; --rigid_body_index_plus_1)
		{
			uint32_t const rigid_body_index = rigid_body_index_plus_1 - 1U;
			uint32_t rigid_body_parent_index = rigid_body_parent_indices[rigid_body_index];
			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != rigid_body_parent_index)
			{
				rigid_body_children_indices[rigid_body_parent_index].push_back(rigid_body_index);
			}
			else
			{
				rigid_body_depth_first_search_stack.push_back(rigid_body_index);
			}
		}
		assert(!rigid_body_depth_first_search_stack.empty());

		mcrt_vector<bool> rigid_body_visited_flags(static_cast<size_t>(rigid_body_count), false);
		mcrt_vector<bool> rigid_body_pushed_flags(static_cast<size_t>(rigid_body_count), false);
		while (!rigid_body_depth_first_search_stack.empty())
		{
			uint32_t const rigid_body_current_index = rigid_body_depth_first_search_stack.back();
			rigid_body_depth_first_search_stack.pop_back();

			assert(!rigid_body_visited_flags[rigid_body_current_index]);
			rigid_body_visited_flags[rigid_body_current_index] = true;

			uint32_t const ragdoll_skeleton_joint_current_index = out_ragdoll_skeleton_joint_parent_indices.size();

			uint32_t const rigid_body_parent_index = rigid_body_parent_indices[rigid_body_current_index];

			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == rigid_body_parent_index)
			{
				out_ragdoll_skeleton_joint_parent_indices.push_back(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
			}
			else
			{
				assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != rigid_body_to_ragdoll_skeleton_joint_map[rigid_body_parent_index]);
				out_ragdoll_skeleton_joint_parent_indices.push_back(rigid_body_to_ragdoll_skeleton_joint_map[rigid_body_parent_index]);
			}

			assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == ragdoll_skeleton_joint_to_rigid_body_map[ragdoll_skeleton_joint_current_index]);
			ragdoll_skeleton_joint_to_rigid_body_map[ragdoll_skeleton_joint_current_index] = rigid_body_current_index;

			assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == rigid_body_to_ragdoll_skeleton_joint_map[rigid_body_current_index]);
			rigid_body_to_ragdoll_skeleton_joint_map[rigid_body_current_index] = ragdoll_skeleton_joint_current_index;

			for (uint32_t rigid_body_child_index_index_plus_1 = static_cast<uint32_t>(rigid_body_children_indices[rigid_body_current_index].size()); rigid_body_child_index_index_plus_1 > 0U; --rigid_body_child_index_index_plus_1)
			{
				uint32_t const rigid_body_child_index = rigid_body_children_indices[rigid_body_current_index][rigid_body_child_index_index_plus_1 - 1U];

				if ((!rigid_body_visited_flags[rigid_body_child_index]) && (!rigid_body_pushed_flags[rigid_body_child_index]))
				{
					rigid_body_pushed_flags[rigid_body_child_index] = true;
					rigid_body_depth_first_search_stack.push_back(rigid_body_child_index);
				}
				else
				{
					assert(false);
				}
			}
		}

		assert(out_ragdoll_skeleton_joint_parent_indices.size() == rigid_body_count);
	}

	{
		assert(out_ragdoll_rigid_bodies.empty());
		out_ragdoll_rigid_bodies = {};

		assert(out_animation_to_ragdoll_mapping.empty());
		out_animation_to_ragdoll_mapping = {};

		assert(out_ragdoll_to_animation_mapping.empty());
		out_ragdoll_to_animation_mapping = {};

		for (uint32_t ragdoll_skeleton_joint_index = 0U; ragdoll_skeleton_joint_index < rigid_body_count; ++ragdoll_skeleton_joint_index)
		{
			uint32_t const rigid_body_index = ragdoll_skeleton_joint_to_rigid_body_map[ragdoll_skeleton_joint_index];
			assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != rigid_body_index);

			mmd_pmx_rigid_body_t const &mmd_rigid_body = in_mmd_rigid_bodies[rigid_body_index];

			DirectX::XMFLOAT4 ragdoll_rotation_model_space;
			// YXZ
			// [FnRigidBody.new_rigid_body_object](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/rigid_body.py#L104)
			DirectX::XMStoreFloat4(&ragdoll_rotation_model_space, DirectX::XMQuaternionNormalize(DirectX::XMQuaternionRotationMatrix(DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationZ(mmd_rigid_body.m_rotation.m_z), DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationX(mmd_rigid_body.m_rotation.m_x), DirectX::XMMatrixRotationY(mmd_rigid_body.m_rotation.m_y))))));

			DirectX::XMFLOAT3 const ragdoll_translation_model_space(mmd_rigid_body.m_translation.m_x, mmd_rigid_body.m_translation.m_y, mmd_rigid_body.m_translation.m_z);

			static_assert(0 == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_SPHERE, "");
			static_assert(1 == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_BOX, "");
			static_assert(2 == BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_CAPSULE, "");
			out_ragdoll_rigid_bodies.push_back(
				brx_asset_import_physics_rigid_body{
					{{
						 ragdoll_rotation_model_space.x,
						 ragdoll_rotation_model_space.y,
						 ragdoll_rotation_model_space.z,
						 ragdoll_rotation_model_space.w,

					 },
					 {ragdoll_translation_model_space.x,
					  ragdoll_translation_model_space.y,
					  ragdoll_translation_model_space.z}},
					static_cast<BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_TYPE>(mmd_rigid_body.m_shape_type),
					{
						mmd_rigid_body.m_shape_size.m_x,
						mmd_rigid_body.m_shape_size.m_y,
						mmd_rigid_body.m_shape_size.m_z,
					},
					(0 == mmd_rigid_body.m_rigid_body_type) ? BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_KEYFRAME : BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_DYNAMIC,
					mmd_rigid_body.m_collision_filter_group,
					mmd_rigid_body.m_collision_filter_mask,
					mmd_rigid_body.m_mass,
					mmd_rigid_body.m_linear_damping,
					mmd_rigid_body.m_angular_damping,
					mmd_rigid_body.m_friction,
					mmd_rigid_body.m_restitution});

			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != mmd_rigid_body.m_bone_index)
			{
				uint32_t const animation_skeleton_joint_index = in_model_node_to_animation_skeleton_joint_map[mmd_rigid_body.m_bone_index];

				DirectX::XMFLOAT4X4 ragdoll_transform_model_space;
				DirectX::XMStoreFloat4x4(&ragdoll_transform_model_space, DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(&ragdoll_rotation_model_space)), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&ragdoll_translation_model_space))));

				// TODO:: remove
				{
					(*reinterpret_cast<glm::mat4 *>(&ragdoll_transform_model_space)) = saba::InvZ(*reinterpret_cast<glm::mat4 *>(&ragdoll_transform_model_space));
				}

				if (0 == mmd_rigid_body.m_rigid_body_type)
				{
					DirectX::XMFLOAT4X4 animation_to_ragdoll_transform_model_space;
					{
						DirectX::XMVECTOR unused_determinant;
						DirectX::XMStoreFloat4x4(&animation_to_ragdoll_transform_model_space, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&ragdoll_transform_model_space), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&in_animation_pose_model_space[animation_skeleton_joint_index]))));
					}
					animation_to_ragdoll_transform_model_space.m[3][3] = 1.0F;

					out_animation_to_ragdoll_mapping.push_back(
						brx_asset_import_ragdoll_direct_mapping{
							animation_skeleton_joint_index,
							ragdoll_skeleton_joint_index,
							{{
								 animation_to_ragdoll_transform_model_space.m[0][0],
								 animation_to_ragdoll_transform_model_space.m[0][1],
								 animation_to_ragdoll_transform_model_space.m[0][2],
								 animation_to_ragdoll_transform_model_space.m[0][3],
							 },
							 {
								 animation_to_ragdoll_transform_model_space.m[1][0],
								 animation_to_ragdoll_transform_model_space.m[1][1],
								 animation_to_ragdoll_transform_model_space.m[1][2],
								 animation_to_ragdoll_transform_model_space.m[1][3],
							 },
							 {
								 animation_to_ragdoll_transform_model_space.m[2][0],
								 animation_to_ragdoll_transform_model_space.m[2][1],
								 animation_to_ragdoll_transform_model_space.m[2][2],
								 animation_to_ragdoll_transform_model_space.m[2][3],
							 },
							 {
								 animation_to_ragdoll_transform_model_space.m[3][0],
								 animation_to_ragdoll_transform_model_space.m[3][1],
								 animation_to_ragdoll_transform_model_space.m[3][2],
								 animation_to_ragdoll_transform_model_space.m[3][3],
							 }}});
				}
				else
				{
					assert(1 == mmd_rigid_body.m_rigid_body_type || 2 == mmd_rigid_body.m_rigid_body_type);

					DirectX::XMFLOAT4X4 ragdoll_to_animation_transform_model_space;
					{
						DirectX::XMVECTOR unused_determinant;
						DirectX::XMStoreFloat4x4(&ragdoll_to_animation_transform_model_space, DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&in_animation_pose_model_space[animation_skeleton_joint_index]), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&ragdoll_transform_model_space))));
					}
					ragdoll_to_animation_transform_model_space.m[3][3] = 1.0F;

					out_ragdoll_to_animation_mapping.push_back(
						brx_asset_import_ragdoll_direct_mapping{
							ragdoll_skeleton_joint_index,
							animation_skeleton_joint_index,
							{{
								 ragdoll_to_animation_transform_model_space.m[0][0],
								 ragdoll_to_animation_transform_model_space.m[0][1],
								 ragdoll_to_animation_transform_model_space.m[0][2],
								 ragdoll_to_animation_transform_model_space.m[0][3],
							 },
							 {
								 ragdoll_to_animation_transform_model_space.m[1][0],
								 ragdoll_to_animation_transform_model_space.m[1][1],
								 ragdoll_to_animation_transform_model_space.m[1][2],
								 ragdoll_to_animation_transform_model_space.m[1][3],
							 },
							 {
								 ragdoll_to_animation_transform_model_space.m[2][0],
								 ragdoll_to_animation_transform_model_space.m[2][1],
								 ragdoll_to_animation_transform_model_space.m[2][2],
								 ragdoll_to_animation_transform_model_space.m[2][3],
							 },
							 {
								 ragdoll_to_animation_transform_model_space.m[3][0],
								 ragdoll_to_animation_transform_model_space.m[3][1],
								 ragdoll_to_animation_transform_model_space.m[3][2],
								 ragdoll_to_animation_transform_model_space.m[3][3],
							 }}});
				}
			}
			else
			{
				assert(0 != mmd_rigid_body.m_rigid_body_type);
				assert(1 == mmd_rigid_body.m_rigid_body_type || 2 == mmd_rigid_body.m_rigid_body_type);
			}
		}
	}

	{
		assert(out_ragdoll_constraints.empty());
		out_ragdoll_constraints = {};

		for (mmd_pmx_constraint_t const &mmd_constraint : in_mmd_constraints)
		{
			float mmd_translation_limit_min_x = std::min(mmd_constraint.m_translation_limit_min.m_x, mmd_constraint.m_translation_limit_max.m_x);
			float mmd_translation_limit_max_x = std::max(mmd_constraint.m_translation_limit_min.m_x, mmd_constraint.m_translation_limit_max.m_x);
			float mmd_translation_limit_min_y = std::min(mmd_constraint.m_translation_limit_min.m_y, mmd_constraint.m_translation_limit_max.m_y);
			float mmd_translation_limit_max_y = std::max(mmd_constraint.m_translation_limit_min.m_y, mmd_constraint.m_translation_limit_max.m_y);
			float mmd_translation_limit_min_z = std::min(mmd_constraint.m_translation_limit_min.m_z, mmd_constraint.m_translation_limit_max.m_z);
			float mmd_translation_limit_max_z = std::max(mmd_constraint.m_translation_limit_min.m_z, mmd_constraint.m_translation_limit_max.m_z);

			float mmd_rotation_limit_min_x = std::min(mmd_constraint.m_rotation_limit_min.m_x, mmd_constraint.m_rotation_limit_max.m_x);
			float mmd_rotation_limit_max_x = std::max(mmd_constraint.m_rotation_limit_min.m_x, mmd_constraint.m_rotation_limit_max.m_x);
			float mmd_rotation_limit_min_y = std::min(mmd_constraint.m_rotation_limit_min.m_y, mmd_constraint.m_rotation_limit_max.m_y);
			float mmd_rotation_limit_max_y = std::max(mmd_constraint.m_rotation_limit_min.m_y, mmd_constraint.m_rotation_limit_max.m_y);
			float mmd_rotation_limit_min_z = std::min(mmd_constraint.m_rotation_limit_min.m_z, mmd_constraint.m_rotation_limit_max.m_z);
			float mmd_rotation_limit_max_z = std::max(mmd_constraint.m_rotation_limit_min.m_z, mmd_constraint.m_rotation_limit_max.m_z);

			DirectX::XMFLOAT3 mmd_constraint_local_origin(mmd_constraint.m_translation.m_x, mmd_constraint.m_translation.m_y, mmd_constraint.m_translation.m_z);

			DirectX::XMFLOAT3 mmd_constraint_local_axis_x;
			DirectX::XMFLOAT3 mmd_constraint_local_axis_y;
			DirectX::XMFLOAT3 mmd_constraint_local_axis_z;
			{
				// YXZ
				// [FnRigidBody.new_joint_object](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/rigid_body.py#L202)
				DirectX::XMMATRIX constraint_rotation_model_space = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationZ(mmd_constraint.m_rotation.m_z), DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationX(mmd_constraint.m_rotation.m_x), DirectX::XMMatrixRotationY(mmd_constraint.m_rotation.m_y)));

				DirectX::XMFLOAT3 local_axis_x(1.0F, 0.0F, 0.0F);
				DirectX::XMStoreFloat3(&mmd_constraint_local_axis_x, DirectX::XMVector3TransformNormal(DirectX::XMLoadFloat3(&local_axis_x), constraint_rotation_model_space));

				DirectX::XMFLOAT3 local_axis_y(0.0F, 1.0F, 0.0F);
				DirectX::XMStoreFloat3(&mmd_constraint_local_axis_y, DirectX::XMVector3TransformNormal(DirectX::XMLoadFloat3(&local_axis_y), constraint_rotation_model_space));

				DirectX::XMFLOAT3 local_axis_z(0.0F, 0.0F, 1.0F);
				DirectX::XMStoreFloat3(&mmd_constraint_local_axis_z, DirectX::XMVector3TransformNormal(DirectX::XMLoadFloat3(&local_axis_z), constraint_rotation_model_space));
			}

			DirectX::XMFLOAT3 const rigid_body_a_translation(
				in_mmd_rigid_bodies[mmd_constraint.m_rigid_body_a_index].m_translation.m_x,
				in_mmd_rigid_bodies[mmd_constraint.m_rigid_body_a_index].m_translation.m_y,
				in_mmd_rigid_bodies[mmd_constraint.m_rigid_body_a_index].m_translation.m_z);

			DirectX::XMFLOAT3 const rigid_body_b_translation(
				in_mmd_rigid_bodies[mmd_constraint.m_rigid_body_b_index].m_translation.m_x,
				in_mmd_rigid_bodies[mmd_constraint.m_rigid_body_b_index].m_translation.m_y,
				in_mmd_rigid_bodies[mmd_constraint.m_rigid_body_b_index].m_translation.m_z);

			BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_TYPE brx_constraint_type;
			DirectX::XMFLOAT3 brx_pivot;
			DirectX::XMFLOAT3 brx_twist_axis;
			DirectX::XMFLOAT3 brx_plane_axis;
			DirectX::XMFLOAT3 brx_normal_axis;
			float brx_twist_limit[2];
			float brx_plane_limit[2];
			float brx_normal_limit[2];
			{
				brx_pivot = mmd_constraint_local_origin;

				constexpr float const INTERNAL_EPSILON = 1E-6F;
				constexpr float const INTERNAL_PI = DirectX::XM_PI;
				constexpr float const INTERNAL_NEAR_PI_DIV_2 = DirectX::XM_PIDIV2 - INTERNAL_EPSILON;

				float mmd_translation_limit_abs_x = std::max(std::abs(mmd_translation_limit_min_x), std::abs(mmd_translation_limit_max_x));
				float mmd_translation_limit_abs_y = std::max(std::abs(mmd_translation_limit_min_y), std::abs(mmd_translation_limit_max_y));
				float mmd_translation_limit_abs_z = std::max(std::abs(mmd_translation_limit_min_z), std::abs(mmd_translation_limit_max_z));

				float mmd_rotation_limit_abs_x = std::max(std::abs(mmd_rotation_limit_min_x), std::abs(mmd_rotation_limit_max_x));
				float mmd_rotation_limit_abs_y = std::max(std::abs(mmd_rotation_limit_min_y), std::abs(mmd_rotation_limit_max_y));
				float mmd_rotation_limit_abs_z = std::max(std::abs(mmd_rotation_limit_min_z), std::abs(mmd_rotation_limit_max_z));

				if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_x <= INTERNAL_EPSILON && mmd_translation_limit_abs_y <= INTERNAL_EPSILON && mmd_translation_limit_abs_z <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_FIXED;

					brx_twist_axis = mmd_constraint_local_axis_x;

					brx_plane_axis = mmd_constraint_local_axis_y;

					brx_normal_axis = mmd_constraint_local_axis_z;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = 0.0F;
					brx_normal_limit[1] = 0.0F;
				}
				else if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_x <= INTERNAL_EPSILON && mmd_translation_limit_abs_y <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_PRISMATIC;

					brx_twist_axis = mmd_constraint_local_axis_x;

					brx_plane_axis = mmd_constraint_local_axis_y;

					brx_normal_axis = mmd_constraint_local_axis_z;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = mmd_translation_limit_min_z;
					brx_normal_limit[1] = mmd_translation_limit_max_z;
				}
				else if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_y <= INTERNAL_EPSILON && mmd_translation_limit_abs_z <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_PRISMATIC;

					brx_twist_axis = mmd_constraint_local_axis_y;

					brx_plane_axis = mmd_constraint_local_axis_z;

					brx_normal_axis = mmd_constraint_local_axis_x;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = mmd_translation_limit_min_x;
					brx_normal_limit[1] = mmd_translation_limit_max_x;
				}
				else if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_x <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_PRISMATIC;

					brx_twist_axis = mmd_constraint_local_axis_z;

					brx_plane_axis = mmd_constraint_local_axis_x;

					brx_normal_axis = mmd_constraint_local_axis_y;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = mmd_translation_limit_min_y;
					brx_normal_limit[1] = mmd_translation_limit_max_y;
				}
				else
				{
					// convert from translation to rotation
					if (mmd_translation_limit_abs_x >= INTERNAL_EPSILON || mmd_translation_limit_abs_y >= INTERNAL_EPSILON || mmd_translation_limit_abs_z >= INTERNAL_EPSILON)
					{
						assert((mmd_rotation_limit_abs_x >= INTERNAL_EPSILON || mmd_rotation_limit_abs_y >= INTERNAL_EPSILON || mmd_rotation_limit_abs_z >= INTERNAL_EPSILON) || (mmd_translation_limit_abs_x >= INTERNAL_EPSILON && mmd_translation_limit_abs_y >= INTERNAL_EPSILON && mmd_translation_limit_abs_z >= INTERNAL_EPSILON));

						float rigid_body_b_body_space_translation_length = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&rigid_body_b_translation), DirectX::XMLoadFloat3(&mmd_constraint_local_origin))));

						if (std::abs(mmd_rotation_limit_abs_x - mmd_rotation_limit_abs_y) <= INTERNAL_EPSILON && std::abs(mmd_rotation_limit_abs_y - mmd_rotation_limit_abs_z) <= INTERNAL_EPSILON && std::abs(mmd_rotation_limit_abs_z - mmd_rotation_limit_abs_x) <= INTERNAL_EPSILON)
						{
							if (mmd_translation_limit_abs_x <= mmd_translation_limit_abs_y && mmd_translation_limit_abs_x <= mmd_translation_limit_abs_z)
							{
								mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
								mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));

								mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
								mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
							}
							else if (mmd_translation_limit_abs_y <= mmd_translation_limit_abs_z && mmd_translation_limit_abs_y <= mmd_translation_limit_abs_x)
							{
								mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
								mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

								mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
								mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
							}
							else
							{
								assert(mmd_translation_limit_abs_z <= mmd_translation_limit_abs_x && mmd_translation_limit_abs_z <= mmd_translation_limit_abs_y);

								mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));
								mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

								mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
								mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
							}
						}
						else if (mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_y && mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_z)
						{
							mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));

							mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
						}
						else if (mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_z && mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_x)
						{
							mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

							mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
						}
						else
						{
							assert(mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_x && mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_y);

							mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

							mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
						}

						mmd_rotation_limit_abs_x = std::max(std::abs(mmd_rotation_limit_min_x), std::abs(mmd_rotation_limit_max_x));
						mmd_rotation_limit_abs_y = std::max(std::abs(mmd_rotation_limit_min_y), std::abs(mmd_rotation_limit_max_y));
						mmd_rotation_limit_abs_z = std::max(std::abs(mmd_rotation_limit_min_z), std::abs(mmd_rotation_limit_max_z));

						mmd_translation_limit_min_x = 0.0F;
						mmd_translation_limit_max_x = 0.0F;
						mmd_translation_limit_min_y = 0.0F;
						mmd_translation_limit_max_y = 0.0F;
						mmd_translation_limit_min_z = 0.0F;
						mmd_translation_limit_max_z = 0.0F;

						mmd_translation_limit_abs_x = std::max(std::abs(mmd_translation_limit_min_x), std::abs(mmd_translation_limit_max_x));
						mmd_translation_limit_abs_y = std::max(std::abs(mmd_translation_limit_min_y), std::abs(mmd_translation_limit_max_y));
						mmd_translation_limit_abs_z = std::max(std::abs(mmd_translation_limit_min_z), std::abs(mmd_translation_limit_max_z));
					}

					assert(mmd_rotation_limit_abs_x >= INTERNAL_EPSILON || mmd_rotation_limit_abs_y >= INTERNAL_EPSILON || mmd_rotation_limit_abs_z >= INTERNAL_EPSILON);

					if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON)
					{
						brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_HINGE;

						brx_twist_axis = mmd_constraint_local_axis_x;

						brx_plane_axis = mmd_constraint_local_axis_y;

						brx_normal_axis = mmd_constraint_local_axis_z;

						brx_twist_limit[0] = 0.0F;
						brx_twist_limit[1] = 0.0F;

						brx_plane_limit[0] = 0.0F;
						brx_plane_limit[1] = 0.0F;

						brx_normal_limit[0] = mmd_rotation_limit_min_z;
						brx_normal_limit[1] = mmd_rotation_limit_max_z;
					}
					else if (mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON)
					{
						brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_HINGE;

						brx_twist_axis = mmd_constraint_local_axis_y;

						brx_plane_axis = mmd_constraint_local_axis_z;

						brx_normal_axis = mmd_constraint_local_axis_x;

						brx_twist_limit[0] = 0.0F;
						brx_twist_limit[1] = 0.0F;

						brx_plane_limit[0] = 0.0F;
						brx_plane_limit[1] = 0.0F;

						brx_normal_limit[0] = mmd_rotation_limit_min_x;
						brx_normal_limit[1] = mmd_rotation_limit_max_x;
					}
					else if (mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_rotation_limit_abs_x <= INTERNAL_EPSILON)
					{
						brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_HINGE;

						brx_twist_axis = mmd_constraint_local_axis_z;

						brx_plane_axis = mmd_constraint_local_axis_x;

						brx_normal_axis = mmd_constraint_local_axis_y;

						brx_twist_limit[0] = 0.0F;
						brx_twist_limit[1] = 0.0F;

						brx_plane_limit[0] = 0.0F;
						brx_plane_limit[1] = 0.0F;

						brx_normal_limit[0] = mmd_rotation_limit_min_y;
						brx_normal_limit[1] = mmd_rotation_limit_max_y;
					}
					else if ((std::abs(mmd_rotation_limit_abs_x) >= INTERNAL_NEAR_PI_DIV_2 && std::abs(mmd_rotation_limit_abs_y) >= INTERNAL_NEAR_PI_DIV_2) || (std::abs(mmd_rotation_limit_abs_y) >= INTERNAL_NEAR_PI_DIV_2 && std::abs(mmd_rotation_limit_abs_z) >= INTERNAL_NEAR_PI_DIV_2) || (std::abs(mmd_rotation_limit_abs_z) >= INTERNAL_NEAR_PI_DIV_2 && std::abs(mmd_rotation_limit_abs_x) >= INTERNAL_NEAR_PI_DIV_2))
					{
						brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_BALL_AND_SOCKET;

						brx_twist_axis = mmd_constraint_local_axis_x;

						brx_plane_axis = mmd_constraint_local_axis_y;

						brx_normal_axis = mmd_constraint_local_axis_z;

						brx_twist_limit[0] = 0.0F;
						brx_twist_limit[1] = 0.0F;

						brx_plane_limit[0] = 0.0F;
						brx_plane_limit[1] = 0.0F;

						brx_normal_limit[0] = 0.0F;
						brx_normal_limit[1] = 0.0F;
					}
					else
					{
						brx_constraint_type = BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_RAGDOLL;

						if (mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_y && mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_z)
						{
							brx_twist_axis = mmd_constraint_local_axis_x;

							brx_plane_axis = mmd_constraint_local_axis_y;

							brx_normal_axis = mmd_constraint_local_axis_z;

							brx_twist_limit[0] = mmd_rotation_limit_min_x;
							brx_twist_limit[1] = mmd_rotation_limit_max_x;

							brx_plane_limit[0] = mmd_rotation_limit_min_y;
							brx_plane_limit[1] = mmd_rotation_limit_max_y;

							brx_normal_limit[0] = mmd_rotation_limit_min_z;
							brx_normal_limit[1] = mmd_rotation_limit_max_z;
						}
						else if (mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_z && mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_x)
						{

							brx_twist_axis = mmd_constraint_local_axis_y;

							brx_plane_axis = mmd_constraint_local_axis_z;

							brx_normal_axis = mmd_constraint_local_axis_x;

							brx_twist_limit[0] = mmd_rotation_limit_min_y;
							brx_twist_limit[1] = mmd_rotation_limit_max_y;

							brx_plane_limit[0] = mmd_rotation_limit_min_z;
							brx_plane_limit[1] = mmd_rotation_limit_max_z;

							brx_normal_limit[0] = mmd_rotation_limit_min_x;
							brx_normal_limit[1] = mmd_rotation_limit_max_x;
						}
						else
						{
							assert(mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_x && mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_y);

							brx_twist_axis = mmd_constraint_local_axis_z;

							brx_plane_axis = mmd_constraint_local_axis_x;

							brx_normal_axis = mmd_constraint_local_axis_y;

							brx_twist_limit[0] = mmd_rotation_limit_min_z;
							brx_twist_limit[1] = mmd_rotation_limit_max_z;

							brx_plane_limit[0] = mmd_rotation_limit_min_x;
							brx_plane_limit[1] = mmd_rotation_limit_max_x;

							brx_normal_limit[0] = mmd_rotation_limit_min_y;
							brx_normal_limit[1] = mmd_rotation_limit_max_y;
						}

						if (DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMLoadFloat3(&brx_twist_axis), DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&rigid_body_b_translation), DirectX::XMLoadFloat3(&rigid_body_a_translation)))) < (-INTERNAL_EPSILON))
						{
							brx_twist_axis.x = 0.0F - brx_twist_axis.x;
							brx_twist_axis.y = 0.0F - brx_twist_axis.y;
							brx_twist_axis.z = 0.0F - brx_twist_axis.z;

							DirectX::XMFLOAT3 temp_plane_axis = brx_plane_axis;

							brx_plane_axis.x = 0.0F - brx_normal_axis.x;
							brx_plane_axis.y = 0.0F - brx_normal_axis.y;
							brx_plane_axis.z = 0.0F - brx_normal_axis.z;

							brx_normal_axis.x = 0.0F - temp_plane_axis.x;
							brx_normal_axis.y = 0.0F - temp_plane_axis.y;
							brx_normal_axis.z = 0.0F - temp_plane_axis.z;

							float temp_plane_limit[2] = {brx_plane_limit[0], brx_plane_limit[1]};

							brx_plane_limit[0] = brx_normal_limit[0];
							brx_plane_limit[1] = brx_normal_limit[1];

							brx_normal_limit[0] = temp_plane_limit[0];
							brx_normal_limit[1] = temp_plane_limit[1];
						}

						// Bullet Cone-Twist Constraint
						// https://help.autodesk.com/view/MAYAUL/2024/ENU/?guid=GUID-CDB3638D-23AF-49EF-8EF6-53081EE4D39D
						// twist_span
						// swing_span2 (plane)
						// swing_span1 (normal)

						// float const twist_span = std::min(std::max(std::abs(brx_twist_limit[0]), std::abs(brx_twist_limit[1])), (INTERNAL_PI * 1.0F));
						// float const swing_span2 = std::min(std::max(std::abs(brx_plane_limit[0]), std::abs(brx_plane_limit[1])), (INTERNAL_PI * 0.5F));
						// float const swing_span1 = std::min(std::max(std::abs(brx_normal_limit[0]), std::abs(brx_normal_limit[1])), (INTERNAL_PI * 0.5F));

						// brx_twist_limit[0] = 0.0F - twist_span;
						// brx_twist_limit[1] = twist_span;

						// brx_plane_limit[0] = 0.0F - swing_span2;
						// brx_plane_limit[1] = swing_span2;

						// brx_normal_limit[0] = 0.0F - swing_span1;
						// brx_normal_limit[1] = swing_span1;
					}
				}
			}

			uint32_t const skeleton_joint_index_a = rigid_body_to_ragdoll_skeleton_joint_map[mmd_constraint.m_rigid_body_a_index];
			assert(ragdoll_skeleton_joint_to_rigid_body_map[skeleton_joint_index_a] == mmd_constraint.m_rigid_body_a_index);

			uint32_t const skeleton_joint_index_b = rigid_body_to_ragdoll_skeleton_joint_map[mmd_constraint.m_rigid_body_b_index];
			assert(ragdoll_skeleton_joint_to_rigid_body_map[skeleton_joint_index_b] == mmd_constraint.m_rigid_body_b_index);

			out_ragdoll_constraints.push_back(
				brx_asset_import_physics_constraint{
					skeleton_joint_index_a,
					skeleton_joint_index_b,
					brx_constraint_type,
					{
						brx_pivot.x,
						brx_pivot.y,
						brx_pivot.z,
					},
					{
						brx_twist_axis.x,
						brx_twist_axis.y,
						brx_twist_axis.z,
					},
					{
						brx_plane_axis.x,
						brx_plane_axis.y,
						brx_plane_axis.z,
					},
					{
						brx_normal_axis.x,
						brx_normal_axis.y,
						brx_normal_axis.z,
					},
					{
						brx_twist_limit[0],
						brx_twist_limit[1],
					},
					{
						brx_plane_limit[0],
						brx_plane_limit[1],
					},
					{
						brx_normal_limit[0],
						brx_normal_limit[1],
					}});
		}
	}
}

#if defined(__GNUC__)
#error 1
#elif defined(_MSC_VER)
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
