//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#define GLM_ENABLE_EXPERIMENTAL

#include "PMXModel.h"

#include "PMXFile.h"
#include "MMDPhysics.h"

#include <Saba/Base/Path.h>
#include <Saba/Base/File.h>
#include <Saba/Base/Log.h>
#include <Saba/Base/Singleton.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/dual_quaternion.hpp>
#include <map>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
// #include <condition_variable>

static inline void *_internal_dynamic_link_open(wchar_t const *filename);
static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol);

#ifndef NDEBUG
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/McRT-Malloc");
void *const libjpeg_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libjpeg");
void *const libpng_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libpng");
void *const libwebp_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libwebp");
void *const libiconv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libiconv");
void *const opencv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/opencv_world3410");
void *const asset_import_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Asset-Import");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/McRT-Malloc");
void *const libjpeg_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libjpeg");
void *const libpng_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libpng");
void *const libwebp_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libwebp");
void *const libiconv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libiconv");
void *const opencv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/opencv_world3410");
void *const asset_import_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Asset-Import");
#endif

decltype(brx_asset_import_create_file_input_stream_factory) *const asset_import_create_file_input_stream_factory = reinterpret_cast<decltype(brx_asset_import_create_file_input_stream_factory) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_create_file_input_stream_factory"));
decltype(brx_asset_import_destroy_file_input_stream_factory) *const asset_import_destroy_file_input_stream_factory = reinterpret_cast<decltype(brx_asset_import_destroy_file_input_stream_factory) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_destroy_file_input_stream_factory"));
decltype(brx_asset_import_create_scene_from_input_stream) *const asset_import_create_scene_from_input_stream = reinterpret_cast<decltype(brx_asset_import_create_scene_from_input_stream) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_create_scene_from_input_stream"));
decltype(brx_asset_import_destory_scene) *const asset_import_destory_scene = reinterpret_cast<decltype(brx_asset_import_destory_scene) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_destory_scene"));

namespace saba
{
	PMXModel::PMXModel() : m_parallelUpdateCount(0)
	{
	}

	PMXModel::~PMXModel()
	{
		Destroy();
	}

	void PMXModel::InitializeAnimation()
	{
		ClearBaseAnimation();

		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_morph_target_name_weights[mmd_morph_target_name] = 0.0F;
		}

		for (uint32_t mmd_skeleton_joint_name = 0U; mmd_skeleton_joint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_name)
		{
			this->m_skeleton_joint_name_rigid_transforms[mmd_skeleton_joint_name] = brx_asset_import_rigid_transform{{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}};
		}

		for (uint32_t mmd_skeleton_joint_constraint_name = 0U; mmd_skeleton_joint_constraint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_constraint_name)
		{
			this->m_skeleton_joint_constraint_name_switches[mmd_skeleton_joint_constraint_name] = true;
		}
	}

	void PMXModel::SaveBaseAnimation()
	{
		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_saved_morph_target_name_weights[mmd_morph_target_name] = this->m_morph_target_name_weights[mmd_morph_target_name];
		}

		for (uint32_t mmd_skeleton_joint_name = 0U; mmd_skeleton_joint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_name)
		{
			this->m_saved_skeleton_joint_name_rigid_transforms[mmd_skeleton_joint_name] = this->m_skeleton_joint_name_rigid_transforms[mmd_skeleton_joint_name];
		}

		for (uint32_t mmd_skeleton_joint_constraint_name = 0U; mmd_skeleton_joint_constraint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_constraint_name)
		{
			this->m_saved_skeleton_joint_constraint_name_switches[mmd_skeleton_joint_constraint_name] = this->m_skeleton_joint_constraint_name_switches[mmd_skeleton_joint_constraint_name];
		}
	}

	void PMXModel::LoadBaseAnimation()
	{
		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_morph_target_name_weights[mmd_morph_target_name] = this->m_saved_morph_target_name_weights[mmd_morph_target_name];
		}

		for (uint32_t mmd_skeleton_joint_name = 0U; mmd_skeleton_joint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_name)
		{
			this->m_skeleton_joint_name_rigid_transforms[mmd_skeleton_joint_name] = this->m_saved_skeleton_joint_name_rigid_transforms[mmd_skeleton_joint_name];
		}

		for (uint32_t mmd_skeleton_joint_constraint_name = 0U; mmd_skeleton_joint_constraint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_constraint_name)
		{
			this->m_skeleton_joint_constraint_name_switches[mmd_skeleton_joint_constraint_name] = this->m_saved_skeleton_joint_constraint_name_switches[mmd_skeleton_joint_constraint_name];
		}
	}

	void PMXModel::ClearBaseAnimation()
	{
		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_saved_morph_target_name_weights[mmd_morph_target_name] = 0.0F;
		}

		for (uint32_t mmd_skeleton_joint_name = 0U; mmd_skeleton_joint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_name)
		{
			this->m_saved_skeleton_joint_name_rigid_transforms[mmd_skeleton_joint_name] = brx_asset_import_rigid_transform{{0.0F, 0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}};
		}

		for (uint32_t mmd_skeleton_joint_constraint_name = 0U; mmd_skeleton_joint_constraint_name < BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_COUNT; ++mmd_skeleton_joint_constraint_name)
		{
			this->m_saved_skeleton_joint_constraint_name_switches[mmd_skeleton_joint_constraint_name] = true;
		}
	}

	void PMXModel::UpdateMorphAnimation()
	{
		uint32_t const vertex_count = m_morphPositions.size();
		for (uint32_t vtxIdx = 0; vtxIdx < vertex_count; ++vtxIdx)
		{
			m_morphPositions[vtxIdx] = glm::vec3(0);
			m_morphUVs[vtxIdx] = glm::vec4(0);
		}

		constexpr float const INTERNAL_WEIGHT_EPSILON = 1E-6F;

		uint32_t const morph_target_count = this->m_morph_target_names.size();
		assert(this->m_morph_targets_vertex_positions.size() == morph_target_count);
		assert(this->m_morph_targets_vertex_uvs.size() == morph_target_count);

		for (uint32_t morph_target_index = 0U; morph_target_index < morph_target_count; ++morph_target_index)
		{
			BRX_ASSET_IMPORT_MORPH_TARGET_NAME const morph_target_name = this->m_morph_target_names[morph_target_index];

			float const morph_target_weight = this->m_morph_target_name_weights[morph_target_name];

			if (morph_target_weight > INTERNAL_WEIGHT_EPSILON)
			{
				mcrt_vector<glm::vec3> const &morph_target_vertex_positions = this->m_morph_targets_vertex_positions[morph_target_index];
				mcrt_vector<glm::vec2> const &morph_target_vertex_uvs = this->m_morph_targets_vertex_uvs[morph_target_index];
				assert(!morph_target_vertex_positions.empty());
				assert(!morph_target_vertex_uvs.empty());

				// NOTE: "vertex_index" valid
				// morph targets are arranged at the beginning of the vertices

				uint32_t const vertex_count = morph_target_vertex_positions.size();
				assert(morph_target_vertex_uvs.size() == vertex_count);

				for (uint32_t vertex_index = 0U; vertex_index < vertex_count; ++vertex_index)
				{
					glm::vec3 const morph_target_vertex_position = morph_target_vertex_positions[vertex_index];
					glm::vec2 const morph_target_vertex_uv = morph_target_vertex_uvs[vertex_index];

					m_morphPositions[vertex_index].x += (morph_target_vertex_position.x * morph_target_weight);
					m_morphPositions[vertex_index].y += (morph_target_vertex_position.y * morph_target_weight);
					m_morphPositions[vertex_index].z += (morph_target_vertex_position.z * morph_target_weight);

					m_morphUVs[vertex_index].x += (morph_target_vertex_uv.x * morph_target_weight);
					m_morphUVs[vertex_index].y += (morph_target_vertex_uv.y * morph_target_weight);
				}
			}
		}
	}

	static inline glm::mat4x4 calculate_transform_model_space(uint32_t const *const in_animation_skeleton_joint_parent_indices, glm::mat4x4 const *const in_animation_skeleton_local_space, uint32_t const in_animation_skeleton_joint_index)
	{
		uint32_t current_animation_skeleton_joint_index = in_animation_skeleton_joint_index;

		mcrt_vector<uint32_t> ancestors;
		while (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != current_animation_skeleton_joint_index)
		{
			ancestors.push_back(current_animation_skeleton_joint_index);
			current_animation_skeleton_joint_index = in_animation_skeleton_joint_parent_indices[current_animation_skeleton_joint_index];
		}

		assert(!ancestors.empty());
		glm::mat4 model_space = in_animation_skeleton_local_space[ancestors.back()];
		ancestors.pop_back();

		while (!ancestors.empty())
		{
			model_space = model_space * in_animation_skeleton_local_space[ancestors.back()];
			ancestors.pop_back();
		}

		return model_space;
	}

	void PMXModel::UpdateNodeAnimation(bool enablePhysics, float elapsed)
	{
		assert(this->m_bind_pose_local_space.size() == this->m_bind_pose_inverse_model_space.size());
		uint32_t const animation_skeleton_joint_count = this->m_bind_pose_local_space.size();

		mcrt_vector<glm::mat4x4> animation_skeleton_bind_pose_local_space(static_cast<size_t>(animation_skeleton_joint_count));
		for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
		{
			glm::quat const rotation(this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[3], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[0], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[1], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[2]);

			glm::vec3 const translation(this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[0], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[1], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[2]);

			animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index] = glm::translate(glm::mat4(1), translation) * glm::mat4_cast(rotation);
		}

		mcrt_vector<glm::mat4x4> animation_skeleton_animation_pose_local_space(static_cast<size_t>(animation_skeleton_joint_count));
		for (size_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
		{
			BRX_ASSET_IMPORT_SKELETON_JOINT_NAME const skeleton_joint_name = this->m_skeleton_joint_names[animation_skeleton_joint_index];

			glm::quat rotation;
			glm::vec3 translation;
			if (BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_INVALID != skeleton_joint_name)
			{
				// [Normalized Local Rotation](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm_animation-1.0/how_to_transform_human_pose.md)
				glm::quat const local_rotation(this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_rotation[3], this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_rotation[0], this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_rotation[1], this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_rotation[2]);
				glm::vec3 const local_translation(this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_translation[0], this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_translation[1], this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name].m_translation[2]);

				rotation = local_rotation * glm::quat(this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[3], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[0], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[1], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[2]);

				translation = local_translation + glm::vec3(this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[0], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[1], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[2]);
			}
			else
			{
				rotation = glm::quat(this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[3], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[0], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[1], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_rotation[2]);

				translation = glm::vec3(this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[0], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[1], this->m_bind_pose_local_space[animation_skeleton_joint_index].m_translation[2]);
			}

			animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index] = glm::translate(glm::mat4(1), translation) * glm::mat4_cast(rotation);
		}

		uint32_t const animation_skeleton_joint_constraint_count = this->m_animation_skeleton_joint_constraint_names.size();
		assert(this->m_animation_skeleton_joint_constraints.size() == animation_skeleton_joint_constraint_count);
		assert(this->m_animation_skeleton_joint_constraints_storage.size() == animation_skeleton_joint_constraint_count);

		for (uint32_t animation_skeleton_joint_constraint_index = 0U; animation_skeleton_joint_constraint_index < animation_skeleton_joint_constraint_count; ++animation_skeleton_joint_constraint_index)
		{
			BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME const animation_skeleton_joint_constraint_name = this->m_animation_skeleton_joint_constraint_names[animation_skeleton_joint_constraint_index];

			brx_asset_import_skeleton_joint_constraint const &animation_skeleton_joint_constraint = this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index];

			if ((BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_INVALID == animation_skeleton_joint_constraint_name) || this->m_skeleton_joint_constraint_name_switches[animation_skeleton_joint_constraint_name])
			{
				if (BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_COPY_TRANSFORM == animation_skeleton_joint_constraint.m_constraint_type)
				{
					glm::quat source_rotation_local_space;
					glm::vec3 source_translation_local_space;
					{
						glm::quat source_rotation_bind_pose_local_space;
						glm::vec3 source_translation_bind_pose_local_space;
						{
							glm::mat4x4 source_transform_bind_pose_local_space = animation_skeleton_bind_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_source_joint_index];

							source_translation_bind_pose_local_space = glm::vec3(source_transform_bind_pose_local_space[3]);

							glm::vec3 scale = glm::vec3(
								glm::length(glm::vec3(source_transform_bind_pose_local_space[0])),
								glm::length(glm::vec3(source_transform_bind_pose_local_space[1])),
								glm::length(glm::vec3(source_transform_bind_pose_local_space[2])));
							assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

							source_rotation_bind_pose_local_space = glm::quat_cast(glm::mat3(
								glm::vec3(source_transform_bind_pose_local_space[0]) / scale.x,
								glm::vec3(source_transform_bind_pose_local_space[1]) / scale.y,
								glm::vec3(source_transform_bind_pose_local_space[2]) / scale.z));
						}

						glm::quat source_rotation_animation_pose_local_space;
						glm::vec3 source_translation_animation_pose_local_space;
						{
							glm::mat4x4 source_transform_animated_local_space = animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_source_joint_index];

							source_translation_animation_pose_local_space = glm::vec3(source_transform_animated_local_space[3]);

							glm::vec3 scale = glm::vec3(
								glm::length(glm::vec3(source_transform_animated_local_space[0])),
								glm::length(glm::vec3(source_transform_animated_local_space[1])),
								glm::length(glm::vec3(source_transform_animated_local_space[2])));
							assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

							source_rotation_animation_pose_local_space = glm::quat_cast(glm::mat3(
								glm::vec3(source_transform_animated_local_space[0]) / scale.x,
								glm::vec3(source_transform_animated_local_space[1]) / scale.y,
								glm::vec3(source_transform_animated_local_space[2]) / scale.z));
						}

						// bind pose rotation always zero
						assert(glm::all(glm::epsilonEqual(source_rotation_bind_pose_local_space, glm::quat(1.0F, 0.0F, 0.0F, 0.0F), 1E-6F)));
						source_rotation_local_space = source_rotation_animation_pose_local_space;

						source_translation_local_space = source_translation_animation_pose_local_space - source_translation_bind_pose_local_space;
					}

					glm::quat destination_rotation_local_space;
					glm::vec3 destination_translation_local_space;
					{
						glm::mat4x4 destination_transform_local_space = animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_destination_joint_index];

						destination_translation_local_space = glm::vec3(destination_transform_local_space[3]);

						glm::vec3 scale = glm::vec3(
							glm::length(glm::vec3(destination_transform_local_space[0])),
							glm::length(glm::vec3(destination_transform_local_space[1])),
							glm::length(glm::vec3(destination_transform_local_space[2])));
						assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

						destination_rotation_local_space = glm::quat_cast(glm::mat3(
							glm::vec3(destination_transform_local_space[0]) / scale.x,
							glm::vec3(destination_transform_local_space[1]) / scale.y,
							glm::vec3(destination_transform_local_space[2]) / scale.z));
					}

					if (animation_skeleton_joint_constraint.m_copy_transform.m_copy_rotation)
					{
						glm::quat append_rotation = source_rotation_local_space;

						for (uint32_t source_weight_index = 0U; source_weight_index < animation_skeleton_joint_constraint.m_copy_transform.m_source_weight_count; ++source_weight_index)
						{
							append_rotation = glm::slerp(glm::quat(1.0F, 0.0F, 0.0F, 0.0F), append_rotation, animation_skeleton_joint_constraint.m_copy_transform.m_source_weights[source_weight_index]);
						}

						destination_rotation_local_space = (destination_rotation_local_space * append_rotation);
					}

					if (animation_skeleton_joint_constraint.m_copy_transform.m_copy_translation)
					{
						glm::vec3 append_translation = source_translation_local_space;

						for (uint32_t source_weight_index = 0U; source_weight_index < animation_skeleton_joint_constraint.m_copy_transform.m_source_weight_count; ++source_weight_index)
						{
							append_translation = append_translation * animation_skeleton_joint_constraint.m_copy_transform.m_source_weights[source_weight_index];
						}

						destination_translation_local_space = (destination_translation_local_space + append_translation);
					}

					glm::mat4 destination_transform_local_space = glm::translate(glm::mat4(1), destination_translation_local_space) * glm::mat4_cast(destination_rotation_local_space);

					animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_destination_joint_index] = destination_transform_local_space;
				}
				else
				{
					assert(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_INVERSE_KINEMATICS == animation_skeleton_joint_constraint.m_constraint_type);

					glm::vec3 const target_position_model_space = calculate_transform_model_space(this->m_animation_skeleton_joint_parent_indices.data(), animation_skeleton_animation_pose_local_space.data(), animation_skeleton_joint_constraint.m_inverse_kinematics.m_target_joint_index)[3];

					glm::mat4 const end_effector_transform_local_space = animation_skeleton_animation_pose_local_space[animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_end_effector_index];

					uint32_t const ik_joint_count = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_count;

					std::vector<glm::mat4x4> ik_joints_local_space(static_cast<size_t>(ik_joint_count));
					std::vector<glm::mat4x4> ik_joints_model_space(static_cast<size_t>(ik_joint_count));

					// TODO: check parent index consistent
					// TODO: check ik joint index out of bound

					for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
					{
						uint32_t const ik_joint_animation_skeleton_joint_index = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_indices[ik_joint_index];

						ik_joints_local_space[ik_joint_index] = animation_skeleton_animation_pose_local_space[ik_joint_animation_skeleton_joint_index];

						ik_joints_model_space[ik_joint_index] = calculate_transform_model_space(this->m_animation_skeleton_joint_parent_indices.data(), animation_skeleton_animation_pose_local_space.data(), ik_joint_animation_skeleton_joint_index);
					}

					glm::vec3 const two_joints_hinge_joint_axis_local_space(animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[0], animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[1], animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[2]);
					float const two_joints_cosine_max_hinge_joint_angle = animation_skeleton_joint_constraint.m_inverse_kinematics.m_cosine_max_ik_two_joints_hinge_joint_angle;
					float const two_joints_cosine_min_hinge_joint_angle = animation_skeleton_joint_constraint.m_inverse_kinematics.m_cosine_min_ik_two_joints_hinge_joint_angle;
					MMDIkSolver::Solve(two_joints_hinge_joint_axis_local_space, two_joints_cosine_max_hinge_joint_angle, two_joints_cosine_min_hinge_joint_angle, target_position_model_space, end_effector_transform_local_space, ik_joint_count, ik_joints_local_space.data(), ik_joints_model_space.data());

					for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
					{
						uint32_t const animation_skeleton_joint_index = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_indices[ik_joint_index];

						animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index] = ik_joints_local_space[ik_joint_index];
					}
				}
			}
		}

		mcrt_vector<glm::mat4x4> animation_skeleton_animation_pose_model_space(static_cast<size_t>(animation_skeleton_joint_count));
		for (uint32_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < static_cast<uint32_t>(animation_skeleton_joint_count); ++current_animation_skeleton_joint_index)
		{
			uint32_t const parent_animation_skeleton_joint_index = this->m_animation_skeleton_joint_parent_indices[current_animation_skeleton_joint_index];
			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
			{
				assert(parent_animation_skeleton_joint_index < current_animation_skeleton_joint_index);
				animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index] = animation_skeleton_animation_pose_model_space[parent_animation_skeleton_joint_index] * animation_skeleton_animation_pose_local_space[current_animation_skeleton_joint_index];
			}
			else
			{
				animation_skeleton_animation_pose_model_space[current_animation_skeleton_joint_index] = animation_skeleton_animation_pose_local_space[current_animation_skeleton_joint_index];
			}
		}

		if (enablePhysics)
		{
			MMDPhysics *physics = this->GetPhysicsManager()->GetMMDPhysics();

			physics->AnimationToRagdoll(animation_skeleton_animation_pose_model_space.data());

			physics->Update(elapsed);

			// TODO: we do not need to calculate the model space of the joints which are retrived from the physics
			physics->RagdollToAnimation(animation_skeleton_animation_pose_model_space.data());
		}

		assert(this->m_transforms.size() == animation_skeleton_joint_count);
		for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < static_cast<uint32_t>(animation_skeleton_joint_count); ++animation_skeleton_joint_index)
		{
			this->m_transforms[animation_skeleton_joint_index] = animation_skeleton_animation_pose_model_space[animation_skeleton_joint_index] * this->m_bind_pose_inverse_model_space[animation_skeleton_joint_index];
		}
	}

	void PMXModel::ResetPhysics()
	{
		assert(false);
	}

	void PMXModel::Update()
	{
		if (m_parallelUpdateCount != m_updateRanges.size())
		{
			SetupParallelUpdate();
		}

		size_t futureCount = m_parallelUpdateFutures.size();
		for (size_t i = 0; i < futureCount; i++)
		{
			size_t rangeIndex = i + 1;
			if (m_updateRanges[rangeIndex].m_vertexCount != 0)
			{
				m_parallelUpdateFutures[i] = std::async(
					std::launch::async,
					[this, rangeIndex]()
					{ this->Update(this->m_updateRanges[rangeIndex]); });
			}
		}

		Update(m_updateRanges[0]);

		for (size_t i = 0; i < futureCount; i++)
		{
			size_t rangeIndex = i + 1;
			if (m_updateRanges[rangeIndex].m_vertexCount != 0)
			{
				m_parallelUpdateFutures[i].wait();
			}
		}
	}

	void PMXModel::SetParallelUpdateHint(uint32_t parallelCount)
	{
		m_parallelUpdateCount = parallelCount;
	}

	bool PMXModel::Load(const std::string &filepath, const std::string &mmdDataDir)
	{
		Destroy();

		brx_asset_import_input_stream_factory *const input_stream_factory = asset_import_create_file_input_stream_factory();

		brx_asset_import_scene *const scene = asset_import_create_scene_from_input_stream(input_stream_factory, filepath.c_str());

		assert(1U == scene->get_surface_group_count());
		brx_asset_import_surface_group const *const surface_group = scene->get_surface_group(0);

		uint32_t const surface_count = surface_group->get_surface_count();

		assert(surface_count >= 1U);

		uint32_t const morph_target_count = surface_group->get_surface(0)->get_morph_target_count();

		uint32_t surface_group_vertex_count = 0U;
		uint32_t surface_group_morph_target_vertex_count = 0U;
		uint32_t surface_group_index_count = 0U;

		for (uint32_t surface_index = 0U; surface_index < surface_count; ++surface_index)
		{
			brx_asset_import_surface const *const surface = surface_group->get_surface(surface_index);

			surface_group_vertex_count = (surface_group_vertex_count + surface->get_vertex_count());

			if (surface->get_morph_target_count() > 0U)
			{
				assert(surface->get_morph_target_count() == morph_target_count);
				surface_group_morph_target_vertex_count = (surface_group_morph_target_vertex_count + surface->get_vertex_count());
			}

			surface_group_index_count = (surface_group_index_count + surface->get_index_count());
		}

		m_positions.reserve(surface_group_vertex_count);
		m_normals.reserve(surface_group_vertex_count);
		m_uvs.reserve(surface_group_vertex_count);
		m_vertexBoneInfos.reserve(surface_group_index_count);

		m_bboxMax = glm::vec3(-std::numeric_limits<float>::max());
		m_bboxMin = glm::vec3(std::numeric_limits<float>::max());

		this->m_morph_target_names.resize(morph_target_count);
		this->m_morph_targets_vertex_positions.resize(morph_target_count);
		this->m_morph_targets_vertex_uvs.resize(morph_target_count);

		for (uint32_t morph_target_index = 0U; morph_target_index < morph_target_count; ++morph_target_index)
		{
			this->m_morph_target_names[morph_target_index] = surface_group->get_surface(0)->get_morph_target_name(morph_target_index);
			this->m_morph_targets_vertex_positions[morph_target_index].reserve(surface_group_morph_target_vertex_count);
			this->m_morph_targets_vertex_uvs[morph_target_index].reserve(surface_group_morph_target_vertex_count);
		}

		m_indexElementSize = 4U;
		assert(sizeof(uint32_t) == m_indexElementSize);
		m_indexCount = surface_group_index_count;
		m_indices.resize(surface_group_index_count * m_indexElementSize);
		uint32_t *const indices = reinterpret_cast<uint32_t *>(m_indices.data());

		m_materials.reserve(surface_count);
		m_subMeshes.reserve(surface_count);

		uint32_t begin_vertex_index = 0U;
		uint32_t begin_index_index = 0U;
		for (uint32_t surface_index = 0U; surface_index < surface_count; ++surface_index)
		{
			brx_asset_import_surface const *const surface = surface_group->get_surface(surface_index);

			uint32_t const vertex_count = surface->get_vertex_count();

			for (uint32_t vertex_index = 0U; vertex_index < vertex_count; ++vertex_index)
			{
				brx_asset_import_vertex_position const *const vertex_position = surface->get_vertex_position(vertex_index);
				brx_asset_import_vertex_varying const *const vertex_varying = surface->get_vertex_varying(vertex_index);
				brx_asset_import_vertex_blending const *const vertex_blending = surface->get_vertex_blending(vertex_index);

				glm::vec3 const position(vertex_position->m_position[0], vertex_position->m_position[1], vertex_position->m_position[2]);
				glm::vec3 const normal(vertex_varying->m_normal[0], vertex_varying->m_normal[1], vertex_varying->m_normal[2]);
				glm::vec2 const texcoord(vertex_varying->m_texcoord[0], vertex_varying->m_texcoord[1]);

				m_positions.push_back(position);
				m_normals.push_back(normal);
				m_uvs.push_back(glm::vec2(texcoord.x, 1.0F - texcoord.y));

				VertexBoneInfo vtxBoneInfo;
				vtxBoneInfo.m_skinningType = SkinningType::Weight4;
				vtxBoneInfo.m_boneIndex[0] = vertex_blending->m_indices[0];
				vtxBoneInfo.m_boneIndex[1] = vertex_blending->m_indices[1];
				vtxBoneInfo.m_boneIndex[2] = vertex_blending->m_indices[2];
				vtxBoneInfo.m_boneIndex[3] = vertex_blending->m_indices[3];
				vtxBoneInfo.m_boneWeight[0] = vertex_blending->m_weights[0];
				vtxBoneInfo.m_boneWeight[1] = vertex_blending->m_weights[1];
				vtxBoneInfo.m_boneWeight[2] = vertex_blending->m_weights[2];
				vtxBoneInfo.m_boneWeight[3] = vertex_blending->m_weights[3];
				m_vertexBoneInfos.push_back(vtxBoneInfo);

				m_bboxMax = glm::max(m_bboxMax, position);
				m_bboxMin = glm::min(m_bboxMin, position);
			}

			assert((surface->get_morph_target_count() == 0U) || (surface->get_morph_target_count() == morph_target_count));

			if (surface->get_morph_target_count() > 0U)
			{
				for (uint32_t morph_target_index = 0U; morph_target_index < morph_target_count; ++morph_target_index)
				{
					assert(surface->get_morph_target_name(morph_target_index) == this->m_morph_target_names[morph_target_index]);

					for (uint32_t vertex_index = 0U; vertex_index < vertex_count; ++vertex_index)
					{
						brx_asset_import_vertex_position const *const vertex_position = surface->get_morph_target_vertex_position(morph_target_index, vertex_index);
						brx_asset_import_vertex_varying const *const vertex_varying = surface->get_morph_target_vertex_varying(morph_target_index, vertex_index);

						glm::vec3 const position(vertex_position->m_position[0], vertex_position->m_position[1], vertex_position->m_position[2]);
						glm::vec2 const texcoord(vertex_varying->m_texcoord[0], vertex_varying->m_texcoord[1]);

						this->m_morph_targets_vertex_positions[morph_target_index].emplace_back(position);
						this->m_morph_targets_vertex_uvs[morph_target_index].emplace_back(texcoord);
					}
				}
			}

			uint32_t const index_count = surface->get_index_count();

			for (uint32_t index_index = 0U; index_index < index_count; ++index_index)
			{
				uint32_t const vertex_index = surface->get_index(index_index);

				indices[begin_index_index + index_index] = (begin_vertex_index + vertex_index);
			}

			assert(1U == surface->get_texture_count());

			brx_asset_import_texture_factor const *const texture_factor = surface->get_texture_factor(0);

			bool const is_double_sided = surface->is_double_sided();

			void const *texture_url = surface->get_texture_url(0);
			assert('f' == reinterpret_cast<uint8_t const *>(texture_url)[0]);
			assert('i' == reinterpret_cast<uint8_t const *>(texture_url)[1]);
			assert('l' == reinterpret_cast<uint8_t const *>(texture_url)[2]);
			assert('e' == reinterpret_cast<uint8_t const *>(texture_url)[3]);
			assert(':' == reinterpret_cast<uint8_t const *>(texture_url)[4]);
			assert('/' == reinterpret_cast<uint8_t const *>(texture_url)[5]);
			assert('/' == reinterpret_cast<uint8_t const *>(texture_url)[6]);

			MMDMaterial mat;
			mat.m_diffuse = glm::vec3(texture_factor->m_rgba[0], texture_factor->m_rgba[1], texture_factor->m_rgba[2]);
			mat.m_alpha = texture_factor->m_rgba[3];
			mat.m_specularPower = 1.0F;
			mat.m_specular = glm::vec3(0.0F, 0.0F, 0.0F);
			mat.m_ambient = glm::vec3(0.5F, 0.5F, 0.5F);
			mat.m_spTextureMode = MMDMaterial::SphereTextureMode::None;
			mat.m_bothFace = is_double_sided;
			mat.m_edgeFlag = 0U;
			mat.m_groundShadow = 1U;
			mat.m_shadowCaster = 1U;
			mat.m_shadowReceiver = 1U;
			mat.m_edgeSize = 0.5F;
			mat.m_edgeColor = glm::vec4(0.0F, 0.0F, 0.0F, 1.0F);
			mat.m_texture = PathUtil::Normalize(PathUtil::Combine(PathUtil::GetDirectoryName(filepath), reinterpret_cast<char const *>(reinterpret_cast<uint8_t const *>(texture_url) + 7)));
			mat.m_toonTexture = PathUtil::Combine(mmdDataDir, "toon3.png");
			mat.m_spTexture = "";
			mat.m_spTextureMode = MMDMaterial::SphereTextureMode::None;

			m_materials.push_back(std::move(mat));

			MMDSubMesh subMesh;
			subMesh.m_beginIndex = begin_index_index;
			subMesh.m_vertexCount = index_count;
			subMesh.m_materialID = (int)(m_materials.size() - 1);
			m_subMeshes.push_back(subMesh);

			begin_vertex_index = begin_vertex_index + vertex_count;
			begin_index_index = begin_index_index + index_count;
		}

		assert(surface_group_vertex_count == m_positions.size());
		assert(surface_group_vertex_count == m_normals.size());
		assert(surface_group_vertex_count == m_uvs.size());
		assert(surface_group_vertex_count == m_vertexBoneInfos.size());

#ifndef NDEBUG
		for (uint32_t morph_target_index = 0U; morph_target_index < morph_target_count; ++morph_target_index)
		{
			assert(this->m_morph_targets_vertex_positions[morph_target_index].size() == surface_group_morph_target_vertex_count);
			assert(this->m_morph_targets_vertex_uvs[morph_target_index].size() == surface_group_morph_target_vertex_count);
		}
#endif

		m_initMaterials = m_materials;

		uint32_t const animation_skeleton_joint_count = surface_group->get_animation_skeleton_joint_count();

		this->m_skeleton_joint_names.resize(animation_skeleton_joint_count);
		this->m_animation_skeleton_joint_parent_indices.resize(animation_skeleton_joint_count);
		this->m_bind_pose_local_space.resize(animation_skeleton_joint_count);
		this->m_bind_pose_inverse_model_space.resize(animation_skeleton_joint_count);

		mcrt_vector<glm::mat4x4> animation_skeleton_bind_pose_model_space(static_cast<size_t>(animation_skeleton_joint_count));

		for (uint32_t animation_skeleton_joint_index = 0U; animation_skeleton_joint_index < animation_skeleton_joint_count; ++animation_skeleton_joint_index)
		{
			this->m_skeleton_joint_names[animation_skeleton_joint_index] = surface_group->get_animation_skeleton_joint_name(animation_skeleton_joint_index);

			brx_asset_import_rigid_transform const *const animation_skeleton_bind_pose_local_space = surface_group->get_animation_skeleton_joint_transform_bind_pose_local_space(animation_skeleton_joint_index);

			this->m_bind_pose_local_space[animation_skeleton_joint_index] = (*animation_skeleton_bind_pose_local_space);

			glm::mat4x4 current_bind_pose_local_space_matrix = glm::translate(glm::mat4(1), glm::vec3(animation_skeleton_bind_pose_local_space->m_translation[0], animation_skeleton_bind_pose_local_space->m_translation[1], animation_skeleton_bind_pose_local_space->m_translation[2])) * glm::mat4_cast(glm::quat(animation_skeleton_bind_pose_local_space->m_rotation[3], animation_skeleton_bind_pose_local_space->m_rotation[0], animation_skeleton_bind_pose_local_space->m_rotation[1], animation_skeleton_bind_pose_local_space->m_rotation[2]));

			uint32_t const parent_animation_skeleton_joint_index = surface_group->get_animation_skeleton_joint_parent_index(animation_skeleton_joint_index);

			this->m_animation_skeleton_joint_parent_indices[animation_skeleton_joint_index] = parent_animation_skeleton_joint_index;

			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
			{
				assert(parent_animation_skeleton_joint_index < animation_skeleton_joint_index);

				animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index] = current_bind_pose_local_space_matrix * animation_skeleton_bind_pose_model_space[parent_animation_skeleton_joint_index];
			}
			else
			{
				animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index] = current_bind_pose_local_space_matrix;
			}

			this->m_bind_pose_inverse_model_space[animation_skeleton_joint_index] = glm::inverse(*reinterpret_cast<glm::mat4x4 const *>(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index]));
		}

		uint32_t const animation_skeleton_joint_constraint_count = surface_group->get_animation_skeleton_joint_constraint_count();

		this->m_animation_skeleton_joint_constraint_names.resize(animation_skeleton_joint_constraint_count);
		this->m_animation_skeleton_joint_constraints.resize(animation_skeleton_joint_constraint_count);
		this->m_animation_skeleton_joint_constraints_storage.resize(animation_skeleton_joint_constraint_count);

		for (uint32_t animation_skeleton_joint_constraint_index = 0U; animation_skeleton_joint_constraint_index < animation_skeleton_joint_constraint_count; ++animation_skeleton_joint_constraint_index)
		{
			this->m_animation_skeleton_joint_constraint_names[animation_skeleton_joint_constraint_index] = surface_group->get_animation_skeleton_joint_constraint_name(animation_skeleton_joint_constraint_index);
			this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index] = *surface_group->get_animation_skeleton_joint_constraint(animation_skeleton_joint_constraint_index);

			if (BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_COPY_TRANSFORM == this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_constraint_type)
			{
				this->m_animation_skeleton_joint_constraints_storage[animation_skeleton_joint_constraint_index].resize(this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_weight_count);

				std::memcpy(this->m_animation_skeleton_joint_constraints_storage[animation_skeleton_joint_constraint_index].data(), this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_weights, sizeof(float) * this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_weight_count);

				this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_copy_transform.m_source_weights = reinterpret_cast<float *>(this->m_animation_skeleton_joint_constraints_storage[animation_skeleton_joint_constraint_index].data());
			}
			else
			{
				assert(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_INVERSE_KINEMATICS == this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_constraint_type);

				this->m_animation_skeleton_joint_constraints_storage[animation_skeleton_joint_constraint_index].resize(this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_joint_count);

				std::memcpy(this->m_animation_skeleton_joint_constraints_storage[animation_skeleton_joint_constraint_index].data(), this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_joint_indices, sizeof(uint32_t) * this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_joint_count);

				this->m_animation_skeleton_joint_constraints[animation_skeleton_joint_constraint_index].m_inverse_kinematics.m_ik_joint_indices = this->m_animation_skeleton_joint_constraints_storage[animation_skeleton_joint_constraint_index].data();
			}
		}

		this->m_transforms.resize(animation_skeleton_joint_count);

		this->m_morphPositions.resize(m_positions.size());
		this->m_morphUVs.resize(m_positions.size());

		this->m_updatePositions.resize(m_positions.size());
		this->m_updateNormals.resize(m_normals.size());
		this->m_updateUVs.resize(m_uvs.size());

		// Physics
		if (!m_physicsMan.Create())
		{
			SABA_ERROR("Create Physics Fail.");
			return false;
		}

		m_physicsMan.GetMMDPhysics()->InitRagdoll(surface_group);

		this->InitializeAnimation();

		SetupParallelUpdate();

		asset_import_destory_scene(scene);

		asset_import_destroy_file_input_stream_factory(input_stream_factory);

		return true;
	}

	void PMXModel::Destroy()
	{
		m_materials.clear();
		m_subMeshes.clear();

		m_positions.clear();
		m_normals.clear();
		m_uvs.clear();
		m_vertexBoneInfos.clear();

		m_indices.clear();

		m_updateRanges.clear();
	}

	void PMXModel::SetupParallelUpdate()
	{
		if (m_parallelUpdateCount == 0)
		{
			m_parallelUpdateCount = std::thread::hardware_concurrency();
		}
		size_t maxParallelCount = std::max(size_t(16), size_t(std::thread::hardware_concurrency()));
		if (m_parallelUpdateCount > maxParallelCount)
		{
			SABA_WARN("PMXModel::SetParallelUpdateCount parallelCount > {}", maxParallelCount);
			m_parallelUpdateCount = 16;
		}

		SABA_INFO("Select PMX Parallel Update Count : {}", m_parallelUpdateCount);

		m_updateRanges.resize(m_parallelUpdateCount);
		m_parallelUpdateFutures.resize(m_parallelUpdateCount - 1);

		const size_t vertexCount = m_positions.size();
		const size_t LowerVertexCount = 1000;
		if (vertexCount < m_updateRanges.size() * LowerVertexCount)
		{
			size_t numRanges = (vertexCount + LowerVertexCount - 1) / LowerVertexCount;
			for (size_t rangeIdx = 0; rangeIdx < m_updateRanges.size(); rangeIdx++)
			{
				auto &range = m_updateRanges[rangeIdx];
				if (rangeIdx < numRanges)
				{
					range.m_vertexOffset = rangeIdx * LowerVertexCount;
					range.m_vertexCount = std::min(LowerVertexCount, vertexCount - range.m_vertexOffset);
				}
				else
				{
					range.m_vertexOffset = 0;
					range.m_vertexCount = 0;
				}
			}
		}
		else
		{
			size_t numVertexCount = vertexCount / m_updateRanges.size();
			size_t offset = 0;
			for (size_t rangeIdx = 0; rangeIdx < m_updateRanges.size(); rangeIdx++)
			{
				auto &range = m_updateRanges[rangeIdx];
				range.m_vertexOffset = offset;
				range.m_vertexCount = numVertexCount;
				if (rangeIdx == 0)
				{
					range.m_vertexCount += vertexCount % m_updateRanges.size();
				}
				offset = range.m_vertexOffset + range.m_vertexCount;
			}
		}
	}

	void PMXModel::Update(const UpdateRange &range)
	{
		const auto *position = m_positions.data() + range.m_vertexOffset;
		const auto *normal = m_normals.data() + range.m_vertexOffset;
		const auto *uv = m_uvs.data() + range.m_vertexOffset;
		const auto *morphPos = m_morphPositions.data() + range.m_vertexOffset;
		const auto *morphUV = m_morphUVs.data() + range.m_vertexOffset;
		const auto *vtxInfo = m_vertexBoneInfos.data() + range.m_vertexOffset;
		const auto *transforms = m_transforms.data();
		auto *updatePosition = m_updatePositions.data() + range.m_vertexOffset;
		auto *updateNormal = m_updateNormals.data() + range.m_vertexOffset;
		auto *updateUV = m_updateUVs.data() + range.m_vertexOffset;

		for (size_t i = 0; i < range.m_vertexCount; i++)
		{
			glm::mat4 m;
			switch (vtxInfo->m_skinningType)
			{
			case PMXModel::SkinningType::Weight1:
			{
				const auto i0 = vtxInfo->m_boneIndex[0];
				const auto &m0 = transforms[i0];
				m = m0;
				break;
			}
			case PMXModel::SkinningType::Weight2:
			{
				const auto i0 = vtxInfo->m_boneIndex[0];
				const auto i1 = vtxInfo->m_boneIndex[1];
				const auto w0 = vtxInfo->m_boneWeight[0];
				const auto w1 = vtxInfo->m_boneWeight[1];
				const auto &m0 = transforms[i0];
				const auto &m1 = transforms[i1];
				m = m0 * w0 + m1 * w1;
				break;
			}
			case PMXModel::SkinningType::Weight4:
			{
				const auto i0 = vtxInfo->m_boneIndex[0];
				const auto i1 = vtxInfo->m_boneIndex[1];
				const auto i2 = vtxInfo->m_boneIndex[2];
				const auto i3 = vtxInfo->m_boneIndex[3];
				const auto w0 = vtxInfo->m_boneWeight[0];
				const auto w1 = vtxInfo->m_boneWeight[1];
				const auto w2 = vtxInfo->m_boneWeight[2];
				const auto w3 = vtxInfo->m_boneWeight[3];
				const auto &m0 = transforms[i0];
				const auto &m1 = transforms[i1];
				const auto &m2 = transforms[i2];
				const auto &m3 = transforms[i3];
				m = m0 * w0 + m1 * w1 + m2 * w2 + m3 * w3;
				break;
			}
			case PMXModel::SkinningType::SDEF:
			{
				assert(false);
				break;
			}
			case PMXModel::SkinningType::DualQuaternion:
			{
				//
				// Skinning with Dual Quaternions
				// https://www.cs.utah.edu/~ladislav/dq/index.html
				//
				glm::dualquat dq[4];
				float w[4] = {0};
				for (int bi = 0; bi < 4; bi++)
				{
					auto boneID = vtxInfo->m_boneIndex[bi];
					if (boneID != -1)
					{
						dq[bi] = glm::dualquat_cast(glm::mat3x4(glm::transpose(transforms[boneID])));
						dq[bi] = glm::normalize(dq[bi]);
						w[bi] = vtxInfo->m_boneWeight[bi];
					}
					else
					{
						w[bi] = 0;
					}
				}
				if (glm::dot(dq[0].real, dq[1].real) < 0)
				{
					w[1] *= -1.0f;
				}
				if (glm::dot(dq[0].real, dq[2].real) < 0)
				{
					w[2] *= -1.0f;
				}
				if (glm::dot(dq[0].real, dq[3].real) < 0)
				{
					w[3] *= -1.0f;
				}
				auto blendDQ = w[0] * dq[0] + w[1] * dq[1] + w[2] * dq[2] + w[3] * dq[3];
				blendDQ = glm::normalize(blendDQ);
				m = glm::transpose(glm::mat3x4_cast(blendDQ));
				break;
			}
			default:
				break;
			}

			if (PMXModel::SkinningType::SDEF != vtxInfo->m_skinningType)
			{
				*updatePosition = glm::vec3(m * glm::vec4(*position + *morphPos, 1));
				*updateNormal = glm::normalize(glm::mat3(m) * *normal);
			}
			*updateUV = *uv + glm::vec2((*morphUV).x, (*morphUV).y);

			vtxInfo++;
			position++;
			normal++;
			uv++;
			updatePosition++;
			updateNormal++;
			updateUV++;
			morphPos++;
			morphUV++;
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
