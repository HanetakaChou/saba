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
#include "VMDAnimation.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#ifndef NDEBUG
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/McRT-Malloc.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Asset-Import.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Motion.lib")
#else
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/McRT-Malloc.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Asset-Import.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Motion.lib")
#endif

static inline BRX_MOTION_MORPH_TARGET_NAME const *wrap(BRX_ASSET_IMPORT_MORPH_TARGET_NAME const *morph_target_name)
{
	static_assert(static_cast<uint32_t>(BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY) == static_cast<uint32_t>(BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_HAPPY), "");
	return reinterpret_cast<BRX_MOTION_MORPH_TARGET_NAME const *>(morph_target_name);
}

static inline BRX_MOTION_SKELETON_JOINT_NAME const *wrap(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME const *skeleton_joint_name)
{
	static_assert(static_cast<uint32_t>(BRX_MOTION_SKELETON_JOINT_NAME_MMD_CONTROL_NODE) == static_cast<uint32_t>(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_CONTROL_NODE), "");
	return reinterpret_cast<BRX_MOTION_SKELETON_JOINT_NAME const *>(skeleton_joint_name);
}

static inline BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *wrap(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME const *skeleton_joint_constraint_name)
{
	static_assert(static_cast<uint32_t>(BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_RIGHT_ANKLE) == static_cast<uint32_t>(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_RIGHT_ANKLE), "");
	return reinterpret_cast<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *>(skeleton_joint_constraint_name);
}

static inline brx_motion_rigid_transform const *wrap(brx_asset_import_rigid_transform const *rigid_transform)
{
	static_assert(sizeof(brx_motion_rigid_transform) == sizeof(brx_asset_import_rigid_transform), "");
	return reinterpret_cast<brx_motion_rigid_transform const *>(rigid_transform);
}

static inline brx_motion_skeleton_joint_constraint const *wrap(brx_asset_import_skeleton_joint_constraint const *skeleton_joint_constraint)
{
	static_assert(sizeof(brx_motion_skeleton_joint_constraint) == sizeof(brx_asset_import_skeleton_joint_constraint), "");
	return reinterpret_cast<brx_motion_skeleton_joint_constraint const *>(skeleton_joint_constraint);
}

static inline brx_motion_physics_rigid_body const *wrap(brx_asset_import_physics_rigid_body const *physics_rigid_body)
{
	static_assert(sizeof(brx_motion_physics_rigid_body) == sizeof(brx_asset_import_physics_rigid_body), "");
	return reinterpret_cast<brx_motion_physics_rigid_body const *>(physics_rigid_body);
}

static inline brx_motion_physics_constraint const *wrap(brx_asset_import_physics_constraint const *physics_constrain)
{
	static_assert(sizeof(brx_motion_physics_constraint) == sizeof(brx_asset_import_physics_constraint), "");
	return reinterpret_cast<brx_motion_physics_constraint const *>(physics_constrain);
}

static inline brx_motion_ragdoll_direct_mapping const *wrap(brx_asset_import_ragdoll_direct_mapping const *ragdoll_direct_mapping)
{
	static_assert(sizeof(brx_motion_ragdoll_direct_mapping) == sizeof(brx_asset_import_ragdoll_direct_mapping), "");
	return reinterpret_cast<brx_motion_ragdoll_direct_mapping const *>(ragdoll_direct_mapping);
}

static_assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == BRX_MOTION_UINT32_INDEX_INVALID, "");

namespace saba
{
	PMXModel::PMXModel() : m_parallelUpdateCount(0), m_skeleton(NULL), m_skeleton_instance(NULL), m_previous_animation_instance(NULL), m_motion_video_capture(NULL), m_motion_video_detector(NULL), m_previous_motion_video_detector(NULL)
	{
	}

	PMXModel::~PMXModel()
	{
		Destroy();
	}

	void PMXModel::InitializeAnimation()
	{
		ClearBaseAnimation();
	}

	void PMXModel::SaveBaseAnimation()
	{
	}

	void PMXModel::LoadBaseAnimation()
	{
	}

	void PMXModel::ClearBaseAnimation()
	{
	}

	void PMXModel::UpdateMotionCaptureAnimation()
	{
		this->m_motion_video_capture->step();

		this->m_motion_video_detector->step();

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

				float const morph_target_weight = this->m_motion_video_detector->get_morph_target_weight(0U, *wrap(&morph_target_name));

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

		if (this->m_motion_video_detector != this->m_previous_motion_video_detector)
		{
			this->m_skeleton_instance->set_input(this->m_motion_video_detector, 0U, 0U);
			this->m_previous_animation_instance = NULL;
			this->m_previous_motion_video_detector = this->m_motion_video_detector;
		}

		this->m_skeleton_instance->step();

		for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < this->m_animation_skeleton_joint_count; ++animation_skeleton_joint_index)
		{
			brx_motion_rigid_transform const &skin_transform = this->m_skeleton_instance->get_skin_transforms()[animation_skeleton_joint_index];

			glm::quat skin_rotation(skin_transform.m_rotation[3], skin_transform.m_rotation[0], skin_transform.m_rotation[1], skin_transform.m_rotation[2]);
			glm::vec3 skin_translation(skin_transform.m_translation[0], skin_transform.m_translation[1], skin_transform.m_translation[2]);

			this->m_transforms[animation_skeleton_joint_index] = glm::translate(glm::mat4(1), skin_translation) * glm::mat4_cast(skin_rotation);
		}
	}

	void PMXModel::UpdateMorphAnimation(VMDAnimation const *animation)
	{
		uint32_t const vertex_count = m_morphPositions.size();
		for (uint32_t vtxIdx = 0; vtxIdx < vertex_count; ++vtxIdx)
		{
			m_morphPositions[vtxIdx] = glm::vec3(0);
			m_morphUVs[vtxIdx] = glm::vec4(0);
		}

		if (animation)
		{
			constexpr float const INTERNAL_WEIGHT_EPSILON = 1E-6F;

			uint32_t const morph_target_count = this->m_morph_target_names.size();
			assert(this->m_morph_targets_vertex_positions.size() == morph_target_count);
			assert(this->m_morph_targets_vertex_uvs.size() == morph_target_count);

			for (uint32_t morph_target_index = 0U; morph_target_index < morph_target_count; ++morph_target_index)
			{
				BRX_ASSET_IMPORT_MORPH_TARGET_NAME const morph_target_name = this->m_morph_target_names[morph_target_index];

				float const morph_target_weight = animation->get_morph_target_weight(morph_target_name);

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
	}

	void PMXModel::UpdateNodeAnimation(VMDAnimation const *animation)
	{
		brx_motion_animation_instance const *const animation_instance = (NULL != animation) ? animation->get_motion_animation_instance() : NULL;

		if (animation_instance != this->m_previous_animation_instance)
		{
			this->m_skeleton_instance->set_input(animation_instance);
			this->m_previous_animation_instance = animation_instance;
			this->m_previous_motion_video_detector = NULL;
		}

		this->m_skeleton_instance->step();

		for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < this->m_animation_skeleton_joint_count; ++animation_skeleton_joint_index)
		{
			brx_motion_rigid_transform const &skin_transform = this->m_skeleton_instance->get_skin_transforms()[animation_skeleton_joint_index];

			glm::quat skin_rotation(skin_transform.m_rotation[3], skin_transform.m_rotation[0], skin_transform.m_rotation[1], skin_transform.m_rotation[2]);
			glm::vec3 skin_translation(skin_transform.m_translation[0], skin_transform.m_translation[1], skin_transform.m_translation[2]);

			this->m_transforms[animation_skeleton_joint_index] = glm::translate(glm::mat4(1), skin_translation) * glm::mat4_cast(skin_rotation);
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
		brx_asset_import_input_stream_factory *const input_stream_factory = brx_asset_import_create_file_input_stream_factory();

		brx_asset_import_scene *const scene = brx_asset_import_create_scene_from_input_stream(input_stream_factory, filepath.c_str());

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
				brx_asset_import_surface_vertex_position const *const vertex_position = surface->get_vertex_positions() + vertex_index;
				brx_asset_import_surface_vertex_varying const *const vertex_varying = surface->get_vertex_varyings() + vertex_index;
				brx_asset_import_surface_vertex_blending const *const vertex_blending = surface->get_vertex_blendings() + vertex_index;

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
						brx_asset_import_surface_vertex_position const *const vertex_position = surface->get_morph_target_vertex_positions(morph_target_index) + vertex_index;
						brx_asset_import_surface_vertex_varying const *const vertex_varying = surface->get_morph_target_vertex_varyings(morph_target_index) + vertex_index;

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

		this->m_animation_skeleton_joint_count = surface_group->get_animation_skeleton_joint_count();

		assert(NULL == this->m_skeleton);
		this->m_skeleton = brx_motion_create_skeleton(this->m_animation_skeleton_joint_count, wrap(surface_group->get_animation_skeleton_joint_names()), surface_group->get_animation_skeleton_joint_parent_indices(), wrap(surface_group->get_animation_skeleton_joint_transforms_bind_pose_local_space()), surface_group->get_animation_skeleton_joint_constraint_count(), wrap(surface_group->get_animation_skeleton_joint_constraint_names()), wrap(surface_group->get_animation_skeleton_joint_constraints()), surface_group->get_ragdoll_skeleton_rigid_body_count(), wrap(surface_group->get_ragdoll_skeleton_rigid_bodies()), surface_group->get_ragdoll_skeleton_constraint_count(), wrap(surface_group->get_ragdoll_skeleton_constraints()), surface_group->get_animation_to_ragdoll_direct_mapping_count(), wrap(surface_group->get_animation_to_ragdoll_direct_mappings()), surface_group->get_ragdoll_to_animation_direct_mapping_count(), wrap(surface_group->get_ragdoll_to_animation_direct_mappings()));

		assert(NULL == this->m_skeleton_instance);
		this->m_skeleton_instance = brx_motion_create_skeleton_instance(this->m_skeleton);

		this->m_transforms.resize(this->m_animation_skeleton_joint_count);

		this->m_morphPositions.resize(m_positions.size());
		this->m_morphUVs.resize(m_positions.size());

		this->m_updatePositions.resize(m_positions.size());
		this->m_updateNormals.resize(m_normals.size());
		this->m_updateUVs.resize(m_uvs.size());

		this->InitializeAnimation();

		SetupParallelUpdate();

		brx_asset_import_destroy_scene(scene);

		brx_asset_import_destroy_file_input_stream_factory(input_stream_factory);

		assert(NULL == this->m_motion_video_capture);
		// this->m_motion_video_capture = motion_create_video_capture("file://C:\\Users\\HanetakaChou\\Videos\\01e76ab2f32208564f03700193f8cb37e4_258.mp4");
		// this->m_motion_video_capture = motion_create_video_capture("file://C:\\Users\\HanetakaChou\\Videos\\7175435261885631801.mp4");
		// this->m_motion_video_capture = motion_create_video_capture("file://C:\\Users\\HanetakaChou\\Videos\\trump-2.mp4");
		// this->m_motion_video_capture = motion_create_video_capture("file://C:\\Users\\HanetakaChou\\Videos\\keqing-lolita-love-you.mp4");
		this->m_motion_video_capture = brx_motion_create_video_capture("camera://0");

		assert(NULL == this->m_motion_video_detector);
		this->m_motion_video_detector = brx_motion_create_video_detector(1U, 1U, true);
		this->m_motion_video_detector->set_enable_debug_renderer(true, filepath.c_str());

		this->m_motion_video_detector->set_input(this->m_motion_video_capture);

		return true;
	}

	void PMXModel::Destroy()
	{
		assert(NULL != this->m_skeleton_instance);
		brx_motion_destroy_skeleton_instance(this->m_skeleton_instance);

		assert(NULL != this->m_skeleton);
		brx_motion_destroy_skeleton(this->m_skeleton);

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
			assert(PMXModel::SkinningType::Weight4 == vtxInfo->m_skinningType);

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

			*updatePosition = glm::vec3(m * glm::vec4(*position + *morphPos, 1));
			*updateNormal = glm::normalize(glm::mat3(m) * *normal);

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
