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

#include <DirectXMath.h>

static inline void internal_import_morph_targets(mcrt_vector<mmd_pmx_vertex_t> const &in_vertices, mcrt_vector<mmd_pmx_face_t> const &in_faces, mcrt_vector<mmd_pmx_material_t> const &in_materials, mcrt_vector<mmd_pmx_morph_t> const &in_mmd_morphs, mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> &out_morph_target_names, mcrt_vector<mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t>> &out_morph_targets);

static inline void internal_import_animation_skeleton(mcrt_vector<mmd_pmx_bone_t> const &in_mmd_model_nodes, mcrt_vector<uint32_t> &out_animation_skeleton_joint_parent_indices, mcrt_vector<uint32_t> &out_model_node_to_animation_skeleton_joint_map, mcrt_vector<uint32_t> &out_animation_skeleton_joint_to_model_node_map, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_model_space, mcrt_vector<BRX_ASSET_IMPORT_IK_NAME> &out_ik_names, mcrt_vector<brx_asset_import_skeleton_joint_constraint> &out_animation_skeleton_joint_constraints, mcrt_vector<mcrt_vector<uint32_t>> &out_animation_skeleton_joint_constraints_storage);

namespace saba
{

	namespace
	{
		glm::vec3 InvZ(const glm::vec3 &v)
		{
			return v;
		}
		glm::mat3 InvZ(const glm::mat3 &m)
		{
			return m;
		}
		glm::quat InvZ(const glm::quat &q)
		{
			return q;
		}

		uint32_t InvZ(uint32_t const vertex_indices[3], uint32_t i)
		{
			return vertex_indices[i];
		}
	}

	PMXModel::PMXModel()
		: m_parallelUpdateCount(0)
	{
	}

	PMXModel::~PMXModel()
	{
		Destroy();
	}

	void PMXModel::InitializeAnimation()
	{
		ClearBaseAnimation();

		for (auto &node : (*m_nodeMan.GetNodes()))
		{
			node->SetAnimationTranslate(glm::vec3(0));
			node->SetAnimationRotate(glm::quat(1, 0, 0, 0));
		}

		BeginAnimation();

		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_morph_target_name_weights[mmd_morph_target_name] = 0.0F;
		}

		for (uint32_t mmd_ik_name = 0U; mmd_ik_name < BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT; ++mmd_ik_name)
		{
			this->m_ik_name_switches[mmd_ik_name] = true;
		}

		EndAnimation();
	}

	void PMXModel::SaveBaseAnimation()
	{
		auto nodeMan = GetNodeManager();
		for (size_t i = 0; i < nodeMan->GetNodeCount(); i++)
		{
			auto node = nodeMan->GetMMDNode(i);
			node->SaveBaseAnimation();
		}

		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_saved_morph_target_name_weights[mmd_morph_target_name] = this->m_morph_target_name_weights[mmd_morph_target_name];
		}

		for (uint32_t mmd_ik_name = 0U; mmd_ik_name < BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT; ++mmd_ik_name)
		{
			this->m_saved_ik_name_switches[mmd_ik_name] = this->m_ik_name_switches[mmd_ik_name];
		}
	}

	void PMXModel::LoadBaseAnimation()
	{
		auto nodeMan = GetNodeManager();
		for (size_t i = 0; i < nodeMan->GetNodeCount(); i++)
		{
			auto node = nodeMan->GetMMDNode(i);
			node->LoadBaseAnimation();
		}

		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_morph_target_name_weights[mmd_morph_target_name] = this->m_saved_morph_target_name_weights[mmd_morph_target_name];
		}

		for (uint32_t mmd_ik_name = 0U; mmd_ik_name < BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT; ++mmd_ik_name)
		{
			this->m_ik_name_switches[mmd_ik_name] = this->m_saved_ik_name_switches[mmd_ik_name];
		}
	}

	void PMXModel::ClearBaseAnimation()
	{
		auto nodeMan = GetNodeManager();
		for (size_t i = 0; i < nodeMan->GetNodeCount(); i++)
		{
			auto node = nodeMan->GetMMDNode(i);
			node->ClearBaseAnimation();
		}

		for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
		{
			this->m_saved_morph_target_name_weights[mmd_morph_target_name] = 0.0F;
		}

		for (uint32_t mmd_ik_name = 0U; mmd_ik_name < BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT; ++mmd_ik_name)
		{
			this->m_saved_ik_name_switches[mmd_ik_name] = true;
		}
	}

	void PMXModel::BeginAnimation()
	{
		for (auto &node : (*m_nodeMan.GetNodes()))
		{
			node->BeginUpdateTransform();
		}

		size_t vtxCount = m_morphPositions.size();
		for (size_t vtxIdx = 0; vtxIdx < vtxCount; vtxIdx++)
		{
			m_morphPositions[vtxIdx] = glm::vec3(0);
			m_morphUVs[vtxIdx] = glm::vec4(0);
		}
	}

	void PMXModel::EndAnimation()
	{
		for (auto &node : (*m_nodeMan.GetNodes()))
		{
			node->EndUpdateTransform();
		}
	}

	void PMXModel::UpdateMorphAnimation()
	{
		constexpr float const INTERNAL_WEIGHT_EPSILON = 1E-6F;

		uint32_t const morph_target_count = this->m_morph_target_names.size();
		assert(this->m_morph_targets.size() == morph_target_count);

		for (uint32_t morph_target_index = 0U; morph_target_index < morph_target_count; ++morph_target_index)
		{
			BRX_ASSET_IMPORT_MORPH_TARGET_NAME const morph_target_name = this->m_morph_target_names[morph_target_index];

			float const morph_target_weight = this->m_morph_target_name_weights[morph_target_name];

			if (morph_target_weight > INTERNAL_WEIGHT_EPSILON)
			{
				mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t> const &morph_target = this->m_morph_targets[morph_target_index];
				assert(!morph_target.empty());

				for (auto const &vertex_index_morph_target_vertex : morph_target)
				{
					uint32_t const vertex_index = vertex_index_morph_target_vertex.first;

					internal_mmd_morph_target_vertex_t const morph_target_vertex{
						{
							vertex_index_morph_target_vertex.second.m_position[0] * morph_target_weight,
							vertex_index_morph_target_vertex.second.m_position[1] * morph_target_weight,
							vertex_index_morph_target_vertex.second.m_position[2] * morph_target_weight,
						},
						{
							vertex_index_morph_target_vertex.second.m_uv[0] * morph_target_weight,
							vertex_index_morph_target_vertex.second.m_uv[1] * morph_target_weight,
						}};

					m_morphPositions[vertex_index].x += morph_target_vertex.m_position[0];
					m_morphPositions[vertex_index].y += morph_target_vertex.m_position[1];
					m_morphPositions[vertex_index].z += morph_target_vertex.m_position[2];

					m_morphUVs[vertex_index].x += morph_target_vertex.m_uv[0];
					m_morphUVs[vertex_index].y += morph_target_vertex.m_uv[1];
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
		mcrt_vector<glm::mat4x4> animation_skeleton_bind_pose_local_space(static_cast<size_t>(m_nodeMan.GetNodeCount()));
		for (size_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < m_nodeMan.GetNodeCount(); ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode const *const animation_node = m_nodeMan.GetNode(model_node_index);
			assert(!animation_node->IsDeformAfterPhysics());

			assert(glm::all(glm::epsilonEqual(animation_node->GetInitialScale(), glm::vec3(1.0F), 1E-3F)));
			animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index] = glm::translate(glm::mat4(1), animation_node->GetInitialTranslate()) * glm::mat4_cast(animation_node->GetInitialRotate());
		}

		mcrt_vector<glm::mat4x4> animation_skeleton_animation_pose_local_space(static_cast<size_t>(m_nodeMan.GetNodeCount()));
		for (size_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < m_nodeMan.GetNodeCount(); ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode const *const animation_node = m_nodeMan.GetNode(model_node_index);
			assert(!animation_node->IsDeformAfterPhysics());
			animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index] = animation_node->GetLocalTransform();
		}

		uint32_t animation_skeleton_ik_joint_constraint_index = 0U;
		for (brx_asset_import_skeleton_joint_constraint const &animation_skeleton_joint_constraint : this->m_animation_skeleton_joint_constraints)
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

				BRX_ASSET_IMPORT_IK_NAME const ik_name = this->m_ik_names[animation_skeleton_ik_joint_constraint_index];

				if ((BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT == ik_name) || this->m_ik_name_switches[ik_name])
				{
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
						glm::quat updated_current_joint_local_space_rotation = glm::quat_cast(glm::mat3(ik_joints_local_space[ik_joint_index]));

						uint32_t const animation_skeleton_joint_index = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_indices[ik_joint_index];

						animation_skeleton_animation_pose_local_space[animation_skeleton_joint_index] = ik_joints_local_space[ik_joint_index];
					}
				}

				++animation_skeleton_ik_joint_constraint_index;
			}
		}
		assert(this->m_ik_names.size() == animation_skeleton_ik_joint_constraint_index);

		mcrt_vector<glm::mat4x4> animation_skeleton_animation_pose_model_space(static_cast<size_t>(m_nodeMan.GetNodeCount()));
		for (uint32_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < static_cast<uint32_t>(m_nodeMan.GetNodeCount()); ++current_animation_skeleton_joint_index)
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

			physics->RagdollToAnimation(animation_skeleton_animation_pose_model_space.data());
		}

		for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < static_cast<uint32_t>(m_nodeMan.GetNodeCount()); ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode *const animation_node = m_nodeMan.GetNode(model_node_index);
			assert(!animation_node->IsDeformAfterPhysics());
			animation_node->SetGlobalTransform(animation_skeleton_animation_pose_model_space[animation_skeleton_joint_index]);
		}
	}

	void PMXModel::ResetPhysics()
	{
		assert(false);
	}

	void PMXModel::Update()
	{
		auto &nodes = (*m_nodeMan.GetNodes());

		// スキンメッシュに使用する変形マトリクスを事前計算
		for (size_t i = 0; i < nodes.size(); i++)
		{
			m_transforms[i] = nodes[i]->GetGlobalTransform() * nodes[i]->GetInverseInitTransform();
		}

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

		PMXFile pmx;
		if (!ReadPMXFile(&pmx, filepath.c_str()))
		{
			return false;
		}

		std::string dirPath = PathUtil::GetDirectoryName(filepath);

		size_t vertexCount = pmx.m_pmx.m_vertices.size();
		m_positions.reserve(vertexCount);
		m_normals.reserve(vertexCount);
		m_uvs.reserve(vertexCount);
		m_vertexBoneInfos.reserve(vertexCount);
		m_bboxMax = glm::vec3(-std::numeric_limits<float>::max());
		m_bboxMin = glm::vec3(std::numeric_limits<float>::max());

		for (mmd_pmx_vertex_t const &v : pmx.m_pmx.m_vertices)
		{
			glm::vec3 pos = InvZ(glm::vec3(v.m_position.m_x, v.m_position.m_y, v.m_position.m_z));
			glm::vec3 nor = InvZ(glm::vec3(v.m_normal.m_x, v.m_normal.m_y, v.m_normal.m_z));
			glm::vec2 uv = glm::vec2(v.m_uv.m_x, 1.0F - v.m_uv.m_y);
			m_positions.push_back(pos);
			m_normals.push_back(nor);
			m_uvs.push_back(uv);

			VertexBoneInfo vtxBoneInfo;
			vtxBoneInfo.m_skinningType = SkinningType::DualQuaternion;
			vtxBoneInfo.m_boneIndex[0] = v.m_bone_indices[0];
			vtxBoneInfo.m_boneIndex[1] = v.m_bone_indices[1];
			vtxBoneInfo.m_boneIndex[2] = v.m_bone_indices[2];
			vtxBoneInfo.m_boneIndex[3] = v.m_bone_indices[3];
			float const sum_weight = v.m_bone_weights[0] + v.m_bone_weights[1] + v.m_bone_weights[2] + v.m_bone_weights[3];
			assert(sum_weight > 1E-6F);
			vtxBoneInfo.m_boneWeight[0] = v.m_bone_weights[0] / std::max(1E-6F, sum_weight);
			vtxBoneInfo.m_boneWeight[1] = v.m_bone_weights[1] / std::max(1E-6F, sum_weight);
			vtxBoneInfo.m_boneWeight[2] = v.m_bone_weights[2] / std::max(1E-6F, sum_weight);
			vtxBoneInfo.m_boneWeight[3] = v.m_bone_weights[3] / std::max(1E-6F, sum_weight);
			m_vertexBoneInfos.push_back(vtxBoneInfo);

			m_bboxMax = glm::max(m_bboxMax, pos);
			m_bboxMin = glm::min(m_bboxMin, pos);
		}
		m_morphPositions.resize(m_positions.size());
		m_morphUVs.resize(m_positions.size());
		m_updatePositions.resize(m_positions.size());
		m_updateNormals.resize(m_normals.size());
		m_updateUVs.resize(m_uvs.size());

		m_indexElementSize = 4U;
		m_indices.resize(pmx.m_pmx.m_faces.size() * 3 * m_indexElementSize);
		m_indexCount = pmx.m_pmx.m_faces.size() * 3;
		assert(sizeof(uint32_t) == m_indexElementSize);
		{
			int idx = 0;
			uint32_t *indices = (uint32_t *)m_indices.data();
			for (mmd_pmx_face_t const &face : pmx.m_pmx.m_faces)
			{
				for (int i = 0; i < 3; i++)
				{
					auto vi = InvZ(face.m_vertex_indices, i);
					indices[idx] = (uint32_t)vi;
					idx++;
				}
			}
		}

		std::vector<std::string> texturePaths;
		texturePaths.reserve(pmx.m_pmx.m_textures.size());
		for (mmd_pmx_texture_t const &pmxTex : pmx.m_pmx.m_textures)
		{
			std::string texPath = PathUtil::Combine(dirPath, pmxTex.m_path);
			texturePaths.emplace_back(std::move(texPath));
		}

		// Materialをコピー
		m_materials.reserve(pmx.m_pmx.m_materials.size());
		m_subMeshes.reserve(pmx.m_pmx.m_materials.size());
		uint32_t beginIndex = 0;
		for (mmd_pmx_material_t const &pmxMat : pmx.m_pmx.m_materials)
		{
			MMDMaterial mat;
			mat.m_diffuse = glm::vec3(pmxMat.m_diffuse.m_x, pmxMat.m_diffuse.m_y, pmxMat.m_diffuse.m_z);
			mat.m_alpha = pmxMat.m_diffuse.m_w;
			mat.m_specularPower = 1.0F;
			mat.m_specular = glm::vec3(0.0F, 0.0F, 0.0F);
			mat.m_ambient = glm::vec3(0.5F, 0.5F, 0.5F);
			mat.m_spTextureMode = MMDMaterial::SphereTextureMode::None;
			mat.m_bothFace = pmxMat.m_is_double_sided;
			mat.m_edgeFlag = 0U;
			mat.m_groundShadow = 1U;
			mat.m_shadowCaster = 1U;
			mat.m_shadowReceiver = 1U;
			mat.m_edgeSize = 0.5F;
			mat.m_edgeColor = glm::vec4(0.0F, 0.0F, 0.0F, 1.0F);

			if (pmxMat.m_texture_index != -1)
			{
				mat.m_texture = PathUtil::Normalize(texturePaths[pmxMat.m_texture_index]);
			}

			mat.m_toonTexture = PathUtil::Combine(mmdDataDir, "toon3.png");

			mat.m_spTexture = "";
			mat.m_spTextureMode = MMDMaterial::SphereTextureMode::None;

			m_materials.emplace_back(std::move(mat));

			MMDSubMesh subMesh;
			subMesh.m_beginIndex = beginIndex;
			subMesh.m_vertexCount = 3U * pmxMat.m_face_count;
			subMesh.m_materialID = (int)(m_materials.size() - 1);
			m_subMeshes.push_back(subMesh);

			beginIndex = beginIndex + 3U * pmxMat.m_face_count;
		}
		m_initMaterials = m_materials;

		internal_import_morph_targets(pmx.m_pmx.m_vertices, pmx.m_pmx.m_faces, pmx.m_pmx.m_materials, pmx.m_pmx.m_morphs, this->m_morph_target_names, this->m_morph_targets);

		mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_local_space;
		mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_model_space;
		internal_import_animation_skeleton(pmx.m_pmx.m_bones, this->m_animation_skeleton_joint_parent_indices, this->m_model_node_to_animation_skeleton_joint_map, this->m_animation_skeleton_joint_to_model_node_map, animation_skeleton_bind_pose_local_space, animation_skeleton_bind_pose_model_space, this->m_ik_names, this->m_animation_skeleton_joint_constraints, this->m_animation_skeleton_joint_constraints_storage);

		uint32_t const model_node_count = pmx.m_pmx.m_bones.size();

		m_nodeMan.GetNodes()->reserve(pmx.m_pmx.m_bones.size());
		for (mmd_pmx_bone_t const &bone : pmx.m_pmx.m_bones)
		{
			PMXNode *const node = m_nodeMan.AddNode();
			node->SetName(bone.m_name);

			node->EnableDeformAfterPhysics(bone.m_meta_physics);
		}

		m_transforms.resize(m_nodeMan.GetNodeCount());

		for (size_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < model_node_count; ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode *const animation_node = m_nodeMan.GetNode(model_node_index);

			animation_node->SetLocalTransform(*reinterpret_cast<glm::mat4x4 const *>(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index]));
			animation_node->SetGlobalTransform(*reinterpret_cast<glm::mat4x4 const *>(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index]));
			animation_node->CalculateInverseInitTransform();
			animation_node->SaveInitialTRS();
		}

		// Physics
		if (!m_physicsMan.Create())
		{
			SABA_ERROR("Create Physics Fail.");
			return false;
		}

		m_physicsMan.GetMMDPhysics()->InitRagdoll(pmx.m_pmx.m_rigid_bodies, pmx.m_pmx.m_constraints, this->m_model_node_to_animation_skeleton_joint_map.data(), reinterpret_cast<glm::mat4x4 *>(animation_skeleton_bind_pose_model_space.data()));

		SetupParallelUpdate();

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

		m_nodeMan.GetNodes()->clear();

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
				// https://github.com/powroupi/blender_mmd_tools/blob/dev_test/mmd_tools/core/sdef.py

				auto &nodes = (*m_nodeMan.GetNodes());
				const auto i0 = vtxInfo->m_sdef.m_boneIndex[0];
				const auto i1 = vtxInfo->m_sdef.m_boneIndex[1];
				const auto w0 = vtxInfo->m_sdef.m_boneWeight;
				const auto w1 = 1.0f - w0;
				const auto center = vtxInfo->m_sdef.m_sdefC;
				const auto cr0 = vtxInfo->m_sdef.m_sdefR0;
				const auto cr1 = vtxInfo->m_sdef.m_sdefR1;
				const auto q0 = glm::quat_cast(nodes[i0]->GetGlobalTransform());
				const auto q1 = glm::quat_cast(nodes[i1]->GetGlobalTransform());
				const auto m0 = transforms[i0];
				const auto m1 = transforms[i1];

				const auto pos = *position + *morphPos;
				const auto rot_mat = glm::mat3_cast(glm::slerp(q0, q1, w1));

				*updatePosition = glm::mat3(rot_mat) * (pos - center) + glm::vec3(m0 * glm::vec4(cr0, 1)) * w0 + glm::vec3(m1 * glm::vec4(cr1, 1)) * w1;
				*updateNormal = rot_mat * *normal;

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

	PMXNode::PMXNode()
		: m_isDeformAfterPhysics(false)
	{
	}

	void PMXNode::OnBeginUpdateTransform()
	{
	}

	void PMXNode::OnEndUpdateTransfrom()
	{
	}
}

static inline void internal_import_morph_targets(mcrt_vector<mmd_pmx_vertex_t> const &in_vertices, mcrt_vector<mmd_pmx_face_t> const &in_faces, mcrt_vector<mmd_pmx_material_t> const &in_materials, mcrt_vector<mmd_pmx_morph_t> const &in_mmd_morphs, mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> &out_morph_target_names, mcrt_vector<mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t>> &out_morph_targets)
{
	mcrt_unordered_map<mcrt_string, mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t>> mmd_morph_targets;
	{
		// DAG
		// Topological Sort

		uint32_t const mmd_morph_count = in_mmd_morphs.size();

		mcrt_vector<uint32_t> mmd_morph_parent_count(static_cast<size_t>(mmd_morph_count), 0U);
		mcrt_vector<mcrt_vector<uint32_t>> mmd_morph_children_indices(static_cast<size_t>(mmd_morph_count));
		mcrt_vector<mcrt_vector<float>> mmd_morph_children_weights(static_cast<size_t>(mmd_morph_count));
		for (size_t mmd_morph_index = 0U; mmd_morph_index < mmd_morph_count; ++mmd_morph_index)
		{
			if (0U == in_mmd_morphs[mmd_morph_index].m_morph_type)
			{
				uint32_t const child_index = mmd_morph_index;
				uint32_t const mmd_morph_offset_count = in_mmd_morphs[mmd_morph_index].m_offsets.size();
				for (size_t mmd_morph_offset_index = 0U; mmd_morph_offset_index < mmd_morph_offset_count; ++mmd_morph_offset_index)
				{
					uint32_t const parent_index = in_mmd_morphs[mmd_morph_index].m_offsets[mmd_morph_offset_index].m_group.m_morph_index;
					++mmd_morph_parent_count[child_index];
					mmd_morph_children_indices[parent_index].push_back(child_index);
					mmd_morph_children_weights[parent_index].push_back(in_mmd_morphs[mmd_morph_index].m_offsets[mmd_morph_offset_index].m_group.m_morph_weight);
				}
			}
		}

		mcrt_vector<uint32_t> topological_sort_stack;
		for (uint32_t mmd_morph_index_plus_1 = mmd_morph_count; mmd_morph_index_plus_1 > 0U; --mmd_morph_index_plus_1)
		{
			uint32_t const mmd_morph_index = mmd_morph_index_plus_1 - 1U;
			assert(mmd_morph_children_indices[mmd_morph_index].size() == mmd_morph_children_weights[mmd_morph_index].size());
			if (0U == mmd_morph_parent_count[mmd_morph_index])
			{
				topological_sort_stack.push_back(mmd_morph_index);
			}
		}

		mcrt_vector<bool> mmd_morph_visited_flags(static_cast<size_t>(mmd_morph_count), false);
		while (!topological_sort_stack.empty())
		{
			uint32_t const mmd_morph_current_index = topological_sort_stack.back();
			topological_sort_stack.pop_back();

			assert(!mmd_morph_visited_flags[mmd_morph_current_index]);
			mmd_morph_visited_flags[mmd_morph_current_index] = true;

			mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t> &current_morph_target = mmd_morph_targets[in_mmd_morphs[mmd_morph_current_index].m_name];

			if (1U == in_mmd_morphs[mmd_morph_current_index].m_morph_type || 2U == in_mmd_morphs[mmd_morph_current_index].m_morph_type)
			{
				assert(current_morph_target.empty());
				uint32_t const mmd_morph_offset_count = in_mmd_morphs[mmd_morph_current_index].m_offsets.size();
				for (size_t mmd_morph_offset_index = 0U; mmd_morph_offset_index < mmd_morph_offset_count; ++mmd_morph_offset_index)
				{
					internal_mmd_morph_target_vertex_t morph_target_vertex;
					if (1U == in_mmd_morphs[mmd_morph_current_index].m_morph_type)
					{
						mmd_pmx_vec3_t const vertex_position = in_mmd_morphs[mmd_morph_current_index].m_offsets[mmd_morph_offset_index].m_vertex_position.m_vertex_position;

						// TODO: remove this
						glm::vec3 temp = saba::InvZ(glm::vec3(vertex_position.m_x, vertex_position.m_y, vertex_position.m_z));

						morph_target_vertex = internal_mmd_morph_target_vertex_t{{temp.x, temp.y, temp.z}, {0.0F, 0.0F}};
					}
					else
					{
						assert(2U == in_mmd_morphs[mmd_morph_current_index].m_morph_type);
						mmd_pmx_vec2_t const vertex_uv = in_mmd_morphs[mmd_morph_current_index].m_offsets[mmd_morph_offset_index].m_vertex_uv.m_vertex_uv;
						morph_target_vertex = internal_mmd_morph_target_vertex_t{{0.0F, 0.0F, 0.0F}, {vertex_uv.m_x, vertex_uv.m_y}};
					}
					uint32_t const vertex_index = in_mmd_morphs[mmd_morph_current_index].m_offsets[mmd_morph_offset_index].m_vertex_position.m_vertex_index;

					auto found_vertex_index = current_morph_target.find(vertex_index);
					if (current_morph_target.end() == found_vertex_index)
					{
						current_morph_target.emplace_hint(found_vertex_index, vertex_index, morph_target_vertex);
					}
					else
					{
						assert(current_morph_target[vertex_index].m_position[0] == morph_target_vertex.m_position[0]);
						assert(current_morph_target[vertex_index].m_position[1] == morph_target_vertex.m_position[1]);
						assert(current_morph_target[vertex_index].m_position[2] == morph_target_vertex.m_position[2]);
						assert(current_morph_target[vertex_index].m_uv[0] == morph_target_vertex.m_uv[0]);
						assert(current_morph_target[vertex_index].m_uv[1] == morph_target_vertex.m_uv[1]);
					}
				}
			}
			else if (0U == in_mmd_morphs[mmd_morph_current_index].m_morph_type)
			{
#ifndef NDEBUG
				if (current_morph_target.empty())
				{
					uint32_t const mmd_morph_offset_count = in_mmd_morphs[mmd_morph_current_index].m_offsets.size();
					for (size_t mmd_morph_offset_index = 0U; mmd_morph_offset_index < mmd_morph_offset_count; ++mmd_morph_offset_index)
					{
						uint32_t const mmd_morph_parent_index = in_mmd_morphs[mmd_morph_current_index].m_offsets[mmd_morph_offset_index].m_group.m_morph_index;
						assert(mmd_morph_targets[in_mmd_morphs[mmd_morph_parent_index].m_name].empty());
					}
				}
#endif
			}
			else
			{
				assert(3U == in_mmd_morphs[mmd_morph_current_index].m_morph_type);
			}

			assert(mmd_morph_children_indices[mmd_morph_current_index].size() == mmd_morph_children_weights[mmd_morph_current_index].size());
			for (uint32_t mmd_morph_child_index_index_plus_1 = static_cast<uint32_t>(mmd_morph_children_indices[mmd_morph_current_index].size()); mmd_morph_child_index_index_plus_1 > 0U; --mmd_morph_child_index_index_plus_1)
			{
				uint32_t const mmd_morph_child_index = mmd_morph_children_indices[mmd_morph_current_index][mmd_morph_child_index_index_plus_1 - 1U];
				float const mmd_morph_child_weight = mmd_morph_children_weights[mmd_morph_current_index][mmd_morph_child_index_index_plus_1 - 1U];

				mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t> &child_morph_target = mmd_morph_targets[in_mmd_morphs[mmd_morph_child_index].m_name];

				for (auto const &vertex_index_and_morph_target_vertex : current_morph_target)
				{
					uint32_t const vertex_index = vertex_index_and_morph_target_vertex.first;
					internal_mmd_morph_target_vertex_t const morph_target_vertex{
						{
							vertex_index_and_morph_target_vertex.second.m_position[0] * mmd_morph_child_weight,
							vertex_index_and_morph_target_vertex.second.m_position[1] * mmd_morph_child_weight,
							vertex_index_and_morph_target_vertex.second.m_position[2] * mmd_morph_child_weight,
						},
						{
							vertex_index_and_morph_target_vertex.second.m_uv[0] * mmd_morph_child_weight,
							vertex_index_and_morph_target_vertex.second.m_uv[1] * mmd_morph_child_weight,
						}};

					auto found_vertex_index = child_morph_target.find(vertex_index);
					if (child_morph_target.end() == found_vertex_index)
					{
						child_morph_target.emplace_hint(found_vertex_index, vertex_index, morph_target_vertex);
					}
					else
					{
						found_vertex_index->second.m_position[0] += morph_target_vertex.m_position[0];
						found_vertex_index->second.m_position[1] += morph_target_vertex.m_position[1];
						found_vertex_index->second.m_position[2] += morph_target_vertex.m_position[2];
						found_vertex_index->second.m_uv[0] += morph_target_vertex.m_uv[0];
						found_vertex_index->second.m_uv[1] += morph_target_vertex.m_uv[1];
					}
				}

				assert(mmd_morph_parent_count[mmd_morph_child_index] > 0U);
				--mmd_morph_parent_count[mmd_morph_child_index];
				if (0U == mmd_morph_parent_count[mmd_morph_child_index])
				{
					topological_sort_stack.push_back(mmd_morph_child_index);
				}
			}
		}

#ifndef NDEBUG
		for (bool mmd_morph_visited_flag : mmd_morph_visited_flags)
		{
			assert(mmd_morph_visited_flag);
		}
#endif
	}

	mcrt_vector<mcrt_vector<mcrt_string>> mmd_morph_target_name_strings(static_cast<size_t>(BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT));
	{
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_HAPPY].emplace_back(u8"にこり");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_ANGRY].emplace_back(u8"怒り");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_ANGRY].emplace_back(u8"真面目");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_SAD].emplace_back(u8"困る");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_SURPRISED].emplace_back(u8"上");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK].emplace_back(u8"まばたき");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_L].emplace_back(u8"ウィンク２");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_L].emplace_back(u8"ウィンク");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_R].emplace_back(u8"ｳｨﾝｸ２右");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_R].emplace_back(u8"ウィンク右");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_HAPPY].emplace_back(u8"笑い");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_ANGRY].emplace_back(u8"ｷﾘｯ");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_SAD].emplace_back(u8"じと目");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_SURPRISED].emplace_back(u8"びっくり");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_A].emplace_back(u8"あ");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_A].emplace_back(u8"あ２");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_I].emplace_back(u8"い");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_U].emplace_back(u8"う");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_E].emplace_back(u8"え");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_O].emplace_back(u8"お");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY].emplace_back(u8"にっこり");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY].emplace_back(u8"にやり");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY].emplace_back(u8"にやり２");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY].emplace_back(u8"∧");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_SAD].emplace_back(u8"口角下げ");
		mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED].emplace_back(u8"▲");
	}

	assert(out_morph_target_names.empty());
	out_morph_target_names = {};

	assert(out_morph_targets.empty());
	out_morph_targets = {};

	for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT; ++mmd_morph_target_name)
	{
		for (mcrt_string const &mmd_morph_target_name_string : mmd_morph_target_name_strings[mmd_morph_target_name])
		{
			auto &found_mmd_name_and_morph_target = mmd_morph_targets.find(mmd_morph_target_name_string);
			if (mmd_morph_targets.end() != found_mmd_name_and_morph_target)
			{
				if (!found_mmd_name_and_morph_target->second.empty())
				{
					out_morph_target_names.push_back(static_cast<BRX_ASSET_IMPORT_MORPH_TARGET_NAME>(mmd_morph_target_name));
					out_morph_targets.push_back(std::move(found_mmd_name_and_morph_target->second));
					break;
				}
				else
				{
					// TODO: support bone morph
					// assert(false);
				}
			}
		}
	}
}

static inline void internal_import_animation_skeleton(mcrt_vector<mmd_pmx_bone_t> const &in_mmd_model_nodes, mcrt_vector<uint32_t> &out_animation_skeleton_joint_parent_indices, mcrt_vector<uint32_t> &out_model_node_to_animation_skeleton_joint_map, mcrt_vector<uint32_t> &out_animation_skeleton_joint_to_model_node_map, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_model_space, mcrt_vector<BRX_ASSET_IMPORT_IK_NAME> &out_ik_names, mcrt_vector<brx_asset_import_skeleton_joint_constraint> &out_animation_skeleton_joint_constraints, mcrt_vector<mcrt_vector<uint32_t>> &out_animation_skeleton_joint_constraints_storage)
{
	uint32_t const model_node_count = in_mmd_model_nodes.size();

	assert(out_animation_skeleton_joint_parent_indices.empty());
	out_animation_skeleton_joint_parent_indices = {};
	assert(out_model_node_to_animation_skeleton_joint_map.empty());
	out_model_node_to_animation_skeleton_joint_map = mcrt_vector<uint32_t>(static_cast<size_t>(model_node_count), BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
	assert(out_animation_skeleton_joint_to_model_node_map.empty());
	out_animation_skeleton_joint_to_model_node_map = mcrt_vector<uint32_t>(static_cast<size_t>(model_node_count), BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
	{
		mcrt_vector<uint32_t> model_node_parent_indices(static_cast<size_t>(model_node_count));
		for (size_t model_node_index = 0U; model_node_index < model_node_count; ++model_node_index)
		{
			model_node_parent_indices[model_node_index] = in_mmd_model_nodes[model_node_index].m_parent_index;
		}

		mcrt_vector<uint32_t> model_node_depth_first_search_stack;
		mcrt_vector<mcrt_vector<uint32_t>> model_node_children_indices(static_cast<size_t>(model_node_count));
		for (uint32_t model_node_index_plus_1 = model_node_count; model_node_index_plus_1 > 0U; --model_node_index_plus_1)
		{
			uint32_t const model_node_index = model_node_index_plus_1 - 1U;
			uint32_t model_node_parent_index = model_node_parent_indices[model_node_index];
			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != model_node_parent_index)
			{
				model_node_children_indices[model_node_parent_index].push_back(model_node_index);
			}
			else
			{
				model_node_depth_first_search_stack.push_back(model_node_index);
			}
		}
		assert(!model_node_depth_first_search_stack.empty());

		mcrt_vector<bool> model_node_visited_flags(static_cast<size_t>(model_node_count), false);
		mcrt_vector<bool> model_node_pushed_flags(static_cast<size_t>(model_node_count), false);
		while (!model_node_depth_first_search_stack.empty())
		{
			uint32_t const model_node_current_index = model_node_depth_first_search_stack.back();
			model_node_depth_first_search_stack.pop_back();

			assert(!model_node_visited_flags[model_node_current_index]);
			model_node_visited_flags[model_node_current_index] = true;

			uint32_t const animation_skeleton_joint_current_index = out_animation_skeleton_joint_parent_indices.size();

			uint32_t const model_node_parent_index = model_node_parent_indices[model_node_current_index];

			if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == model_node_parent_index)
			{
				out_animation_skeleton_joint_parent_indices.push_back(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
			}
			else
			{
				assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != out_model_node_to_animation_skeleton_joint_map[model_node_parent_index]);
				out_animation_skeleton_joint_parent_indices.push_back(out_model_node_to_animation_skeleton_joint_map[model_node_parent_index]);
			}

			assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == out_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_current_index]);
			out_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_current_index] = model_node_current_index;

			assert(BRX_ASSET_IMPORT_UINT32_INDEX_INVALID == out_model_node_to_animation_skeleton_joint_map[model_node_current_index]);
			out_model_node_to_animation_skeleton_joint_map[model_node_current_index] = animation_skeleton_joint_current_index;

			for (uint32_t model_node_child_index_index_plus_1 = static_cast<uint32_t>(model_node_children_indices[model_node_current_index].size()); model_node_child_index_index_plus_1 > 0U; --model_node_child_index_index_plus_1)
			{
				uint32_t const model_node_child_index = model_node_children_indices[model_node_current_index][model_node_child_index_index_plus_1 - 1U];

				if ((!model_node_visited_flags[model_node_child_index]) && (!model_node_pushed_flags[model_node_child_index]))
				{
					model_node_pushed_flags[model_node_child_index] = true;
					model_node_depth_first_search_stack.push_back(model_node_child_index);
				}
				else
				{
					assert(false);
				}
			}
		}

		assert(out_animation_skeleton_joint_parent_indices.size() == model_node_count);
	}

	assert(out_animation_skeleton_bind_pose_local_space.empty());
	out_animation_skeleton_bind_pose_local_space = mcrt_vector<DirectX::XMFLOAT4X4>(static_cast<size_t>(model_node_count));
	assert(out_animation_skeleton_bind_pose_model_space.empty());
	out_animation_skeleton_bind_pose_model_space = mcrt_vector<DirectX::XMFLOAT4X4>(static_cast<size_t>(model_node_count));
	for (size_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < model_node_count; ++current_animation_skeleton_joint_index)
	{
		{
			uint32_t const current_model_node_index = out_animation_skeleton_joint_to_model_node_map[current_animation_skeleton_joint_index];

			// TODO: remove this
			DirectX::XMFLOAT3 node_translation_model_space;
			{
				glm::vec3 temp = saba::InvZ(glm::vec3(in_mmd_model_nodes[current_model_node_index].m_translation.m_x, in_mmd_model_nodes[current_model_node_index].m_translation.m_y, in_mmd_model_nodes[current_model_node_index].m_translation.m_z));
				node_translation_model_space.x = temp.x;
				node_translation_model_space.y = temp.y;
				node_translation_model_space.z = temp.z;
			}

			DirectX::XMStoreFloat4x4(&out_animation_skeleton_bind_pose_model_space[current_animation_skeleton_joint_index], DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(&node_translation_model_space)));
		}

		uint32_t const parent_animation_skeleton_joint_index = out_animation_skeleton_joint_parent_indices[current_animation_skeleton_joint_index];
		if (BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
		{
			assert(parent_animation_skeleton_joint_index < current_animation_skeleton_joint_index);

			DirectX::XMVECTOR unused_determinant;
			DirectX::XMStoreFloat4x4(&out_animation_skeleton_bind_pose_local_space[current_animation_skeleton_joint_index], DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&out_animation_skeleton_bind_pose_model_space[current_animation_skeleton_joint_index]), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&out_animation_skeleton_bind_pose_model_space[parent_animation_skeleton_joint_index]))));
		}
		else
		{
			out_animation_skeleton_bind_pose_local_space[current_animation_skeleton_joint_index] = out_animation_skeleton_bind_pose_model_space[current_animation_skeleton_joint_index];
		}
	}

	assert(out_ik_names.empty());
	out_ik_names = {};
	assert(out_animation_skeleton_joint_constraints.empty());
	out_animation_skeleton_joint_constraints = {};
	assert(out_animation_skeleton_joint_constraints_storage.empty());
	out_animation_skeleton_joint_constraints_storage = {};
	{
		mcrt_vector<uint32_t> sorted_model_node_indices(static_cast<size_t>(model_node_count));
		for (uint32_t model_node_index = 0U; model_node_index < model_node_count; ++model_node_index)
		{
			sorted_model_node_indices[model_node_index] = model_node_index;
		}

		std::stable_sort(sorted_model_node_indices.begin(), sorted_model_node_indices.end(),
						 [&in_mmd_model_nodes](uint32_t x, uint32_t y)
						 {
							 if ((in_mmd_model_nodes[x].m_meta_physics && in_mmd_model_nodes[y].m_meta_physics) || ((!in_mmd_model_nodes[x].m_meta_physics) && (!in_mmd_model_nodes[y].m_meta_physics)))
							 {
								 return in_mmd_model_nodes[x].m_transformation_hierarchy < in_mmd_model_nodes[y].m_transformation_hierarchy;
							 }
							 else if ((!in_mmd_model_nodes[x].m_meta_physics) && in_mmd_model_nodes[y].m_meta_physics)
							 {
								 return true;
							 }
							 else
							 {
								 assert(in_mmd_model_nodes[x].m_meta_physics && (!in_mmd_model_nodes[y].m_meta_physics));
								 return false;
							 }
						 });

		mcrt_vector<mcrt_vector<mcrt_string>> mmd_ik_name_strings(static_cast<size_t>(BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT));
		{
			mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_RIGHT_FOOT].emplace_back(u8"右足ＩＫ");
			mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_RIGHT_TOE].emplace_back(u8"右つま先ＩＫ");
			mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_LEFT_FOOT].emplace_back(u8"左足ＩＫ");
			mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_LEFT_TOE].emplace_back(u8"左つま先ＩＫ");
		}

		for (uint32_t sorted_model_node_index = 0U; sorted_model_node_index < model_node_count; ++sorted_model_node_index)
		{
			uint32_t const model_node_index = sorted_model_node_indices[sorted_model_node_index];

			if (in_mmd_model_nodes[model_node_index].m_append_rotation || in_mmd_model_nodes[model_node_index].m_append_translation)
			{
				brx_asset_import_skeleton_joint_constraint animation_skeleton_joint_copy_transform_constraint;
				animation_skeleton_joint_copy_transform_constraint.m_constraint_type = BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_COPY_TRANSFORM;
				animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_copy_rotation = in_mmd_model_nodes[model_node_index].m_append_rotation;
				animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_copy_translation = in_mmd_model_nodes[model_node_index].m_append_translation;
				animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_destination_joint_index = out_model_node_to_animation_skeleton_joint_map[model_node_index];

				if (!in_mmd_model_nodes[model_node_index].m_append_local)
				{
					mcrt_vector<uint32_t> ancestors;
					ancestors.push_back(model_node_index);
					for (uint32_t current_model_node_index = ((in_mmd_model_nodes[model_node_index].m_append_rotation || in_mmd_model_nodes[model_node_index].m_append_translation) ? in_mmd_model_nodes[model_node_index].m_append_parent_index : BRX_ASSET_IMPORT_UINT32_INDEX_INVALID);
						 BRX_ASSET_IMPORT_UINT32_INDEX_INVALID != current_model_node_index;
						 current_model_node_index = (((in_mmd_model_nodes[current_model_node_index].m_append_rotation || in_mmd_model_nodes[current_model_node_index].m_append_translation) && (current_model_node_index != in_mmd_model_nodes[current_model_node_index].m_append_parent_index)) ? in_mmd_model_nodes[current_model_node_index].m_append_parent_index : BRX_ASSET_IMPORT_UINT32_INDEX_INVALID))
					{
						ancestors.push_back(current_model_node_index);
					}

					assert(!ancestors.empty());

					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_joint_index = out_model_node_to_animation_skeleton_joint_map[ancestors.back()];

					ancestors.pop_back();

					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weight_count = ancestors.size();

					out_animation_skeleton_joint_constraints_storage.emplace_back();
					out_animation_skeleton_joint_constraints_storage.back().resize(animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weight_count);
					static_assert(sizeof(float) == sizeof(uint32_t), "");
					static_assert(alignof(float) == alignof(uint32_t), "");
					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weights = reinterpret_cast<float *>(out_animation_skeleton_joint_constraints_storage.back().data());

					for (uint32_t source_weight_index = 0U; !ancestors.empty(); ++source_weight_index)
					{
						animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weights[source_weight_index] = in_mmd_model_nodes[ancestors.back()].m_append_rate;
						ancestors.pop_back();
					}
				}
				else
				{
					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_joint_index = out_model_node_to_animation_skeleton_joint_map[in_mmd_model_nodes[model_node_index].m_append_parent_index];
					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weight_count = 1U;

					out_animation_skeleton_joint_constraints_storage.emplace_back();
					out_animation_skeleton_joint_constraints_storage.back().resize(animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weight_count);
					static_assert(sizeof(float) == sizeof(uint32_t), "");
					static_assert(alignof(float) == alignof(uint32_t), "");
					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weights = reinterpret_cast<float *>(out_animation_skeleton_joint_constraints_storage.back().data());

					animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_source_weights[0] = in_mmd_model_nodes[model_node_index].m_append_rate;
				}

				out_animation_skeleton_joint_constraints.push_back(animation_skeleton_joint_copy_transform_constraint);
			}

			if (in_mmd_model_nodes[model_node_index].m_ik)
			{
				{
					bool found_ik_name = false;
					for (uint32_t mmd_ik_name = 0U; mmd_ik_name < BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT; ++mmd_ik_name)
					{
						for (mcrt_string const &mmd_ik_name_string : mmd_ik_name_strings[mmd_ik_name])
						{
							if (mmd_ik_name_string == in_mmd_model_nodes[model_node_index].m_name)
							{

								out_ik_names.push_back(static_cast<BRX_ASSET_IMPORT_IK_NAME>(mmd_ik_name));
								found_ik_name = true;
								break;
							}
						}
					}

					if (!found_ik_name)
					{
						assert(false);
						out_ik_names.push_back(BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT);
					}
				}

				{
					brx_asset_import_skeleton_joint_constraint animation_skeleton_joint_inverse_kinematics_constraint;
					animation_skeleton_joint_inverse_kinematics_constraint.m_constraint_type = BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_INVERSE_KINEMATICS;
					animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_target_joint_index = out_model_node_to_animation_skeleton_joint_map[model_node_index];
					animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_end_effector_index = out_model_node_to_animation_skeleton_joint_map[in_mmd_model_nodes[model_node_index].m_ik_end_effector_index];
					animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_count = in_mmd_model_nodes[model_node_index].m_ik_link_indices.size();

					out_animation_skeleton_joint_constraints_storage.emplace_back();
					out_animation_skeleton_joint_constraints_storage.back().resize(animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_count);
					animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_indices = out_animation_skeleton_joint_constraints_storage.back().data();

					for (uint32_t ik_joint_index = 0U; ik_joint_index < animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_count; ++ik_joint_index)
					{
						animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_indices[ik_joint_index] = out_model_node_to_animation_skeleton_joint_map[in_mmd_model_nodes[model_node_index].m_ik_link_indices[ik_joint_index]];
					}

					if (2U == animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_count)
					{
						DirectX::XMFLOAT3 hinge_joint_normal_local_space;
						{
							DirectX::XMFLOAT4X4 const end_effector_transform_local_space = out_animation_skeleton_bind_pose_local_space[animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_end_effector_index];

							constexpr uint32_t const ball_and_socket_ik_joint_index = 0U;
							constexpr uint32_t const hinge_ik_joint_index = 1U;

							uint32_t const ball_and_socket_animation_skeleton_joint_index = animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_indices[ball_and_socket_ik_joint_index];
							uint32_t const hinge_animation_skeleton_joint_index = animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_joint_indices[hinge_ik_joint_index];

							DirectX::XMFLOAT4X4 const ball_and_socket_joint_transform_model_space = out_animation_skeleton_bind_pose_model_space[ball_and_socket_animation_skeleton_joint_index];

							DirectX::XMFLOAT4X4 const hinge_joint_transform_model_space = out_animation_skeleton_bind_pose_model_space[hinge_animation_skeleton_joint_index];

							constexpr float const INTERNAL_SCALE_EPSILON = 9E-5F;

							DirectX::XMVECTOR ball_and_socket_joint_hinge_joint_local_space_translation;
							{

								DirectX::XMVECTOR unused_determinant;
								DirectX::XMMATRIX ball_and_socket_joint_hinge_joint_local_space_transform = DirectX::XMMatrixMultiply(DirectX::XMLoadFloat4x4(&ball_and_socket_joint_transform_model_space), DirectX::XMMatrixInverse(&unused_determinant, DirectX::XMLoadFloat4x4(&hinge_joint_transform_model_space)));

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
								bool directx_xm_matrix_decompose = DirectX::XMMatrixDecompose(&end_effector_hinge_joint_local_space_scale, &end_effector_hinge_joint_local_space_rotation, &end_effector_hinge_joint_local_space_translation, DirectX::XMLoadFloat4x4(&end_effector_transform_local_space));
								assert(directx_xm_matrix_decompose);

								assert(DirectX::XMVector3EqualInt(DirectX::XMVectorTrueInt(), DirectX::XMVectorLess(DirectX::XMVectorAbs(DirectX::XMVectorSubtract(end_effector_hinge_joint_local_space_scale, DirectX::XMVectorSplatOne())), DirectX::XMVectorReplicate(INTERNAL_SCALE_EPSILON))));
							}

							DirectX::XMStoreFloat3(&hinge_joint_normal_local_space, DirectX::XMVector3Normalize(DirectX::XMVector3Cross(ball_and_socket_joint_hinge_joint_local_space_translation, end_effector_hinge_joint_local_space_translation)));
						}

						DirectX::XMFLOAT3 hinge_joint_axis_local_space;
						float cosine_max_hinge_joint_angle;
						float cosine_min_hinge_joint_angle;
						if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle)
						{
							float rotation_limit_x = std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x));
							float rotation_limit_y = std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y));
							float rotation_limit_z = std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z));

							if ((rotation_limit_x >= rotation_limit_y) && (rotation_limit_x >= rotation_limit_z))
							{
								if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x >= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x >= 0.0F)
								{
									hinge_joint_axis_local_space.x = -1.0F;
									hinge_joint_axis_local_space.y = 0.0F;
									hinge_joint_axis_local_space.z = 0.0F;

									cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x)));
									cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x)));
								}
								else if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x <= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x <= 0.0F)
								{
									hinge_joint_axis_local_space.x = 1.0F;
									hinge_joint_axis_local_space.y = 0.0F;
									hinge_joint_axis_local_space.z = 0.0F;

									cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x)));
									cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x)));
								}
								else
								{
									float rotation_limit_min = std::min(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x, in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x);
									float rotation_limit_max = std::max(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x, in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x);
									assert(rotation_limit_min <= 0.0F);
									assert(rotation_limit_max >= 0.0F);

									if (std::abs(rotation_limit_max) >= std::abs(rotation_limit_min))
									{
										hinge_joint_axis_local_space.x = -1.0F;
										hinge_joint_axis_local_space.y = 0.0F;
										hinge_joint_axis_local_space.z = 0.0F;

										cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_max));
										cosine_min_hinge_joint_angle = 1.0F;
									}
									else
									{
										hinge_joint_axis_local_space.x = 1.0F;
										hinge_joint_axis_local_space.y = 0.0F;
										hinge_joint_axis_local_space.z = 0.0F;

										cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_min));
										cosine_min_hinge_joint_angle = 1.0F;
									}
								}
							}
							else if ((rotation_limit_y >= rotation_limit_z) && (rotation_limit_y >= rotation_limit_x))
							{
								if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y >= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y >= 0.0F)
								{
									hinge_joint_axis_local_space.x = 0.0F;
									hinge_joint_axis_local_space.y = -1.0F;
									hinge_joint_axis_local_space.z = 0.0F;

									cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y)));
									cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y)));
								}
								else if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y <= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y <= 0.0F)
								{
									hinge_joint_axis_local_space.x = 0.0F;
									hinge_joint_axis_local_space.y = 1.0F;
									hinge_joint_axis_local_space.z = 0.0F;

									cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y)));
									cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y)));
								}
								else
								{
									float rotation_limit_min = std::min(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y, in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y);
									float rotation_limit_max = std::max(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y, in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y);
									assert(rotation_limit_min <= 0.0F);
									assert(rotation_limit_max >= 0.0F);

									if (std::abs(rotation_limit_max) >= std::abs(rotation_limit_min))
									{
										hinge_joint_axis_local_space.x = 0.0F;
										hinge_joint_axis_local_space.y = -1.0F;
										hinge_joint_axis_local_space.z = 0.0F;

										cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_max));
										cosine_min_hinge_joint_angle = 1.0F;
									}
									else
									{
										hinge_joint_axis_local_space.x = 0.0F;
										hinge_joint_axis_local_space.y = 1.0F;
										hinge_joint_axis_local_space.z = 0.0F;

										cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_min));
										cosine_min_hinge_joint_angle = 1.0F;
									}
								}
							}
							else
							{
								assert((rotation_limit_z >= rotation_limit_x) && (rotation_limit_z >= rotation_limit_y));

								if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z >= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z >= 0.0F)
								{
									hinge_joint_axis_local_space.x = 0.0F;
									hinge_joint_axis_local_space.y = 0.0F;
									hinge_joint_axis_local_space.z = -1.0F;

									cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z)));
									cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z)));
								}
								else if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z <= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z <= 0.0F)
								{
									hinge_joint_axis_local_space.x = 0.0F;
									hinge_joint_axis_local_space.y = 0.0F;
									hinge_joint_axis_local_space.z = 1.0F;

									cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z)));
									cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z)));
								}
								else
								{
									float rotation_limit_min = std::min(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z, in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z);
									float rotation_limit_max = std::max(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z, in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z);
									assert(rotation_limit_min <= 0.0F);
									assert(rotation_limit_max >= 0.0F);

									if (std::abs(rotation_limit_max) >= std::abs(rotation_limit_min))
									{
										hinge_joint_axis_local_space.x = 0.0F;
										hinge_joint_axis_local_space.y = 0.0F;
										hinge_joint_axis_local_space.z = -1.0F;

										cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_max));
										cosine_min_hinge_joint_angle = 1.0F;
									}
									else
									{
										hinge_joint_axis_local_space.x = 0.0F;
										hinge_joint_axis_local_space.y = 0.0F;
										hinge_joint_axis_local_space.z = 1.0F;

										cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_min));
										cosine_min_hinge_joint_angle = 1.0F;
									}
								}
							}

							if (DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMLoadFloat3(&hinge_joint_axis_local_space), DirectX::XMLoadFloat3(&hinge_joint_normal_local_space))) < (1.0F - 5E-2F))
							{
								assert(false);
								hinge_joint_axis_local_space = hinge_joint_normal_local_space;
								cosine_max_hinge_joint_angle = -1.0F;
								cosine_min_hinge_joint_angle = 1.0F;
							}
						}
						else
						{
							hinge_joint_axis_local_space = hinge_joint_normal_local_space;
							cosine_max_hinge_joint_angle = -1.0F;
							cosine_min_hinge_joint_angle = 1.0F;
						}

						animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[0] = hinge_joint_axis_local_space.x;
						animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[1] = hinge_joint_axis_local_space.y;
						animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[2] = hinge_joint_axis_local_space.z;
						animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_cosine_max_ik_two_joints_hinge_joint_angle = cosine_max_hinge_joint_angle;

						animation_skeleton_joint_inverse_kinematics_constraint.m_inverse_kinematics.m_cosine_min_ik_two_joints_hinge_joint_angle = cosine_min_hinge_joint_angle;
					}

					out_animation_skeleton_joint_constraints.push_back(animation_skeleton_joint_inverse_kinematics_constraint);
				}
			}
		}
	}
}
