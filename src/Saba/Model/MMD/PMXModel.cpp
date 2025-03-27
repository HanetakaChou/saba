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

static inline void internal_import_animation_skeleton(mcrt_vector<mmd_pmx_bone_t> const &in_mmd_model_nodes, mcrt_vector<uint32_t> &out_animation_skeleton_joint_parent_indices, mcrt_vector<uint32_t> &out_model_node_to_animation_skeleton_joint_map, mcrt_vector<uint32_t> &out_animation_skeleton_joint_to_model_node_map, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_model_space, mcrt_vector<brx_motion_joint_constraint> &out_animation_skeleton_joint_constraints, mcrt_vector<mcrt_vector<uint32_t>> &out_animation_skeleton_joint_constraints_storage);

namespace saba
{
	namespace
	{
		glm::vec3 InvZ(const glm::vec3 &v)
		{
			return v * glm::vec3(1, 1, -1);
		}
		glm::mat3 InvZ(const glm::mat3 &m)
		{
			const glm::mat3 invZ = glm::scale(glm::mat4(1.0f), glm::vec3(1, 1, -1));
			return invZ * m * invZ;
		}
		glm::quat InvZ(const glm::quat &q)
		{
			auto rot0 = glm::mat3_cast(q);
			auto rot1 = InvZ(rot0);
			return glm::quat_cast(rot1);
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

		for (auto &morph : (*m_morphMan.GetMorphs()))
		{
			morph->SetWeight(0);
		}

		EndAnimation();

		// ResetPhysics();
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
		// Morph の処理
		BeginMorphMaterial();

		const auto &morphs = (*m_morphMan.GetMorphs());
		for (size_t i = 0; i < morphs.size(); i++)
		{
			const auto &morph = morphs[i];
			Morph(morph.get(), morph->GetWeight());
		}

		EndMorphMaterial();
	}

	static inline glm::mat4x4 calculate_transform_model_space(uint32_t const *const in_animation_skeleton_joint_parent_indices, glm::mat4x4 const *const in_animation_skeleton_local_space, uint32_t const in_animation_skeleton_joint_index)
	{
		uint32_t current_animation_skeleton_joint_index = in_animation_skeleton_joint_index;

		mcrt_vector<uint32_t> ancestors;
		while (BRX_MOTION_UINT32_INDEX_INVALID != current_animation_skeleton_joint_index)
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
		mcrt_vector<glm::mat4x4> animation_skeleton_local_space(static_cast<size_t>(m_nodeMan.GetNodeCount()));
		for (size_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < m_nodeMan.GetNodeCount(); ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode const *const animation_node = m_nodeMan.GetNode(model_node_index);
			assert(!animation_node->IsDeformAfterPhysics());
			animation_skeleton_local_space[animation_skeleton_joint_index] = animation_node->GetLocalTransform();
		}

		for (brx_motion_joint_constraint const &animation_skeleton_joint_constraint : this->m_animation_skeleton_joint_constraints)
		{
			if (BRX_JOINT_CONSTRAINT_COPY_TRANSFORM == animation_skeleton_joint_constraint.m_constraint_type)
			{
				glm::quat source_rotation_local_space;
				glm::vec3 source_translation_local_space;
				{
					glm::mat4x4 source_transform_local_space = animation_skeleton_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_source_joint_index];

					source_translation_local_space = glm::vec3(source_transform_local_space[3]);

					glm::vec3 scale = glm::vec3(
						glm::length(glm::vec3(source_transform_local_space[0])),
						glm::length(glm::vec3(source_transform_local_space[1])),
						glm::length(glm::vec3(source_transform_local_space[2])));
					assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

					source_rotation_local_space = glm::quat_cast(glm::mat3(
						glm::vec3(source_transform_local_space[0]) / scale.x,
						glm::vec3(source_transform_local_space[1]) / scale.y,
						glm::vec3(source_transform_local_space[2]) / scale.z));
				}

				glm::quat destination_rotation_local_space;
				glm::vec3 destination_translation_local_space;
				{
					glm::mat4x4 destination_transform_local_space = animation_skeleton_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_destination_joint_index];

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
					// bind pose rotation always zero
					// assert(glm::all(glm::epsilonEqual(source_node->GetInitialRotate(), glm::quat(1.0F, 0.0F, 0.0F, 0.0F), 1E-6F)));

					glm::quat append_rotation = source_rotation_local_space;

					for (uint32_t source_weight_index = 0U; source_weight_index < animation_skeleton_joint_constraint.m_copy_transform.m_source_weight_count; ++source_weight_index)
					{
						append_rotation = glm::slerp(glm::quat(1.0F, 0.0F, 0.0F, 0.0F), append_rotation, animation_skeleton_joint_constraint.m_copy_transform.m_source_weights[source_weight_index]);
					}

					destination_rotation_local_space = (destination_rotation_local_space * append_rotation);
				}

				if (animation_skeleton_joint_constraint.m_copy_transform.m_copy_translation)
				{
					assert(false);
				}

				glm::mat4 destination_transform_local_space = glm::translate(glm::mat4(1), destination_translation_local_space) * glm::mat4_cast(destination_rotation_local_space);

				animation_skeleton_local_space[animation_skeleton_joint_constraint.m_copy_transform.m_destination_joint_index] = destination_transform_local_space;
			}
			else
			{
				assert(BRX_JOINT_CONSTRAINT_INVERSE_KINEMATICS == animation_skeleton_joint_constraint.m_constraint_type);

				if (static_cast<MMDIKManager *>(&this->m_ikSolverMan)->GetMMDIKSolver(this->m_nodeMan.GetNode(this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_constraint.m_inverse_kinematics.m_target_joint_index])->GetName())->Enabled())
				{
					glm::vec3 const target_position_model_space = calculate_transform_model_space(this->m_animation_skeleton_joint_parent_indices.data(), animation_skeleton_local_space.data(), animation_skeleton_joint_constraint.m_inverse_kinematics.m_target_joint_index)[3];

					glm::mat4 const end_effector_transform_local_space = animation_skeleton_local_space[animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_end_effector_index];

					uint32_t const ik_joint_count = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_count;

					std::vector<glm::mat4x4> ik_joints_local_space(static_cast<size_t>(ik_joint_count));
					std::vector<glm::mat4x4> ik_joints_model_space(static_cast<size_t>(ik_joint_count));

					// TODO: check parent index consistent
					// TODO: check ik joint index out of bound

					for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
					{
						uint32_t const ik_joint_animation_skeleton_joint_index = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_indices[ik_joint_index];

						ik_joints_local_space[ik_joint_index] = animation_skeleton_local_space[ik_joint_animation_skeleton_joint_index];

						ik_joints_model_space[ik_joint_index] = calculate_transform_model_space(this->m_animation_skeleton_joint_parent_indices.data(), animation_skeleton_local_space.data(), ik_joint_animation_skeleton_joint_index);
					}

					glm::vec3 const two_joints_hinge_joint_axis_local_space(animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[0], animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[1], animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_two_joints_hinge_joint_axis_local_space[2]);
					float const two_joints_cosine_max_hinge_joint_angle = animation_skeleton_joint_constraint.m_inverse_kinematics.m_cosine_max_ik_two_joints_hinge_joint_angle;
					float const two_joints_cosine_min_hinge_joint_angle = animation_skeleton_joint_constraint.m_inverse_kinematics.m_cosine_min_ik_two_joints_hinge_joint_angle;
					MMDIkSolver::Solve(two_joints_hinge_joint_axis_local_space, two_joints_cosine_max_hinge_joint_angle, two_joints_cosine_min_hinge_joint_angle, target_position_model_space, end_effector_transform_local_space, ik_joint_count, ik_joints_local_space.data(), ik_joints_model_space.data());

					for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
					{
						glm::quat updated_current_joint_local_space_rotation = glm::quat_cast(glm::mat3(ik_joints_local_space[ik_joint_index]));

						uint32_t const animation_skeleton_joint_index = animation_skeleton_joint_constraint.m_inverse_kinematics.m_ik_joint_indices[ik_joint_index];

						animation_skeleton_local_space[animation_skeleton_joint_index] = ik_joints_local_space[ik_joint_index];
					}
				}
			}
		}

		mcrt_vector<glm::mat4x4> animation_skeleton_model_space(static_cast<size_t>(m_nodeMan.GetNodeCount()));
		for (uint32_t current_animation_skeleton_joint_index = 0; current_animation_skeleton_joint_index < static_cast<uint32_t>(m_nodeMan.GetNodeCount()); ++current_animation_skeleton_joint_index)
		{
			uint32_t const parent_animation_skeleton_joint_index = this->m_animation_skeleton_joint_parent_indices[current_animation_skeleton_joint_index];
			if (BRX_MOTION_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
			{
				assert(parent_animation_skeleton_joint_index < current_animation_skeleton_joint_index);
				animation_skeleton_model_space[current_animation_skeleton_joint_index] = animation_skeleton_model_space[parent_animation_skeleton_joint_index] * animation_skeleton_local_space[current_animation_skeleton_joint_index];
			}
			else
			{
				animation_skeleton_model_space[current_animation_skeleton_joint_index] = animation_skeleton_local_space[current_animation_skeleton_joint_index];
			}
		}

		if (enablePhysics)
		{
			MMDPhysics *physics = this->GetPhysicsManager()->GetMMDPhysics();

			physics->AnimationToRagdoll(animation_skeleton_model_space.data());

			physics->Update(elapsed);

			physics->RagdollToAnimation(animation_skeleton_model_space.data());
		}

		for (uint32_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < static_cast<uint32_t>(m_nodeMan.GetNodeCount()); ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode *const animation_node = m_nodeMan.GetNode(model_node_index);
			assert(!animation_node->IsDeformAfterPhysics());
			animation_node->SetGlobalTransform(animation_skeleton_model_space[animation_skeleton_joint_index]);
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

		size_t vertexCount = pmx.m_vertices.size();
		m_positions.reserve(vertexCount);
		m_normals.reserve(vertexCount);
		m_uvs.reserve(vertexCount);
		m_vertexBoneInfos.reserve(vertexCount);
		m_bboxMax = glm::vec3(-std::numeric_limits<float>::max());
		m_bboxMin = glm::vec3(std::numeric_limits<float>::max());

		bool warnSDEF = false;
		bool infoQDEF = false;
		for (const auto &v : pmx.m_vertices)
		{
			glm::vec3 pos = InvZ(v.m_position);
			glm::vec3 nor = InvZ(v.m_normal);
			glm::vec2 uv = glm::vec2(v.m_uv.x, 1.0f - v.m_uv.y);
			m_positions.push_back(pos);
			m_normals.push_back(nor);
			m_uvs.push_back(uv);
			VertexBoneInfo vtxBoneInfo;
			if (PMXVertexWeight::SDEF != v.m_weightType)
			{
				vtxBoneInfo.m_boneIndex[0] = v.m_boneIndices[0];
				vtxBoneInfo.m_boneIndex[1] = v.m_boneIndices[1];
				vtxBoneInfo.m_boneIndex[2] = v.m_boneIndices[2];
				vtxBoneInfo.m_boneIndex[3] = v.m_boneIndices[3];

				vtxBoneInfo.m_boneWeight[0] = v.m_boneWeights[0];
				vtxBoneInfo.m_boneWeight[1] = v.m_boneWeights[1];
				vtxBoneInfo.m_boneWeight[2] = v.m_boneWeights[2];
				vtxBoneInfo.m_boneWeight[3] = v.m_boneWeights[3];
			}

			switch (v.m_weightType)
			{
			case PMXVertexWeight::BDEF1:
				vtxBoneInfo.m_skinningType = SkinningType::Weight1;
				break;
			case PMXVertexWeight::BDEF2:
				vtxBoneInfo.m_skinningType = SkinningType::Weight2;
				vtxBoneInfo.m_boneWeight[1] = 1.0f - vtxBoneInfo.m_boneWeight[0];
				break;
			case PMXVertexWeight::BDEF4:
				vtxBoneInfo.m_skinningType = SkinningType::Weight4;
				break;
			case PMXVertexWeight::SDEF:
				if (!warnSDEF)
				{
					SABA_WARN("Use SDEF");
					warnSDEF = true;
				}
				vtxBoneInfo.m_skinningType = SkinningType::SDEF;
				{
					auto i0 = v.m_boneIndices[0];
					auto i1 = v.m_boneIndices[1];
					auto w0 = v.m_boneWeights[0];
					auto w1 = 1.0f - w0;

					auto center = InvZ(v.m_sdefC);
					auto r0 = InvZ(v.m_sdefR0);
					auto r1 = InvZ(v.m_sdefR1);
					auto rw = r0 * w0 + r1 * w1;
					r0 = center + r0 - rw;
					r1 = center + r1 - rw;
					auto cr0 = (center + r0) * 0.5f;
					auto cr1 = (center + r1) * 0.5f;

					vtxBoneInfo.m_sdef.m_boneIndex[0] = v.m_boneIndices[0];
					vtxBoneInfo.m_sdef.m_boneIndex[1] = v.m_boneIndices[1];
					vtxBoneInfo.m_sdef.m_boneWeight = v.m_boneWeights[0];
					vtxBoneInfo.m_sdef.m_sdefC = center;
					vtxBoneInfo.m_sdef.m_sdefR0 = cr0;
					vtxBoneInfo.m_sdef.m_sdefR1 = cr1;
				}
				break;
			case PMXVertexWeight::QDEF:
				vtxBoneInfo.m_skinningType = SkinningType::DualQuaternion;
				if (!infoQDEF)
				{
					SABA_INFO("Use QDEF");
					infoQDEF = true;
				}
				break;
			default:
				vtxBoneInfo.m_skinningType = SkinningType::Weight1;
				SABA_ERROR("Unknown PMX Vertex Weight Type: {}", (int)v.m_weightType);
				break;
			}
			m_vertexBoneInfos.push_back(vtxBoneInfo);

			m_bboxMax = glm::max(m_bboxMax, pos);
			m_bboxMin = glm::min(m_bboxMin, pos);
		}
		m_morphPositions.resize(m_positions.size());
		m_morphUVs.resize(m_positions.size());
		m_updatePositions.resize(m_positions.size());
		m_updateNormals.resize(m_normals.size());
		m_updateUVs.resize(m_uvs.size());

		m_indexElementSize = pmx.m_header.m_vertexIndexSize;
		m_indices.resize(pmx.m_faces.size() * 3 * m_indexElementSize);
		m_indexCount = pmx.m_faces.size() * 3;
		switch (m_indexElementSize)
		{
		case 1:
		{
			int idx = 0;
			uint8_t *indices = (uint8_t *)m_indices.data();
			for (const auto &face : pmx.m_faces)
			{
				for (int i = 0; i < 3; i++)
				{
					auto vi = face.m_vertices[3 - i - 1];
					indices[idx] = (uint8_t)vi;
					idx++;
				}
			}
			break;
		}
		case 2:
		{
			int idx = 0;
			uint16_t *indices = (uint16_t *)m_indices.data();
			for (const auto &face : pmx.m_faces)
			{
				for (int i = 0; i < 3; i++)
				{
					auto vi = face.m_vertices[3 - i - 1];
					indices[idx] = (uint16_t)vi;
					idx++;
				}
			}
			break;
		}
		case 4:
		{
			int idx = 0;
			uint32_t *indices = (uint32_t *)m_indices.data();
			for (const auto &face : pmx.m_faces)
			{
				for (int i = 0; i < 3; i++)
				{
					auto vi = face.m_vertices[3 - i - 1];
					indices[idx] = (uint32_t)vi;
					idx++;
				}
			}
			break;
		}
		default:
			SABA_ERROR("Unsupported Index Size: [{}]", m_indexElementSize);
			return false;
		}

		std::vector<std::string> texturePaths;
		texturePaths.reserve(pmx.m_textures.size());
		for (const auto &pmxTex : pmx.m_textures)
		{
			std::string texPath = PathUtil::Combine(dirPath, pmxTex.m_textureName);
			texturePaths.emplace_back(std::move(texPath));
		}

		// Materialをコピー
		m_materials.reserve(pmx.m_materials.size());
		m_subMeshes.reserve(pmx.m_materials.size());
		uint32_t beginIndex = 0;
		for (const auto &pmxMat : pmx.m_materials)
		{
			MMDMaterial mat;
			mat.m_diffuse = pmxMat.m_diffuse;
			mat.m_alpha = pmxMat.m_diffuse.a;
			mat.m_specularPower = pmxMat.m_specularPower;
			mat.m_specular = pmxMat.m_specular;
			mat.m_ambient = pmxMat.m_ambient;
			mat.m_spTextureMode = MMDMaterial::SphereTextureMode::None;
			mat.m_bothFace = !!((uint8_t)pmxMat.m_drawMode & (uint8_t)PMXDrawModeFlags::BothFace);
			mat.m_edgeFlag = ((uint8_t)pmxMat.m_drawMode & (uint8_t)PMXDrawModeFlags::DrawEdge) == 0 ? 0 : 1;
			mat.m_groundShadow = !!((uint8_t)pmxMat.m_drawMode & (uint8_t)PMXDrawModeFlags::GroundShadow);
			mat.m_shadowCaster = !!((uint8_t)pmxMat.m_drawMode & (uint8_t)PMXDrawModeFlags::CastSelfShadow);
			mat.m_shadowReceiver = !!((uint8_t)pmxMat.m_drawMode & (uint8_t)PMXDrawModeFlags::RecieveSelfShadow);
			mat.m_edgeSize = pmxMat.m_edgeSize;
			mat.m_edgeColor = pmxMat.m_edgeColor;

			// Texture
			if (pmxMat.m_textureIndex != -1)
			{
				mat.m_texture = PathUtil::Normalize(texturePaths[pmxMat.m_textureIndex]);
			}

			// ToonTexture
			if (pmxMat.m_toonMode == PMXToonMode::Common)
			{
				if (pmxMat.m_toonTextureIndex != -1)
				{
					std::stringstream ss;
					ss << "toon" << std::setfill('0') << std::setw(2) << (pmxMat.m_toonTextureIndex + 1) << ".bmp";
					mat.m_toonTexture = PathUtil::Combine(mmdDataDir, ss.str());
				}
			}
			else if (pmxMat.m_toonMode == PMXToonMode::Separate)
			{
				if (pmxMat.m_toonTextureIndex != -1)
				{
					mat.m_toonTexture = PathUtil::Normalize(texturePaths[pmxMat.m_toonTextureIndex]);
				}
			}

			// SpTexture
			if (pmxMat.m_sphereTextureIndex != -1)
			{
				mat.m_spTexture = PathUtil::Normalize(texturePaths[pmxMat.m_sphereTextureIndex]);
				mat.m_spTextureMode = MMDMaterial::SphereTextureMode::None;
				if (pmxMat.m_sphereMode == PMXSphereMode::Mul)
				{
					mat.m_spTextureMode = MMDMaterial::SphereTextureMode::Mul;
				}
				else if (pmxMat.m_sphereMode == PMXSphereMode::Add)
				{
					mat.m_spTextureMode = MMDMaterial::SphereTextureMode::Add;
				}
				else if (pmxMat.m_sphereMode == PMXSphereMode::SubTexture)
				{
					// TODO: SphareTexture が SubTexture の処理
				}
			}

			m_materials.emplace_back(std::move(mat));

			MMDSubMesh subMesh;
			subMesh.m_beginIndex = beginIndex;
			subMesh.m_vertexCount = pmxMat.m_numFaceVertices;
			subMesh.m_materialID = (int)(m_materials.size() - 1);
			m_subMeshes.push_back(subMesh);

			beginIndex = beginIndex + pmxMat.m_numFaceVertices;
		}
		m_initMaterials = m_materials;
		m_mulMaterialFactors.resize(m_materials.size());
		m_addMaterialFactors.resize(m_materials.size());

		// Node
		m_nodeMan.GetNodes()->reserve(pmx.m_bones.size());
		for (const auto &bone : pmx.m_bones)
		{
			auto *node = m_nodeMan.AddNode();
			node->SetName(bone.m_name);

			bool const deformAfterPhysics = !!((uint16_t)bone.m_boneFlag & (uint16_t)PMXBoneFlags::DeformAfterPhysics);
			node->EnableDeformAfterPhysics(deformAfterPhysics);
		}
		m_transforms.resize(m_nodeMan.GetNodeCount());

		// Morph
		for (const auto &pmxMorph : pmx.m_morphs)
		{
			auto morph = m_morphMan.AddMorph();
			morph->SetName(pmxMorph.m_name);
			morph->SetWeight(0.0f);
			morph->m_morphType = MorphType::None;
			if (pmxMorph.m_morphType == PMXMorphType::Position)
			{
				morph->m_morphType = MorphType::Position;
				morph->m_dataIndex = m_positionMorphDatas.size();
				PositionMorphData morphData;
				for (const auto &vtx : pmxMorph.m_positionMorph)
				{
					PositionMorph morphVtx;
					morphVtx.m_index = vtx.m_vertexIndex;
					morphVtx.m_position = vtx.m_position * glm::vec3(1, 1, -1);
					morphData.m_morphVertices.push_back(morphVtx);
				}
				m_positionMorphDatas.emplace_back(std::move(morphData));
			}
			else if (pmxMorph.m_morphType == PMXMorphType::UV)
			{
				morph->m_morphType = MorphType::UV;
				morph->m_dataIndex = m_uvMorphDatas.size();
				UVMorphData morphData;
				for (const auto &uv : pmxMorph.m_uvMorph)
				{
					UVMorph morphUV;
					morphUV.m_index = uv.m_vertexIndex;
					morphUV.m_uv = uv.m_uv;
					morphData.m_morphUVs.push_back(morphUV);
				}
				m_uvMorphDatas.emplace_back(std::move(morphData));
			}
			else if (pmxMorph.m_morphType == PMXMorphType::Material)
			{
				morph->m_morphType = MorphType::Material;
				morph->m_dataIndex = m_materialMorphDatas.size();

				MaterialMorphData materialMorphData;
				materialMorphData.m_materialMorphs = pmxMorph.m_materialMorph;
				m_materialMorphDatas.emplace_back(materialMorphData);
			}
			else if (pmxMorph.m_morphType == PMXMorphType::Bone)
			{
				morph->m_morphType = MorphType::Bone;
				morph->m_dataIndex = m_boneMorphDatas.size();

				BoneMorphData boneMorphData;
				for (const auto &pmxBoneMorphElem : pmxMorph.m_boneMorph)
				{
					BoneMorphElement boneMorphElem;
					boneMorphElem.m_node = m_nodeMan.GetMMDNode(pmxBoneMorphElem.m_boneIndex);
					boneMorphElem.m_position = InvZ(pmxBoneMorphElem.m_position);
					boneMorphElem.m_rotate = InvZ(pmxBoneMorphElem.m_quaternion);
					boneMorphData.m_boneMorphs.push_back(boneMorphElem);
				}
				m_boneMorphDatas.emplace_back(boneMorphData);
			}
			else if (pmxMorph.m_morphType == PMXMorphType::Group)
			{
				morph->m_morphType = MorphType::Group;
				morph->m_dataIndex = m_groupMorphDatas.size();

				GroupMorphData groupMorphData;
				groupMorphData.m_groupMorphs = pmxMorph.m_groupMorph;
				m_groupMorphDatas.emplace_back(groupMorphData);
			}
			else
			{
				SABA_WARN("Not Supported Morp Type({}): [{}]",
						  (uint8_t)pmxMorph.m_morphType,
						  pmxMorph.m_name);
			}
		}

		// Check whether Group Morph infinite loop.
		{
			std::vector<int32_t> groupMorphStack;
			std::function<void(int32_t)> fixInifinitGropuMorph;
			fixInifinitGropuMorph = [this, &fixInifinitGropuMorph, &groupMorphStack](int32_t morphIdx)
			{
				const auto &morphs = (*m_morphMan.GetMorphs());
				const auto &morph = morphs[morphIdx];

				if (morph->m_morphType == MorphType::Group)
				{
					auto &groupMorphData = m_groupMorphDatas[morph->m_dataIndex];
					for (size_t i = 0; i < groupMorphData.m_groupMorphs.size(); i++)
					{
						auto &groupMorph = groupMorphData.m_groupMorphs[i];

						auto findIt = std::find(
							groupMorphStack.begin(),
							groupMorphStack.end(),
							groupMorph.m_morphIndex);
						if (findIt != groupMorphStack.end())
						{
							SABA_WARN("Infinit Group Morph:[{}][{}][{}]",
									  morphIdx, morph->GetName(), i);
							groupMorph.m_morphIndex = -1;
						}
						else
						{
							groupMorphStack.push_back(morphIdx);
							if (groupMorph.m_morphIndex > 0)
								fixInifinitGropuMorph(groupMorph.m_morphIndex);
							else
								SABA_ERROR("Invalid morph index: group={}, morph={}", groupMorph.m_morphIndex, morphIdx);
							groupMorphStack.pop_back();
						}
					}
				}
			};

			for (int32_t morphIdx = 0; morphIdx < int32_t(m_morphMan.GetMorphCount()); morphIdx++)
			{
				fixInifinitGropuMorph(morphIdx);
				groupMorphStack.clear();
			}
		}

		// Physics
		if (!m_physicsMan.Create())
		{
			SABA_ERROR("Create Physics Fail.");
			return false;
		}

		uint32_t const model_node_count = pmx.m_bones.size();

		mcrt_vector<mmd_pmx_bone_t> mmd_model_nodes(static_cast<size_t>(model_node_count));
		for (uint32_t model_node_index = 0U; model_node_index < pmx.m_bones.size(); ++model_node_index)
		{
			mmd_model_nodes[model_node_index].m_name = pmx.m_bones[model_node_index].m_name;
			mmd_model_nodes[model_node_index].m_translation.m_x = pmx.m_bones[model_node_index].m_position.x;
			mmd_model_nodes[model_node_index].m_translation.m_y = pmx.m_bones[model_node_index].m_position.y;
			mmd_model_nodes[model_node_index].m_translation.m_z = pmx.m_bones[model_node_index].m_position.z;
			mmd_model_nodes[model_node_index].m_parent_index = pmx.m_bones[model_node_index].m_parentBoneIndex;
			mmd_model_nodes[model_node_index].m_transformation_hierarchy = pmx.m_bones[model_node_index].m_deformDepth;
			mmd_model_nodes[model_node_index].m_meta_physics = (0 != ((uint16_t)pmx.m_bones[model_node_index].m_boneFlag & (uint16_t)PMXBoneFlags::DeformAfterPhysics));
			mmd_model_nodes[model_node_index].m_append_rotation = (0 != ((uint16_t)pmx.m_bones[model_node_index].m_boneFlag & (uint16_t)PMXBoneFlags::AppendRotate));
			mmd_model_nodes[model_node_index].m_append_translation = (0 != ((uint16_t)pmx.m_bones[model_node_index].m_boneFlag & (uint16_t)PMXBoneFlags::AppendTranslate));
			mmd_model_nodes[model_node_index].m_append_local = (0 != ((uint16_t)pmx.m_bones[model_node_index].m_boneFlag & (uint16_t)PMXBoneFlags::AppendLocal));
			mmd_model_nodes[model_node_index].m_append_parent_index = pmx.m_bones[model_node_index].m_appendBoneIndex;
			mmd_model_nodes[model_node_index].m_append_rate = pmx.m_bones[model_node_index].m_appendWeight;
			mmd_model_nodes[model_node_index].m_ik = (0 != ((uint16_t)pmx.m_bones[model_node_index].m_boneFlag & (uint16_t)PMXBoneFlags::IK));
			mmd_model_nodes[model_node_index].m_ik_end_effector_index = pmx.m_bones[model_node_index].m_ikTargetBoneIndex;

			uint32_t const ik_joint_count = pmx.m_bones[model_node_index].m_ikLinks.size();

			mmd_model_nodes[model_node_index].m_ik_link_indices.resize(ik_joint_count);

			uint32_t current_joint_index_plus_1 = ik_joint_count;
			for (uint32_t ik_joint_index = 0U; ik_joint_index < ik_joint_count; ++ik_joint_index)
			{
				uint32_t const current_joint_index = current_joint_index_plus_1 - 1U;

				mmd_model_nodes[model_node_index].m_ik_link_indices[current_joint_index] = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_ikBoneIndex;

				constexpr uint32_t const hinge_joint_index = 1U;
				if (2U == ik_joint_count && hinge_joint_index == current_joint_index)
				{
					mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle = (0U != pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_enableLimit);

					if (mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle)
					{
						mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_limitMin.x;
						mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_limitMin.y;
						mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_limitMin.z;

						mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_limitMax.x;
						mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_limitMax.y;
						mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z = pmx.m_bones[model_node_index].m_ikLinks[ik_joint_index].m_limitMax.z;
					}
				}

				--current_joint_index_plus_1;
			}
		}

		mcrt_vector<mmd_pmx_rigid_body_t> mmd_rigid_bodies(static_cast<size_t>(pmx.m_rigidbodies.size()));
		for (uint32_t rigid_body_index = 0U; rigid_body_index < pmx.m_rigidbodies.size(); ++rigid_body_index)
		{
			mmd_rigid_bodies[rigid_body_index].m_name = pmx.m_rigidbodies[rigid_body_index].m_name;
			mmd_rigid_bodies[rigid_body_index].m_bone_index = pmx.m_rigidbodies[rigid_body_index].m_boneIndex;
			mmd_rigid_bodies[rigid_body_index].m_collision_filter_group = pmx.m_rigidbodies[rigid_body_index].m_group;
			mmd_rigid_bodies[rigid_body_index].m_collision_filter_mask = pmx.m_rigidbodies[rigid_body_index].m_collisionGroup;
			mmd_rigid_bodies[rigid_body_index].m_shape_type = static_cast<uint32_t>(pmx.m_rigidbodies[rigid_body_index].m_shape);
			mmd_rigid_bodies[rigid_body_index].m_shape_size.m_x = pmx.m_rigidbodies[rigid_body_index].m_shapeSize.x;
			mmd_rigid_bodies[rigid_body_index].m_shape_size.m_y = pmx.m_rigidbodies[rigid_body_index].m_shapeSize.y;
			mmd_rigid_bodies[rigid_body_index].m_shape_size.m_z = pmx.m_rigidbodies[rigid_body_index].m_shapeSize.z;
			mmd_rigid_bodies[rigid_body_index].m_translation.m_x = pmx.m_rigidbodies[rigid_body_index].m_translate.x;
			mmd_rigid_bodies[rigid_body_index].m_translation.m_y = pmx.m_rigidbodies[rigid_body_index].m_translate.y;
			mmd_rigid_bodies[rigid_body_index].m_translation.m_z = pmx.m_rigidbodies[rigid_body_index].m_translate.z;
			mmd_rigid_bodies[rigid_body_index].m_rotation.m_x = pmx.m_rigidbodies[rigid_body_index].m_rotate.x;
			mmd_rigid_bodies[rigid_body_index].m_rotation.m_y = pmx.m_rigidbodies[rigid_body_index].m_rotate.y;
			mmd_rigid_bodies[rigid_body_index].m_rotation.m_z = pmx.m_rigidbodies[rigid_body_index].m_rotate.z;
			mmd_rigid_bodies[rigid_body_index].m_mass = pmx.m_rigidbodies[rigid_body_index].m_mass;
			mmd_rigid_bodies[rigid_body_index].m_linear_damping = pmx.m_rigidbodies[rigid_body_index].m_translateDimmer;
			mmd_rigid_bodies[rigid_body_index].m_angular_damping = pmx.m_rigidbodies[rigid_body_index].m_rotateDimmer;
			mmd_rigid_bodies[rigid_body_index].m_friction = pmx.m_rigidbodies[rigid_body_index].m_friction;
			mmd_rigid_bodies[rigid_body_index].m_restitution = pmx.m_rigidbodies[rigid_body_index].m_repulsion;
			mmd_rigid_bodies[rigid_body_index].m_rigid_body_type = static_cast<uint32_t>(pmx.m_rigidbodies[rigid_body_index].m_op);
		}

		mcrt_vector<mmd_pmx_constraint_t> mmd_joints(static_cast<size_t>(pmx.m_joints.size()));
		for (uint32_t joint_index = 0U; joint_index < pmx.m_joints.size(); ++joint_index)
		{
			mmd_joints[joint_index].m_name = pmx.m_joints[joint_index].m_name;
			mmd_joints[joint_index].m_rigid_body_a_index = pmx.m_joints[joint_index].m_rigidbodyAIndex;
			mmd_joints[joint_index].m_rigid_body_b_index = pmx.m_joints[joint_index].m_rigidbodyBIndex;
			mmd_joints[joint_index].m_translation.m_x = pmx.m_joints[joint_index].m_translate.x;
			mmd_joints[joint_index].m_translation.m_y = pmx.m_joints[joint_index].m_translate.y;
			mmd_joints[joint_index].m_translation.m_z = pmx.m_joints[joint_index].m_translate.z;
			mmd_joints[joint_index].m_rotation.m_x = pmx.m_joints[joint_index].m_rotate.x;
			mmd_joints[joint_index].m_rotation.m_y = pmx.m_joints[joint_index].m_rotate.y;
			mmd_joints[joint_index].m_rotation.m_z = pmx.m_joints[joint_index].m_rotate.z;
			mmd_joints[joint_index].m_translation_limit_min.m_x = pmx.m_joints[joint_index].m_translateLowerLimit.x;
			mmd_joints[joint_index].m_translation_limit_min.m_y = pmx.m_joints[joint_index].m_translateLowerLimit.y;
			mmd_joints[joint_index].m_translation_limit_min.m_z = pmx.m_joints[joint_index].m_translateLowerLimit.z;
			mmd_joints[joint_index].m_translation_limit_max.m_x = pmx.m_joints[joint_index].m_translateUpperLimit.x;
			mmd_joints[joint_index].m_translation_limit_max.m_y = pmx.m_joints[joint_index].m_translateUpperLimit.y;
			mmd_joints[joint_index].m_translation_limit_max.m_z = pmx.m_joints[joint_index].m_translateUpperLimit.z;
			mmd_joints[joint_index].m_rotation_limit_min.m_x = pmx.m_joints[joint_index].m_rotateLowerLimit.x;
			mmd_joints[joint_index].m_rotation_limit_min.m_y = pmx.m_joints[joint_index].m_rotateLowerLimit.y;
			mmd_joints[joint_index].m_rotation_limit_min.m_z = pmx.m_joints[joint_index].m_rotateLowerLimit.z;
			mmd_joints[joint_index].m_rotation_limit_max.m_x = pmx.m_joints[joint_index].m_rotateUpperLimit.x;
			mmd_joints[joint_index].m_rotation_limit_max.m_y = pmx.m_joints[joint_index].m_rotateUpperLimit.y;
			mmd_joints[joint_index].m_rotation_limit_max.m_z = pmx.m_joints[joint_index].m_rotateUpperLimit.z;
		}

		mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_local_space;
		mcrt_vector<DirectX::XMFLOAT4X4> animation_skeleton_bind_pose_model_space;
		internal_import_animation_skeleton(mmd_model_nodes, this->m_animation_skeleton_joint_parent_indices, this->m_model_node_to_animation_skeleton_joint_map, this->m_animation_skeleton_joint_to_model_node_map, animation_skeleton_bind_pose_local_space, animation_skeleton_bind_pose_model_space, this->m_animation_skeleton_joint_constraints, this->m_animation_skeleton_joint_constraints_storage);

		for (size_t animation_skeleton_joint_index = 0; animation_skeleton_joint_index < m_nodeMan.GetNodeCount(); ++animation_skeleton_joint_index)
		{
			uint32_t const model_node_index = this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_index];
			PMXNode *const animation_node = m_nodeMan.GetNode(model_node_index);

			animation_node->SetLocalTransform(*reinterpret_cast<glm::mat4x4 const *>(&animation_skeleton_bind_pose_local_space[animation_skeleton_joint_index]));
			animation_node->SetGlobalTransform(*reinterpret_cast<glm::mat4x4 const *>(&animation_skeleton_bind_pose_model_space[animation_skeleton_joint_index]));
			animation_node->CalculateInverseInitTransform();
			animation_node->SaveInitialTRS();
		}

		for (brx_motion_joint_constraint const &animation_skeleton_joint_constraint : this->m_animation_skeleton_joint_constraints)
		{
			if (BRX_JOINT_CONSTRAINT_INVERSE_KINEMATICS == animation_skeleton_joint_constraint.m_constraint_type)
			{
				m_ikSolverMan.AddIKSolver(this->m_nodeMan.GetNode(this->m_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_constraint.m_inverse_kinematics.m_target_joint_index])->GetName());
			}
		}

		m_physicsMan.GetMMDPhysics()->InitRagdoll(mmd_rigid_bodies, mmd_joints, this->m_model_node_to_animation_skeleton_joint_map.data(), reinterpret_cast<glm::mat4x4 *>(animation_skeleton_bind_pose_model_space.data()));

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

	void PMXModel::Morph(PMXMorph *morph, float weight)
	{
		switch (morph->m_morphType)
		{
		case MorphType::Position:
			MorphPosition(
				m_positionMorphDatas[morph->m_dataIndex],
				weight);
			break;
		case MorphType::UV:
			MorphUV(
				m_uvMorphDatas[morph->m_dataIndex],
				weight);
			break;
		case MorphType::Material:
			MorphMaterial(
				m_materialMorphDatas[morph->m_dataIndex],
				weight);
			break;
		case MorphType::Bone:
			MorphBone(
				m_boneMorphDatas[morph->m_dataIndex],
				weight);
			break;
		case MorphType::Group:
		{
			auto &groupMorphData = m_groupMorphDatas[morph->m_dataIndex];
			for (const auto &groupMorph : groupMorphData.m_groupMorphs)
			{
				if (groupMorph.m_morphIndex == -1)
				{
					continue;
				}
				auto &elemMorph = (*m_morphMan.GetMorphs())[groupMorph.m_morphIndex];
				Morph(elemMorph.get(), groupMorph.m_weight * weight);
			}
			break;
		}
		default:
			break;
		}
	}

	void PMXModel::MorphPosition(const PositionMorphData &morphData, float weight)
	{
		if (weight == 0)
		{
			return;
		}

		for (const auto &morphVtx : morphData.m_morphVertices)
		{
			m_morphPositions[morphVtx.m_index] += morphVtx.m_position * weight;
		}
	}

	void PMXModel::MorphUV(const UVMorphData &morphData, float weight)
	{
		if (weight == 0)
		{
			return;
		}

		for (const auto &morphUV : morphData.m_morphUVs)
		{
			m_morphUVs[morphUV.m_index] += morphUV.m_uv * weight;
		}
	}

	void PMXModel::BeginMorphMaterial()
	{
		MaterialFactor initMul;
		initMul.m_diffuse = glm::vec3(1);
		initMul.m_alpha = 1;
		initMul.m_specular = glm::vec3(1);
		initMul.m_specularPower = 1;
		initMul.m_ambient = glm::vec3(1);
		initMul.m_edgeColor = glm::vec4(1);
		initMul.m_edgeSize = 1;
		initMul.m_textureFactor = glm::vec4(1);
		initMul.m_spTextureFactor = glm::vec4(1);
		initMul.m_toonTextureFactor = glm::vec4(1);

		MaterialFactor initAdd;
		initAdd.m_diffuse = glm::vec3(0);
		initAdd.m_alpha = 0;
		initAdd.m_specular = glm::vec3(0);
		initAdd.m_specularPower = 0;
		initAdd.m_ambient = glm::vec3(0);
		initAdd.m_edgeColor = glm::vec4(0);
		initAdd.m_edgeSize = 0;
		initAdd.m_textureFactor = glm::vec4(0);
		initAdd.m_spTextureFactor = glm::vec4(0);
		initAdd.m_toonTextureFactor = glm::vec4(0);

		size_t matCount = m_materials.size();
		for (size_t matIdx = 0; matIdx < matCount; matIdx++)
		{
			m_mulMaterialFactors[matIdx] = initMul;
			m_mulMaterialFactors[matIdx].m_diffuse = m_initMaterials[matIdx].m_diffuse;
			m_mulMaterialFactors[matIdx].m_alpha = m_initMaterials[matIdx].m_alpha;
			m_mulMaterialFactors[matIdx].m_specular = m_initMaterials[matIdx].m_specular;
			m_mulMaterialFactors[matIdx].m_specularPower = m_initMaterials[matIdx].m_specularPower;
			m_mulMaterialFactors[matIdx].m_ambient = m_initMaterials[matIdx].m_ambient;

			m_addMaterialFactors[matIdx] = initAdd;
		}
	}

	void PMXModel::EndMorphMaterial()
	{
		size_t matCount = m_materials.size();
		for (size_t matIdx = 0; matIdx < matCount; matIdx++)
		{
			MaterialFactor matFactor = m_mulMaterialFactors[matIdx];
			matFactor.Add(m_addMaterialFactors[matIdx], 1.0f);

			m_materials[matIdx].m_diffuse = matFactor.m_diffuse;
			m_materials[matIdx].m_alpha = matFactor.m_alpha;
			m_materials[matIdx].m_specular = matFactor.m_specular;
			m_materials[matIdx].m_specularPower = matFactor.m_specularPower;
			m_materials[matIdx].m_ambient = matFactor.m_ambient;
			m_materials[matIdx].m_textureMulFactor = m_mulMaterialFactors[matIdx].m_textureFactor;
			m_materials[matIdx].m_textureAddFactor = m_addMaterialFactors[matIdx].m_textureFactor;
			m_materials[matIdx].m_spTextureMulFactor = m_mulMaterialFactors[matIdx].m_spTextureFactor;
			m_materials[matIdx].m_spTextureAddFactor = m_addMaterialFactors[matIdx].m_spTextureFactor;
			m_materials[matIdx].m_toonTextureMulFactor = m_mulMaterialFactors[matIdx].m_toonTextureFactor;
			m_materials[matIdx].m_toonTextureAddFactor = m_addMaterialFactors[matIdx].m_toonTextureFactor;
		}
	}

	void PMXModel::MorphMaterial(const MaterialMorphData &morphData, float weight)
	{
		for (const auto &matMorph : morphData.m_materialMorphs)
		{
			if (matMorph.m_materialIndex != -1)
			{
				auto mi = matMorph.m_materialIndex;
				auto &mat = m_materials[mi];
				switch (matMorph.m_opType)
				{
				case saba::PMXMorph::MaterialMorph::OpType::Mul:
					m_mulMaterialFactors[mi].Mul(
						MaterialFactor(matMorph),
						weight);
					break;
				case saba::PMXMorph::MaterialMorph::OpType::Add:
					m_addMaterialFactors[mi].Add(
						MaterialFactor(matMorph),
						weight);
					break;
				default:
					break;
				}
			}
			else
			{
				switch (matMorph.m_opType)
				{
				case saba::PMXMorph::MaterialMorph::OpType::Mul:
					for (size_t i = 0; i < m_materials.size(); i++)
					{
						m_mulMaterialFactors[i].Mul(
							MaterialFactor(matMorph),
							weight);
					}
					break;
				case saba::PMXMorph::MaterialMorph::OpType::Add:
					for (size_t i = 0; i < m_materials.size(); i++)
					{
						m_addMaterialFactors[i].Add(
							MaterialFactor(matMorph),
							weight);
					}
					break;
				default:
					break;
				}
			}
		}
	}

	void PMXModel::MorphBone(const BoneMorphData &morphData, float weight)
	{
		for (auto &boneMorph : morphData.m_boneMorphs)
		{
			auto node = boneMorph.m_node;
			glm::vec3 t = glm::mix(glm::vec3(0), boneMorph.m_position, weight);
			node->SetTranslate(node->GetTranslate() + t);
			glm::quat q = glm::slerp(node->GetRotate(), boneMorph.m_rotate, weight);
			node->SetRotate(q);
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

	PMXModel::MaterialFactor::MaterialFactor(const saba::PMXMorph::MaterialMorph &pmxMat)
	{
		m_diffuse.r = pmxMat.m_diffuse.r;
		m_diffuse.g = pmxMat.m_diffuse.g;
		m_diffuse.b = pmxMat.m_diffuse.b;
		m_alpha = pmxMat.m_diffuse.a;
		m_specular = pmxMat.m_specular;
		m_specularPower = pmxMat.m_specularPower;
		m_ambient = pmxMat.m_ambient;
		m_edgeColor = pmxMat.m_edgeColor;
		m_edgeSize = pmxMat.m_edgeSize;
		m_textureFactor = pmxMat.m_textureFactor;
		m_spTextureFactor = pmxMat.m_sphereTextureFactor;
		m_toonTextureFactor = pmxMat.m_toonTextureFactor;
	}

	void PMXModel::MaterialFactor::Mul(const MaterialFactor &val, float weight)
	{
		m_diffuse = glm::mix(m_diffuse, m_diffuse * val.m_diffuse, weight);
		m_alpha = glm::mix(m_alpha, m_alpha * val.m_alpha, weight);
		m_specular = glm::mix(m_specular, m_specular * val.m_specular, weight);
		m_specularPower = glm::mix(m_specularPower, m_specularPower * val.m_specularPower, weight);
		m_ambient = glm::mix(m_ambient, m_ambient * val.m_ambient, weight);
		m_edgeColor = glm::mix(m_edgeColor, m_edgeColor * val.m_edgeColor, weight);
		m_edgeSize = glm::mix(m_edgeSize, m_edgeSize * val.m_edgeSize, weight);
		m_textureFactor = glm::mix(m_textureFactor, m_textureFactor * val.m_textureFactor, weight);
		m_spTextureFactor = glm::mix(m_spTextureFactor, m_spTextureFactor * val.m_spTextureFactor, weight);
		m_toonTextureFactor = glm::mix(m_toonTextureFactor, m_toonTextureFactor * val.m_toonTextureFactor, weight);
	}

	void PMXModel::MaterialFactor::Add(const MaterialFactor &val, float weight)
	{
		m_diffuse += val.m_diffuse * weight;
		m_alpha += val.m_alpha * weight;
		m_specular += val.m_specular * weight;
		m_specularPower += val.m_specularPower * weight;
		m_ambient += val.m_ambient * weight;
		m_edgeColor += val.m_edgeColor * weight;
		m_edgeSize += val.m_edgeSize * weight;
		m_textureFactor += val.m_textureFactor * weight;
		m_spTextureFactor += val.m_spTextureFactor * weight;
		m_toonTextureFactor += val.m_toonTextureFactor * weight;
	}
}

static inline void internal_import_animation_skeleton(mcrt_vector<mmd_pmx_bone_t> const &in_mmd_model_nodes, mcrt_vector<uint32_t> &out_animation_skeleton_joint_parent_indices, mcrt_vector<uint32_t> &out_model_node_to_animation_skeleton_joint_map, mcrt_vector<uint32_t> &out_animation_skeleton_joint_to_model_node_map, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_local_space, mcrt_vector<DirectX::XMFLOAT4X4> &out_animation_skeleton_bind_pose_model_space, mcrt_vector<brx_motion_joint_constraint> &out_animation_skeleton_joint_constraints, mcrt_vector<mcrt_vector<uint32_t>> &out_animation_skeleton_joint_constraints_storage)
{
	uint32_t const model_node_count = in_mmd_model_nodes.size();

	assert(out_animation_skeleton_joint_parent_indices.empty());
	out_animation_skeleton_joint_parent_indices = {};
	assert(out_model_node_to_animation_skeleton_joint_map.empty());
	out_model_node_to_animation_skeleton_joint_map = mcrt_vector<uint32_t>(static_cast<size_t>(model_node_count), BRX_MOTION_UINT32_INDEX_INVALID);
	assert(out_animation_skeleton_joint_to_model_node_map.empty());
	out_animation_skeleton_joint_to_model_node_map = mcrt_vector<uint32_t>(static_cast<size_t>(model_node_count), BRX_MOTION_UINT32_INDEX_INVALID);
	{
		mcrt_vector<uint32_t> model_node_parent_indices(static_cast<size_t>(model_node_count));
		{
			for (size_t model_node_index = 0; model_node_index < model_node_count; ++model_node_index)
			{
				model_node_parent_indices[model_node_index] = in_mmd_model_nodes[model_node_index].m_parent_index;
			}
		}

		mcrt_vector<uint32_t> model_node_depth_first_search_stack;
		mcrt_vector<mcrt_vector<uint32_t>> model_node_children_indices(static_cast<size_t>(model_node_count));
		for (uint32_t model_node_index_plus_1 = model_node_count; model_node_index_plus_1 > 0U; --model_node_index_plus_1)
		{
			uint32_t const model_node_index = model_node_index_plus_1 - 1U;
			uint32_t model_node_parent_index = model_node_parent_indices[model_node_index];
			if (BRX_MOTION_UINT32_INDEX_INVALID != model_node_parent_index)
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

			if (BRX_MOTION_UINT32_INDEX_INVALID == model_node_parent_index)
			{
				out_animation_skeleton_joint_parent_indices.push_back(BRX_MOTION_UINT32_INDEX_INVALID);
			}
			else
			{
				assert(BRX_MOTION_UINT32_INDEX_INVALID != out_model_node_to_animation_skeleton_joint_map[model_node_parent_index]);
				out_animation_skeleton_joint_parent_indices.push_back(out_model_node_to_animation_skeleton_joint_map[model_node_parent_index]);
			}

			assert(BRX_MOTION_UINT32_INDEX_INVALID == out_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_current_index]);
			out_animation_skeleton_joint_to_model_node_map[animation_skeleton_joint_current_index] = model_node_current_index;

			assert(BRX_MOTION_UINT32_INDEX_INVALID == out_model_node_to_animation_skeleton_joint_map[model_node_current_index]);
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
		if (BRX_MOTION_UINT32_INDEX_INVALID != parent_animation_skeleton_joint_index)
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

		for (uint32_t sorted_model_node_index = 0U; sorted_model_node_index < model_node_count; ++sorted_model_node_index)
		{
			uint32_t const model_node_index = sorted_model_node_indices[sorted_model_node_index];

			if (in_mmd_model_nodes[model_node_index].m_append_rotation || in_mmd_model_nodes[model_node_index].m_append_translation)
			{
				brx_motion_joint_constraint animation_skeleton_joint_copy_transform_constraint;
				animation_skeleton_joint_copy_transform_constraint.m_constraint_type = BRX_JOINT_CONSTRAINT_COPY_TRANSFORM;
				animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_copy_rotation = in_mmd_model_nodes[model_node_index].m_append_rotation;
				animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_copy_translation = in_mmd_model_nodes[model_node_index].m_append_translation;
				animation_skeleton_joint_copy_transform_constraint.m_copy_transform.m_destination_joint_index = out_model_node_to_animation_skeleton_joint_map[model_node_index];

				if (!in_mmd_model_nodes[model_node_index].m_append_local)
				{
					mcrt_vector<uint32_t> ancestors;
					ancestors.push_back(model_node_index);
					for (uint32_t current_model_node_index = ((in_mmd_model_nodes[model_node_index].m_append_rotation || in_mmd_model_nodes[model_node_index].m_append_translation) ? in_mmd_model_nodes[model_node_index].m_append_parent_index : BRX_MOTION_UINT32_INDEX_INVALID); BRX_MOTION_UINT32_INDEX_INVALID != current_model_node_index; current_model_node_index = ((in_mmd_model_nodes[current_model_node_index].m_append_rotation || in_mmd_model_nodes[current_model_node_index].m_append_translation) ? in_mmd_model_nodes[current_model_node_index].m_append_parent_index : BRX_MOTION_UINT32_INDEX_INVALID))
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
				brx_motion_joint_constraint animation_skeleton_joint_inverse_kinematics_constraint;

				animation_skeleton_joint_inverse_kinematics_constraint.m_constraint_type = BRX_JOINT_CONSTRAINT_INVERSE_KINEMATICS;
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
								hinge_joint_axis_local_space.x = 1.0F;
								hinge_joint_axis_local_space.y = 0.0F;
								hinge_joint_axis_local_space.z = 0.0F;

								cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x)));
								cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x)));
							}
							else if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_x <= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_x <= 0.0F)
							{
								hinge_joint_axis_local_space.x = -1.0F;
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
									hinge_joint_axis_local_space.x = 1.0F;
									hinge_joint_axis_local_space.y = 0.0F;
									hinge_joint_axis_local_space.z = 0.0F;

									cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_max));
									cosine_min_hinge_joint_angle = 1.0F;
								}
								else
								{
									hinge_joint_axis_local_space.x = -1.0F;
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
								hinge_joint_axis_local_space.y = 1.0F;
								hinge_joint_axis_local_space.z = 0.0F;

								cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y)));
								cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y)));
							}
							else if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_y <= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_y <= 0.0F)
							{
								hinge_joint_axis_local_space.x = 0.0F;
								hinge_joint_axis_local_space.y = -1.0F;
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
									hinge_joint_axis_local_space.y = 1.0F;
									hinge_joint_axis_local_space.z = 0.0F;

									cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_max));
									cosine_min_hinge_joint_angle = 1.0F;
								}
								else
								{
									hinge_joint_axis_local_space.x = 0.0F;
									hinge_joint_axis_local_space.y = -1.0F;
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
								hinge_joint_axis_local_space.z = 1.0F;

								cosine_max_hinge_joint_angle = std::cos(std::max(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z)));
								cosine_min_hinge_joint_angle = std::cos(std::min(std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z), std::abs(in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z)));
							}
							else if (in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_min.m_z <= 0.0F && in_mmd_model_nodes[model_node_index].m_ik_two_links_hinge_limit_angle_max.m_z <= 0.0F)
							{
								hinge_joint_axis_local_space.x = 0.0F;
								hinge_joint_axis_local_space.y = 0.0F;
								hinge_joint_axis_local_space.z = -1.0F;

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
									hinge_joint_axis_local_space.z = 1.0F;

									cosine_max_hinge_joint_angle = std::cos(std::abs(rotation_limit_max));
									cosine_min_hinge_joint_angle = 1.0F;
								}
								else
								{
									hinge_joint_axis_local_space.x = 0.0F;
									hinge_joint_axis_local_space.y = 0.0F;
									hinge_joint_axis_local_space.z = -1.0F;

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
