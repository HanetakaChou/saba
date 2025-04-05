//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_MMD_PMXMODEL_H_
#define SABA_MODEL_MMD_PMXMODEL_H_

#include "MMDMaterial.h"
#include "MMDModel.h"
#include "MMDIkSolver.h"
#include "PMXFile.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <future>

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_input_stream.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_scene.h"

namespace saba
{
	class PMXModel : public MMDModel
	{
	public:
		PMXModel();
		~PMXModel();

		void set_morph_target_name_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name, float weight) override { this->m_morph_target_name_weights[morph_target_name] = weight; }
		float get_morph_target_name_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name) const override { return this->m_morph_target_name_weights[morph_target_name]; };
		void set_skeleton_joint_name_rigid_transform(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME skeleton_joint_name, brx_asset_import_rigid_transform const &rigid_transform) override { m_skeleton_joint_name_rigid_transforms[skeleton_joint_name] = rigid_transform; }
		brx_asset_import_rigid_transform get_skeleton_joint_name_rigid_transform(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME skeleton_joint_name) const override { return this->m_skeleton_joint_name_rigid_transforms[skeleton_joint_name]; };
		void set_skeleton_joint_constraint_name_switch(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME skeleton_joint_constraint_name, bool _switch) override { this->m_skeleton_joint_constraint_name_switches[skeleton_joint_constraint_name] = _switch; }
		bool get_skeleton_joint_constraint_name_switch(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME skeleton_joint_constraint_name) const override { return this->m_skeleton_joint_constraint_name_switches[skeleton_joint_constraint_name]; };

		MMDPhysicsManager *GetPhysicsManager() override { return &m_physicsMan; }

		size_t GetVertexCount() const override { return m_positions.size(); }
		const glm::vec3 *GetPositions() const override { return m_positions.data(); }
		const glm::vec3 *GetNormals() const override { return m_normals.data(); }
		const glm::vec2 *GetUVs() const override { return m_uvs.data(); }
		const glm::vec3 *GetUpdatePositions() const override { return m_updatePositions.data(); }
		const glm::vec3 *GetUpdateNormals() const override { return m_updateNormals.data(); }
		const glm::vec2 *GetUpdateUVs() const override { return m_updateUVs.data(); }

		size_t GetIndexElementSize() const override { return m_indexElementSize; }
		size_t GetIndexCount() const override { return m_indexCount; }
		const void *GetIndices() const override { return &m_indices[0]; }

		size_t GetMaterialCount() const override { return m_materials.size(); }
		const MMDMaterial *GetMaterials() const override { return &m_materials[0]; }

		size_t GetSubMeshCount() const override { return m_subMeshes.size(); }
		const MMDSubMesh *GetSubMeshes() const override { return &m_subMeshes[0]; }

		MMDPhysics *GetMMDPhysics() override { return m_physicsMan.GetMMDPhysics(); }

		void InitializeAnimation() override;

		void SaveBaseAnimation() override;
		void LoadBaseAnimation() override;
		void ClearBaseAnimation() override;

		// Morph
		void UpdateMorphAnimation() override;
		// ノードを更新する
		void UpdateNodeAnimation(bool enablePhysics, float elapsed) override;
		// Physicsを更新する
		void ResetPhysics() override;
		// 頂点データーを更新する
		void Update() override;
		void SetParallelUpdateHint(uint32_t parallelCount) override;

		bool Load(const std::string &filepath, const std::string &mmdDataDir);
		void Destroy();

		const glm::vec3 &GetBBoxMin() const { return m_bboxMin; }
		const glm::vec3 &GetBBoxMax() const { return m_bboxMax; }

	public:
		enum class SkinningType
		{
			Weight1,
			Weight2,
			Weight4,
			SDEF,
			DualQuaternion,
		};
		struct VertexBoneInfo
		{
			SkinningType m_skinningType;
			union
			{
				struct
				{
					int32_t m_boneIndex[4];
					float m_boneWeight[4];
				};
				struct
				{
					int32_t m_boneIndex[2];
					float m_boneWeight;

					glm::vec3 m_sdefC;
					glm::vec3 m_sdefR0;
					glm::vec3 m_sdefR1;
				} m_sdef;
			};
		};

	private:
		struct UpdateRange
		{
			size_t m_vertexOffset;
			size_t m_vertexCount;
		};

	private:
		void SetupParallelUpdate();
		void Update(const UpdateRange &range);

	private:
		std::vector<glm::vec3> m_positions;
		std::vector<glm::vec3> m_normals;
		std::vector<glm::vec2> m_uvs;
		std::vector<VertexBoneInfo> m_vertexBoneInfos;

		std::vector<char> m_indices;
		size_t m_indexCount;
		size_t m_indexElementSize;

		std::vector<glm::mat4> m_transforms;

		// PositionMorph用
		std::vector<glm::vec3> m_morphPositions;
		std::vector<glm::vec4> m_morphUVs;

		std::vector<glm::vec3> m_updatePositions;
		std::vector<glm::vec3> m_updateNormals;
		std::vector<glm::vec2> m_updateUVs;

		// マテリアルMorph用
		std::vector<MMDMaterial> m_initMaterials;

		glm::vec3 m_bboxMin;
		glm::vec3 m_bboxMax;

		std::vector<MMDMaterial> m_materials;
		std::vector<MMDSubMesh> m_subMeshes;

		MMDPhysicsManager m_physicsMan;

		// Morph Target
		float m_morph_target_name_weights[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT];
		float m_saved_morph_target_name_weights[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT];

		mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> m_morph_target_names;
		mcrt_vector<mcrt_vector<glm::vec3>> m_morph_targets_vertex_positions;
		mcrt_vector<mcrt_vector<glm::vec2>> m_morph_targets_vertex_uvs;

		// Skeleton Joint
		brx_asset_import_rigid_transform m_skeleton_joint_name_rigid_transforms[BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_COUNT];
		brx_asset_import_rigid_transform m_saved_skeleton_joint_name_rigid_transforms[BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_COUNT];

		mcrt_vector<BRX_ASSET_IMPORT_SKELETON_JOINT_NAME> m_skeleton_joint_names;
		mcrt_vector<uint32_t> m_animation_skeleton_joint_parent_indices;
		mcrt_vector<brx_asset_import_rigid_transform> m_bind_pose_local_space;
		mcrt_vector<glm::mat4x4> m_bind_pose_inverse_model_space;
		
		// Skeleton Joint Constraint
		bool m_skeleton_joint_constraint_name_switches[BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_COUNT];
		float m_saved_skeleton_joint_constraint_name_switches[BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_COUNT];

		mcrt_vector<BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME> m_animation_skeleton_joint_constraint_names;
		mcrt_vector<brx_asset_import_skeleton_joint_constraint> m_animation_skeleton_joint_constraints;
		mcrt_vector<mcrt_vector<uint32_t>> m_animation_skeleton_joint_constraints_storage;

		uint32_t m_parallelUpdateCount;
		std::vector<UpdateRange> m_updateRanges;
		std::vector<std::future<void>> m_parallelUpdateFutures;
	};
}

#endif // !SABA_MODEL_MMD_PMXMODEL_H_
