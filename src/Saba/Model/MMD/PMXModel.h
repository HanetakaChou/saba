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

struct internal_mmd_morph_target_vertex_t
{
	float m_position[3];
	float m_uv[2];
};

namespace saba
{
	class PMXNode : public MMDNode
	{
	public:
		PMXNode();

		void EnableDeformAfterPhysics(bool enable) { m_isDeformAfterPhysics = enable; }
		bool IsDeformAfterPhysics() const { return m_isDeformAfterPhysics; }

	protected:
		void OnBeginUpdateTransform() override;
		void OnEndUpdateTransfrom() override;

	private:
		bool m_isDeformAfterPhysics;
	};

	class PMXModel : public MMDModel
	{
	public:
		PMXModel();
		~PMXModel();

		MMDNodeManager *GetNodeManager() override { return &m_nodeMan; }
		void set_morph_target_name_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name, float weight) override { this->m_morph_target_name_weights[morph_target_name] = weight; }
		float get_morph_target_name_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name) const override { return this->m_morph_target_name_weights[morph_target_name]; };
		void set_ik_name_switch(BRX_ASSET_IMPORT_IK_NAME ik_name, bool _switch) override { this->m_ik_name_switches[ik_name] = _switch; }
		bool get_ik_name_switch(BRX_ASSET_IMPORT_IK_NAME ik_name) const override { return this->m_ik_name_switches[ik_name]; };

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

		// アニメーションの前後で呼ぶ (VMDアニメーションの前後)
		void BeginAnimation() override;
		void EndAnimation() override;
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
		std::vector<glm::vec3> m_updatePositions;
		std::vector<glm::vec3> m_updateNormals;
		std::vector<glm::vec2> m_updateUVs;
		std::vector<glm::mat4> m_transforms;

		std::vector<char> m_indices;
		size_t m_indexCount;
		size_t m_indexElementSize;

		// PositionMorph用
		std::vector<glm::vec3> m_morphPositions;
		std::vector<glm::vec4> m_morphUVs;

		// マテリアルMorph用
		std::vector<MMDMaterial> m_initMaterials;

		glm::vec3 m_bboxMin;
		glm::vec3 m_bboxMax;

		std::vector<MMDMaterial> m_materials;
		std::vector<MMDSubMesh> m_subMeshes;

		mcrt_vector<uint32_t> m_animation_skeleton_joint_parent_indices;
		mcrt_vector<uint32_t> m_model_node_to_animation_skeleton_joint_map;
		mcrt_vector<uint32_t> m_animation_skeleton_joint_to_model_node_map;
		mcrt_vector<brx_asset_import_skeleton_joint_constraint> m_animation_skeleton_joint_constraints;
		mcrt_vector<mcrt_vector<uint32_t>> m_animation_skeleton_joint_constraints_storage;

		MMDNodeManagerT<PMXNode> m_nodeMan;
		MMDPhysicsManager m_physicsMan;

		float m_morph_target_name_weights[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT];
		float m_saved_morph_target_name_weights[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT];

		mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> m_morph_target_names;
		mcrt_vector<mcrt_map<uint32_t, internal_mmd_morph_target_vertex_t>> m_morph_targets;

		bool m_ik_name_switches[BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT];
		float m_saved_ik_name_switches[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT];

		mcrt_vector<BRX_ASSET_IMPORT_IK_NAME> m_ik_names;

		uint32_t m_parallelUpdateCount;
		std::vector<UpdateRange> m_updateRanges;
		std::vector<std::future<void>> m_parallelUpdateFutures;
	};
}

#endif // !SABA_MODEL_MMD_PMXMODEL_H_
