//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMD_MMDMODEL_H_
#define SABA_MODEL_MMD_MMDMODEL_H_

#include "MMDNode.h"
#include "MMDIkSolver.h"
#include "MMDMorph.h"

#include <vector>
#include <string>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_input_stream.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_scene.h"

namespace saba
{
	struct MMDMaterial;
	class MMDPhysics;
	class MMDRigidBody;
	class MMDJoint;
	struct VPDFile;

	class MMDPhysicsManager
	{
	public:
		using RigidBodyPtr = std::unique_ptr<MMDRigidBody>;
		using JointPtr = std::unique_ptr<MMDJoint>;

		MMDPhysicsManager();
		~MMDPhysicsManager();

		bool Create();

		MMDPhysics *GetMMDPhysics();

	private:
		std::unique_ptr<MMDPhysics> m_mmdPhysics;
	};

	struct MMDSubMesh
	{
		int m_beginIndex;
		int m_vertexCount;
		int m_materialID;
	};

	class VMDAnimation;

	class MMDModel
	{
	public:
		virtual void set_morph_target_name_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name, float weight) = 0;
		virtual float get_morph_target_name_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name) const = 0;
		virtual void set_skeleton_joint_name_rigid_transform(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME skeleton_joint_name, brx_asset_import_rigid_transform const &rigid_transform) = 0;
		virtual brx_asset_import_rigid_transform get_skeleton_joint_name_rigid_transform(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME skeleton_joint_name) const = 0;
		virtual void set_skeleton_joint_constraint_name_switch(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME skeleton_joint_constraint_name, bool _switch) = 0;
		virtual bool get_skeleton_joint_constraint_name_switch(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME skeleton_joint_constraint_name) const = 0;
		virtual MMDPhysicsManager *GetPhysicsManager() = 0;

		virtual size_t GetVertexCount() const = 0;
		virtual const glm::vec3 *GetPositions() const = 0;
		virtual const glm::vec3 *GetNormals() const = 0;
		virtual const glm::vec2 *GetUVs() const = 0;
		virtual const glm::vec3 *GetUpdatePositions() const = 0;
		virtual const glm::vec3 *GetUpdateNormals() const = 0;
		virtual const glm::vec2 *GetUpdateUVs() const = 0;

		virtual size_t GetIndexElementSize() const = 0;
		virtual size_t GetIndexCount() const = 0;
		virtual const void *GetIndices() const = 0;

		virtual size_t GetMaterialCount() const = 0;
		virtual const MMDMaterial *GetMaterials() const = 0;

		virtual size_t GetSubMeshCount() const = 0;
		virtual const MMDSubMesh *GetSubMeshes() const = 0;

		virtual MMDPhysics *GetMMDPhysics() = 0;

		// ノードを初期化する
		virtual void InitializeAnimation() = 0;

		// ベースアニメーション(アニメーション読み込み時、Physics反映用)
		virtual void SaveBaseAnimation() = 0;
		virtual void LoadBaseAnimation() = 0;
		virtual void ClearBaseAnimation() = 0;

		// Morph
		virtual void UpdateMorphAnimation() = 0;
		// ノードを更新する
		[[deprecated("Please use UpdateAllAnimation() function")]]
		void UpdateAnimation();
		virtual void UpdateNodeAnimation(bool enablePhysics, float elapsed) = 0;
		// Physicsを更新する
		virtual void ResetPhysics() = 0;
		// 頂点を更新する
		virtual void Update() = 0;
		virtual void SetParallelUpdateHint(uint32_t parallelCount) = 0;

		void UpdateAllAnimation(VMDAnimation *vmdAnim, float vmdFrame, float physicsElapsed);
		void LoadPose(const VPDFile &vpd, int frameCount = 30);
	};
}

#endif // !SABA_MODEL_MMD_MMDMODEL_H_
