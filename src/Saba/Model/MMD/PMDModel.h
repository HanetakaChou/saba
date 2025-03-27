//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMD_PMDMODEL_H_
#define SABA_MODEL_MMD_PMDMODEL_H_

#include "MMDMaterial.h"
#include "MMDModel.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <vector>
#include <string>
#include <algorithm>

namespace saba
{
	class PMDModel : public MMDModel
	{
	public:
		MMDNodeManager *GetNodeManager() override
		{
			assert(false);
			return NULL;
		}

		MMDIKManager *GetIKManager() override
		{
			assert(false);
			return NULL;
		}

		MMDMorphManager *GetMorphManager() override
		{
			assert(false);
			return NULL;
		}

		MMDPhysicsManager *GetPhysicsManager() override
		{
			assert(false);
			return NULL;
		}

		size_t GetVertexCount() const override
		{
			assert(false);
			return 0U;
		}

		const glm::vec3 *GetPositions() const override
		{
			assert(false);
			return NULL;
		}

		const glm::vec3 *GetNormals() const override
		{
			assert(false);
			return NULL;
		}

		const glm::vec2 *GetUVs() const override
		{
			assert(false);
			return NULL;
		}

		const glm::vec3 *GetUpdatePositions() const override
		{
			assert(false);
			return NULL;
		}

		const glm::vec3 *GetUpdateNormals() const override
		{
			assert(false);
			return NULL;
		}

		const glm::vec2 *GetUpdateUVs() const override
		{
			assert(false);
			return NULL;
		}

		size_t GetIndexElementSize() const override
		{
			return sizeof(uint16_t);
		}

		size_t GetIndexCount() const override
		{
			assert(false);
			return 0U;
		}

		const void *GetIndices() const override
		{
			assert(false);
			return NULL;
		}

		size_t GetMaterialCount() const override
		{
			assert(false);
			return 0U;
		}

		const MMDMaterial *GetMaterials() const override
		{
			assert(false);
			return NULL;
		}

		size_t GetSubMeshCount() const override
		{
			assert(false);
			return 0U;
		}

		const MMDSubMesh *GetSubMeshes() const override
		{
			assert(false);
			return NULL;
		}

		MMDPhysics *GetMMDPhysics() override
		{
			assert(false);
			return NULL;
		}

		void InitializeAnimation() override;
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
		void SetParallelUpdateHint(uint32_t) override
		{
			assert(false);
		}
	};
}

#endif // !SABA_MODEL_MMD_PMDMODEL_H_
