//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_MMD_VMDANIMATION_H_
#define SABA_MODEL_MMD_VMDANIMATION_H_

#include "MMDModel.h"
#include "MMDNode.h"
#include "VMDFile.h"
#include "MMDIkSolver.h"

#include <vector>
#include <algorithm>
#include <memory>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace saba
{
	struct VMDBezier
	{
		uint8_t m_packed_k_1_x;
		uint8_t m_packed_k_1_y;
		uint8_t m_packed_k_2_x;
		uint8_t m_packed_k_2_y;

		float XToY(float const x) const;
	};

	class VMDAnimation
	{
	public:
		VMDAnimation();

		bool Create(std::shared_ptr<MMDModel> model);
		bool Add(const VMDFile &vmd);
		void Destroy();

		void Evaluate(float t);

		// Physics を同期させる
		void SyncPhysics(float t, int frameCount = 30);

	private:
		mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> m_weight_channel_names;
		mcrt_vector<float> m_weights;
		mcrt_vector<mcrt_string> m_rigid_transform_channel_names;
		mcrt_vector<brx_asset_import_rigid_transform> m_rigid_transforms;
		mcrt_vector<BRX_ASSET_IMPORT_IK_NAME> m_ik_switch_channel_names;
		mcrt_vector<bool> m_ik_switches;

		uint32_t get_frame_count() const;

		std::shared_ptr<MMDModel> m_model;
	};

}

#endif // !SABA_MODEL_MMD_VMDANIMATION_H_
