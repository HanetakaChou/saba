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

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_input_stream.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_scene.h"

namespace saba
{
	class VMDAnimation
	{
	public:
		VMDAnimation();

		bool Create(std::shared_ptr<MMDModel> model);
		bool Add(const VMDFile &vmd, const char *filename);
		void Destroy();

		void Evaluate(float t);

		// Physics を同期させる
		void SyncPhysics(float t, int frameCount = 30);

	private:
		mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> m_weight_channel_names;
		mcrt_vector<float> m_weights;
		mcrt_vector<BRX_ASSET_IMPORT_SKELETON_JOINT_NAME> m_rigid_transform_channel_names;
		mcrt_vector<brx_asset_import_rigid_transform> m_rigid_transforms;
		mcrt_vector<BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME> m_switch_channel_names;
		mcrt_vector<bool> m_switches;

		uint32_t get_frame_count() const;

		std::shared_ptr<MMDModel> m_model;
	};

}

#endif // !SABA_MODEL_MMD_VMDANIMATION_H_
