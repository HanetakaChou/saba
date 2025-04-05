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

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_input_stream.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Asset-Import/include/brx_asset_import_scene.h"
#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/thirdparty/Brioche-Motion/include/brx_motion.h"

namespace saba
{
	class VMDAnimation
	{
	public:
		VMDAnimation();
		~VMDAnimation();

		bool Create(std::shared_ptr<MMDModel> model);
		bool Add(const char *filename);
		void Destroy();

		void Evaluate(float current_frame_index);

		// Physics を同期させる
		void SyncPhysics(float current_frame_index, int frameCount = 30);

		float get_morph_target_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name) const;

		brx_motion_animation_instance *get_motion_animation_instance() const;

		float m_previous_frame_index;

	private:
		brx_motion_animation *m_motion_animation;
		brx_motion_animation_instance *m_motion_animation_instance;

		std::shared_ptr<MMDModel> m_model;
	};

}

#endif // !SABA_MODEL_MMD_VMDANIMATION_H_
