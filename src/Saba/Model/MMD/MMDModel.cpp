//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "MMDModel.h"
#include "MMDPhysics.h"
#include "VPDFile.h"
#include "VMDAnimation.h"

#include <glm/gtc/matrix_transform.hpp>

#include <Saba/Base/Log.h>

namespace saba
{
	void MMDModel::UpdateAllAnimation(VMDAnimation *vmdAnim, float vmdFrame, float physicsElapsed)
	{
		if (vmdAnim != nullptr)
		{
			vmdAnim->Evaluate(vmdFrame);
		}

		UpdateMorphAnimation(vmdAnim);

		UpdateNodeAnimation(vmdAnim);
	}

	void MMDModel::LoadPose(const VPDFile &vpd, int frameCount)
	{
		assert(false);
	}

	void MMDModel::UpdateAnimation()
	{
		assert(false);
	}
}
