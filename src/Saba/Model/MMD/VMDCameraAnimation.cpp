//
// Copyright(c) 2016-2019 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "VMDCameraAnimation.h"
#include "VMDAnimationCommon.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>

namespace saba
{
	VMDCameraAnimation::VMDCameraAnimation()
	{
		Destroy();
	}

	bool VMDCameraAnimation::Create()
	{
		return true;
	}

	void VMDCameraAnimation::Destroy()
	{
	}

	void VMDCameraAnimation::Evaluate(float t)
	{
	}

}
