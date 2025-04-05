//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMD_VMDCAMERAANIMATION_H_
#define SABA_MODEL_MMD_VMDCAMERAANIMATION_H_

#include "MMDCamera.h"
#include "VMDFile.h"

#include <cstdint>
#include <memory>

namespace saba
{
	class VMDCameraAnimation
	{
	public:
		VMDCameraAnimation();

		bool Create();
		void Destroy();

		void Evaluate(float t);

		const MMDCamera &GetCamera() const { return m_camera; }

	private:
		MMDCamera m_camera;
	};

}

#endif // !SABA_MODEL_MMD_VMDCAMERAANIMATION_H_
