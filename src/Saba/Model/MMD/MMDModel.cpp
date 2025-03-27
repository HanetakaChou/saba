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
	MMDPhysicsManager::MMDPhysicsManager()
	{
	}

	MMDPhysicsManager::~MMDPhysicsManager()
	{
		m_mmdPhysics.reset();
	}

	bool MMDPhysicsManager::Create()
	{
		m_mmdPhysics = std::make_unique<MMDPhysics>();
		return m_mmdPhysics->Create();
	}

	MMDPhysics *MMDPhysicsManager::GetMMDPhysics()
	{
		return m_mmdPhysics.get();
	}

	namespace
	{
		glm::vec3 InvZ(const glm::vec3 &v)
		{
			return v;
		}
		glm::mat3 InvZ(const glm::mat3 &m)
		{
			return m;
		}
		glm::quat InvZ(const glm::quat &q)
		{
			return q;
		}
	}

	void MMDModel::UpdateAllAnimation(VMDAnimation *vmdAnim, float vmdFrame, float physicsElapsed)
	{
		if (vmdAnim != nullptr)
		{
			vmdAnim->Evaluate(vmdFrame);
		}

		UpdateMorphAnimation();

		UpdateNodeAnimation(true, physicsElapsed);
	}

	void MMDModel::LoadPose(const VPDFile &vpd, int frameCount)
	{
		assert(false);
	}

	void MMDModel::UpdateAnimation()
	{
		UpdateMorphAnimation();
		UpdateNodeAnimation(true, 0.0f);
	}
}
