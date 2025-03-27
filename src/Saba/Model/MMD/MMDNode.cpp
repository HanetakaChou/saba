//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "MMDNode.h"

#include <Saba/Base/Log.h>

#include <glm/gtc/matrix_transform.hpp>

namespace saba
{
	MMDNode::MMDNode()
		: m_index(0), m_translate(0), m_rotate(1, 0, 0, 0), m_scale(1), m_baseAnimTranslate(0), m_baseAnimRotate(1, 0, 0, 0), m_global(1), m_inverseInit(1), m_initTranslate(0), m_initRotate(1, 0, 0, 0), m_initScale(1)
	{
	}

	void MMDNode::BeginUpdateTransform()
	{
		LoadInitialTRS();
		OnBeginUpdateTransform();
	}

	void MMDNode::EndUpdateTransform()
	{
		OnEndUpdateTransfrom();
	}

	void MMDNode::CalculateInverseInitTransform()
	{
		m_inverseInit = glm::inverse(m_global);
	}

	void MMDNode::OnBeginUpdateTransform()
	{
	}

	void MMDNode::OnEndUpdateTransfrom()
	{
	}
}
