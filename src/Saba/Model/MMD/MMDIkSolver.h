﻿//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMD_MMDIKSOLVER_H_
#define SABA_MODEL_MMD_MMDIKSOLVER_H_

#include "MMDNode.h"

#include <vector>
#include <string>
#include <glm/vec3.hpp>

namespace saba
{
	class MMDIkSolver
	{
	public:
		MMDIkSolver();

		void SetIKNode(MMDNode *node) { m_ikNode = node; }
		void SetTargetNode(MMDNode *node) { m_ikTarget = node; }
		MMDNode *GetIKNode() const { return m_ikNode; }
		MMDNode *GetTargetNode() const { return m_ikTarget; }
		std::string GetName() const
		{
			if (m_ikNode != nullptr)
			{
				return m_ikNode->GetName();
			}
			else
			{
				return "";
			}
		}

		void SetIterateCount(uint32_t count) { m_iterateCount = count; }
		void SetLimitAngle(float angle) { m_limitAngle = angle; }
		void Enable(bool enable) { m_enable = enable; }
		bool Enabled() { return m_enable; }

		void AddIKChain(MMDNode *node, bool isKnee = false);
		void AddIKChain(
			MMDNode *node,
			bool axisLimit,
			const glm::vec3 &limixMin,
			const glm::vec3 &limitMax);

		void Solve();

		void SaveBaseAnimation() { m_baseAnimEnable = m_enable; }
		void LoadBaseAnimation() { m_enable = m_baseAnimEnable; }
		void ClearBaseAnimation() { m_baseAnimEnable = true; }
		bool GetBaseAnimationEnabled() const { return m_baseAnimEnable; }

	private:
		struct IKChain
		{
			MMDNode *m_node;
			bool m_enableAxisLimit;
			glm::vec3 m_limitMax;
			glm::vec3 m_limitMin;
		};

	private:
		void AddIKChain(IKChain &&chain);
		void SolveCore(std::vector<glm::vec3> &in_prev_angles, glm::mat4 const &in_base_parent_transform_model_space, glm::vec3 const &in_target_position_model_space, glm::mat4 const &in_end_effector_transform_local_space, std::vector<glm::mat4x4> &inout_chain_local_space, std::vector<glm::mat4x4> &inout_chain_model_space);

	private:
		std::vector<IKChain> m_chains;
		MMDNode *m_ikNode;
		MMDNode *m_ikTarget;
		uint32_t m_iterateCount;
		float m_limitAngle;
		bool m_enable;
		bool m_baseAnimEnable;
	};
}

#endif // !SABA_MODEL_MMD_MMDIKSOLVER_H_
