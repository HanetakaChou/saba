//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "MMDIkSolver.h"

#include <algorithm>
#include <functional>
#include <glm/gtc/matrix_transform.hpp>

#include <DirectXMath.h>

static inline DirectX::XMVECTOR XM_CALLCONV internal_compute_shortest_rotation_damped(DirectX::XMVECTOR from, DirectX::XMVECTOR to, float gain);

static inline DirectX::XMVECTOR XM_CALLCONV internal_calculate_perpendicular_vector(DirectX::XMVECTOR in);

namespace saba
{
	MMDIkSolver::MMDIkSolver()
		: m_ikNode(nullptr), m_ikTarget(nullptr), m_iterateCount(1), m_limitAngle(glm::pi<float>() * 2.0f), m_enable(true), m_baseAnimEnable(true)
	{
	}

	void MMDIkSolver::AddIKChain(MMDNode *node, bool isKnee)
	{
		IKChain chain;
		chain.m_node = node;
		chain.m_enableAxisLimit = isKnee;
		if (isKnee)
		{
			chain.m_limitMin = glm::vec3(glm::radians(0.5f), 0, 0);
			chain.m_limitMax = glm::vec3(glm::radians(180.0f), 0, 0);
		}
		AddIKChain(std::move(chain));
	}

	void MMDIkSolver::AddIKChain(
		MMDNode *node,
		bool axisLimit,
		const glm::vec3 &limixMin,
		const glm::vec3 &limitMax)
	{
		IKChain chain;
		chain.m_node = node;
		chain.m_enableAxisLimit = axisLimit;
		chain.m_limitMin = limixMin;
		chain.m_limitMax = limitMax;
		AddIKChain(std::move(chain));
	}

	void MMDIkSolver::AddIKChain(MMDIkSolver::IKChain &&chain)
	{
		m_chains.emplace_back(chain);
	}

	void MMDIkSolver::Solve()
	{
		if (!m_enable)
		{
			return;
		}

		if ((m_ikNode == nullptr) || (m_ikTarget == nullptr))
		{
			// wrong ik
			return;
		}

		std::vector<glm::vec3> in_prev_angles;
		in_prev_angles.resize(m_chains.size(), glm::vec3(0));

		glm::mat4 in_base_parent_transform_model_space = (m_chains.back().m_node->GetParent() != nullptr) ? m_chains.back().m_node->GetParent()->SyncLocalTransformToGlobal() : glm::mat4(1.0F);

		glm::vec3 in_target_position_model_space = m_ikNode->SyncLocalTransformToGlobal()[3];

		glm::mat4 in_end_effector_transform_local_space = m_ikTarget->GetLocalTransform();

		std::vector<glm::mat4x4> inout_chain_local_space;
		std::vector<glm::mat4x4> inout_chain_model_space;

		for (uint32_t chain_index_plus_1 = m_chains.size(); chain_index_plus_1 >= 1U; --chain_index_plus_1)
		{
			inout_chain_local_space.push_back(m_chains[chain_index_plus_1 - 1U].m_node->GetLocalTransform());
			inout_chain_model_space.push_back(m_chains[chain_index_plus_1 - 1U].m_node->SyncLocalTransformToGlobal());
		}

		for (uint32_t i = 0; i < m_iterateCount; ++i)
		{
			SolveCore(in_prev_angles, in_base_parent_transform_model_space, in_target_position_model_space, in_end_effector_transform_local_space, inout_chain_local_space, inout_chain_model_space);
		}

		uint32_t current_joint_chain_index = m_chains.size() - 1;
		for (size_t chainIdx = 0; chainIdx < m_chains.size(); chainIdx++)
		{
			glm::quat updated_current_joint_local_space_rotation = glm::quat_cast(glm::mat3(inout_chain_local_space[current_joint_chain_index]));

			auto &chain = m_chains[chainIdx];
			MMDNode *chainNode = chain.m_node;

			chainNode->SetAnimateRotate(updated_current_joint_local_space_rotation);

			--current_joint_chain_index;
		}
	}

	namespace
	{
		float NormalizeAngle(float angle)
		{
			float ret = angle;
			while (ret >= glm::two_pi<float>())
			{
				ret -= glm::two_pi<float>();
			}
			while (ret < 0)
			{
				ret += glm::two_pi<float>();
			}

			return ret;
		}

		float DiffAngle(float a, float b)
		{
			float diff = NormalizeAngle(a) - NormalizeAngle(b);
			if (diff > glm::pi<float>())
			{
				return diff - glm::two_pi<float>();
			}
			else if (diff < -glm::pi<float>())
			{
				return diff + glm::two_pi<float>();
			}
			return diff;
		}

		glm::vec3 Decompose(const glm::mat3 &m, const glm::vec3 &before)
		{
			glm::vec3 r;
			float sy = -m[0][2];
			const float e = 1E-6F;
			if ((1.0f - std::abs(sy)) < e)
			{
				r.y = std::asin(sy);
				// 180°に近いほうを探す
				float sx = std::sin(before.x);
				float sz = std::sin(before.z);
				if (std::abs(sx) < std::abs(sz))
				{
					// Xのほうが0または180
					float cx = std::cos(before.x);
					if (cx > 0)
					{
						r.x = 0;
						r.z = std::asin(-m[1][0]);
					}
					else
					{
						r.x = glm::pi<float>();
						r.z = std::asin(m[1][0]);
					}
				}
				else
				{
					float cz = std::cos(before.z);
					if (cz > 0)
					{
						r.z = 0;
						r.x = std::asin(-m[2][1]);
					}
					else
					{
						r.z = glm::pi<float>();
						r.x = std::asin(m[2][1]);
					}
				}
			}
			else
			{
				r.x = std::atan2(m[1][2], m[2][2]);
				r.y = std::asin(-m[0][2]);
				r.z = std::atan2(m[0][1], m[0][0]);
			}

			const float pi = glm::pi<float>();
			glm::vec3 tests[] =
				{
					{r.x + pi, pi - r.y, r.z + pi},
					{r.x + pi, pi - r.y, r.z - pi},
					{r.x + pi, -pi - r.y, r.z + pi},
					{r.x + pi, -pi - r.y, r.z - pi},
					{r.x - pi, pi - r.y, r.z + pi},
					{r.x - pi, pi - r.y, r.z - pi},
					{r.x - pi, -pi - r.y, r.z + pi},
					{r.x - pi, -pi - r.y, r.z - pi},
				};

			float errX = std::abs(DiffAngle(r.x, before.x));
			float errY = std::abs(DiffAngle(r.y, before.y));
			float errZ = std::abs(DiffAngle(r.z, before.z));
			float minErr = errX + errY + errZ;
			for (const auto test : tests)
			{
				float err = std::abs(DiffAngle(test.x, before.x)) + std::abs(DiffAngle(test.y, before.y)) + std::abs(DiffAngle(test.z, before.z));
				if (err < minErr)
				{
					minErr = err;
					r = test;
				}
			}
			return r;
		}
	}

	void MMDIkSolver::SolveCore(std::vector<glm::vec3> &in_prev_angles, glm::mat4 const &in_base_parent_transform_model_space, glm::vec3 const &in_target_position_model_space, glm::mat4 const &in_end_effector_transform_local_space, std::vector<glm::mat4x4> &inout_chain_local_space, std::vector<glm::mat4x4> &inout_chain_model_space)
	{
		// ccd IK from tail to root
		uint32_t current_joint_chain_index = m_chains.size() - 1;
		uint32_t const end_effector_parent_chain_index = m_chains.size() - 1U;
		uint32_t const in_chain_joint_count = m_chains.size();
		for (size_t chainIdx = 0; chainIdx < m_chains.size(); chainIdx++)
		{
			uint32_t const current_joint_chain_index_plus_1 = current_joint_chain_index + 1;

			auto &chain = m_chains[chainIdx];
			MMDNode *chainNode = chain.m_node;
			if (chainNode == m_ikTarget)
			{
				/*
				ターゲットとチェインが同じ場合、 chainTargetVec が0ベクトルとなる。
				その後の計算で求める回転値がnanになるため、計算を行わない
				対象モデル：ぽんぷ長式比叡.pmx
				*/
				assert(false);
				continue;
			}

			glm::vec3 end_effector_model_space_translation = (inout_chain_model_space[end_effector_parent_chain_index] * in_end_effector_transform_local_space)[3];

			auto invChain = glm::inverse(inout_chain_model_space[current_joint_chain_index]);

			auto chainIkPos = glm::vec3(invChain * glm::vec4(in_target_position_model_space, 1));
			auto chainTargetPos = glm::vec3(invChain * glm::vec4(end_effector_model_space_translation, 1));

			auto chainIkVec = glm::normalize(chainIkPos);
			auto chainTargetVec = glm::normalize(chainTargetPos);

			auto dot_local_space = glm::dot(chainTargetVec, chainIkVec);
			dot_local_space = glm::clamp(dot_local_space, -1.0f, 1.0f);

			glm::quat rot_local_space;
			float dot_angle_limit = std::cos(std::min(m_limitAngle, 3.14F));
			if (dot_local_space >= dot_angle_limit)
			{

				DirectX::XMVECTOR dx_chainIkVec = DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&chainIkVec));
				DirectX::XMVECTOR dx_chainTargetVec = DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&chainTargetVec));

				DirectX::XMFLOAT4 dx_rot_local_space;
				DirectX::XMStoreFloat4(&dx_rot_local_space, internal_compute_shortest_rotation_damped(dx_chainTargetVec, dx_chainIkVec, 1.0F));

				rot_local_space.w = dx_rot_local_space.w;
				rot_local_space.x = dx_rot_local_space.x;
				rot_local_space.y = dx_rot_local_space.y;
				rot_local_space.z = dx_rot_local_space.z;
			}
			else
			{
				if (dot_local_space > (-(1.0F - 1E-6F)) && dot_local_space < (1.0F - 1E-6F))
				{
					float angle_local_space = std::acos(dot_local_space);
					angle_local_space = glm::clamp(angle_local_space, -m_limitAngle, m_limitAngle);
					auto cross = glm::normalize(glm::cross(chainTargetVec, chainIkVec));
					rot_local_space = glm::rotate(glm::quat(1, 0, 0, 0), angle_local_space, cross);
				}
				else
				{
					rot_local_space = glm::quat(1, 0, 0, 0);
				}
			}

#if 0
			glm::vec3 current_joint_model_space_translation = glm::vec3(inout_chain_model_space[current_joint_chain_index][3]);
			glm::quat current_joint_model_space_rotation = glm::quat_cast(glm::mat3(inout_chain_model_space[current_joint_chain_index]));

			glm::vec3 current_model_space_direction = glm::normalize(end_effector_model_space_translation - current_joint_model_space_translation);
			glm::vec3 target_model_space_direction = glm::normalize(in_target_position_model_space - current_joint_model_space_translation);

			auto dot_model_space = glm::dot(current_model_space_direction, target_model_space_direction);
			dot_model_space = glm::clamp(dot_model_space, -1.0f, 1.0f);

			float angle_model_space = std::acos(dot_model_space);

			glm::quat rot_model_space;
			if (dot_model_space > (-(1.0F - 1E-6F)) && dot_model_space < (1.0F - 1E-6F))
			{
				angle_model_space = glm::clamp(angle_model_space, -m_limitAngle, m_limitAngle);
				auto cross = glm::normalize(glm::cross(current_model_space_direction, target_model_space_direction));
				rot_model_space = glm::rotate(glm::quat(1, 0, 0, 0), angle_model_space, cross);
			}
			else
			{
				rot_model_space = glm::quat(1, 0, 0, 0);
			}
#endif

			glm::quat updated_current_joint_local_space_rotation;
			glm::mat4 updated_current_joint_local_space_transform;
			glm::mat4 updated_current_joint_model_space_transform;
			{
				glm::mat4 current_parent_joint_model_space = (current_joint_chain_index >= 1U) ? inout_chain_model_space[current_joint_chain_index - 1U] : in_base_parent_transform_model_space;

				glm::mat4 current_joint_local_space = inout_chain_local_space[current_joint_chain_index];

				glm::vec3 current_joint_local_space_translation = glm::vec3(current_joint_local_space[3]);
				glm::vec3 current_joint_local_space_scale = glm::vec3(
					glm::length(glm::vec3(current_joint_local_space[0])),
					glm::length(glm::vec3(current_joint_local_space[1])),
					glm::length(glm::vec3(current_joint_local_space[2])));
				glm::quat current_joint_local_space_rotation = glm::quat_cast(glm::mat3(
					glm::vec3(current_joint_local_space[0]) / current_joint_local_space_scale.x,
					glm::vec3(current_joint_local_space[1]) / current_joint_local_space_scale.y,
					glm::vec3(current_joint_local_space[2]) / current_joint_local_space_scale.z));
				assert(glm::all(glm::epsilonEqual(current_joint_local_space_scale, glm::vec3(1.0F), 1E-6F)));

				updated_current_joint_local_space_rotation = current_joint_local_space_rotation * rot_local_space;

				if (chain.m_enableAxisLimit)
				{
					auto chainRotM = glm::mat3_cast(updated_current_joint_local_space_rotation);

					auto rotXYZ = Decompose(chainRotM, in_prev_angles[chainIdx]);
					glm::vec3 clampXYZ;
					clampXYZ = glm::clamp(rotXYZ, chain.m_limitMin, chain.m_limitMax);
					// clampXYZ = glm::clamp(clampXYZ - chain.m_prevAngle, -m_limitAngle, m_limitAngle) + chain.m_prevAngle;
					auto r = glm::rotate(glm::quat(1, 0, 0, 0), clampXYZ.x, glm::vec3(1, 0, 0));
					r = glm::rotate(r, clampXYZ.y, glm::vec3(0, 1, 0));
					r = glm::rotate(r, clampXYZ.z, glm::vec3(0, 0, 1));
					chainRotM = glm::mat3_cast(r);
					in_prev_angles[chainIdx] = clampXYZ;

					updated_current_joint_local_space_rotation = glm::quat_cast(chainRotM);
				}

				updated_current_joint_local_space_transform = glm::translate(glm::mat4(1), current_joint_local_space_translation) * glm::mat4_cast(updated_current_joint_local_space_rotation);
				updated_current_joint_model_space_transform = current_parent_joint_model_space * updated_current_joint_local_space_transform;
			}

			inout_chain_local_space[current_joint_chain_index] = updated_current_joint_local_space_transform;
			inout_chain_model_space[current_joint_chain_index] = updated_current_joint_model_space_transform;

			for (uint32_t child_joint_chain_index_plus_1 = (current_joint_chain_index_plus_1 + 1U); child_joint_chain_index_plus_1 <= in_chain_joint_count; ++child_joint_chain_index_plus_1)
			{
				uint32_t const parent_joint_chain_index = child_joint_chain_index_plus_1 - 1U - 1U;
				uint32_t const child_joint_chain_index = child_joint_chain_index_plus_1 - 1U;

				inout_chain_model_space[child_joint_chain_index] = inout_chain_model_space[parent_joint_chain_index] * inout_chain_local_space[child_joint_chain_index];
			}

			--current_joint_chain_index;
		}
	}
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_compute_shortest_rotation_damped(DirectX::XMVECTOR from, DirectX::XMVECTOR to, float gain)
{
	constexpr float const one = 1.0F;
	constexpr float const half = 0.5F;
	constexpr float const zero = 0.0F;
	constexpr float const epsilon = 1E-6F;
	constexpr float const nearly_one = one - epsilon;

	// cos(theta)
	float const dot_product = DirectX::XMVectorGetX(DirectX::XMVector3Dot(from, to));

	float const damped_dot = one - gain + gain * dot_product;

	float const cos_theta_div_2_square = (damped_dot + one) * half;

	if (cos_theta_div_2_square > zero && dot_product >= (-nearly_one) && dot_product <= nearly_one)
	{
		// cos(theta/2) = sqrt((1+cos(theta))/2)
		float cos_theta_div_2 = std::sqrt(cos_theta_div_2_square);

		// sin(theta)
		DirectX::XMVECTOR cross = DirectX::XMVector3Cross(from, to);

		// sin(theta/2) = sin(theta)/(2*cos(theta/2))
		DirectX::XMVECTOR sin_theta_div_2 = DirectX::XMVectorScale(cross, ((gain * half) / cos_theta_div_2));

		// "dot_product >= -nearly_one && dot_product <= nearly_one" to avoid zero vector which can NOT be normalized
		return DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(sin_theta_div_2, cos_theta_div_2));
	}
	else if (cos_theta_div_2_square > zero && dot_product > nearly_one)
	{
		return DirectX::XMQuaternionIdentity();
	}
	else
	{
		return DirectX::XMVectorSetW(DirectX::XMVector3Normalize(internal_calculate_perpendicular_vector(from)), 0.0F);
	}
}

static inline DirectX::XMVECTOR XM_CALLCONV internal_calculate_perpendicular_vector(DirectX::XMVECTOR simd_in_v)
{
	int min = 0;
	int ok1 = 1;
	int ok2 = 2;

	float in_v[3];
	DirectX::XMStoreFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&in_v[0]), simd_in_v);

	float a0 = in_v[0];
	float a1 = in_v[1];
	float a2 = in_v[2];

	if (a1 < a0)
	{
		ok1 = 0;
		min = 1;
		a0 = a1;
	}

	if (a2 < a0)
	{
		ok2 = min;
		min = 2;
	}

	float out_v[3] = {0.0F, 0.0F, 0.0F};
	out_v[ok1] = in_v[ok2];
	out_v[ok2] = -in_v[ok1];
	return DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 *>(&out_v[0]));
}
