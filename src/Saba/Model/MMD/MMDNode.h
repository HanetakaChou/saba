//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMD_MMDNODE_H_
#define SABA_MODEL_MMD_MMDNODE_H_

#include <string>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>

namespace saba
{
	class MMDNode
	{
	public:
		MMDNode();

		// アニメーションの前後て呼ぶ
		void BeginUpdateTransform();
		void EndUpdateTransform();

		void SetIndex(uint32_t idx) { m_index = idx; }
		uint32_t GetIndex() const { return m_index; }

		void SetName(const std::string &name) { m_name = name; }
		const std::string &GetName() const { return m_name; }

		void SetTranslate(const glm::vec3 &t) { m_translate = t; }
		const glm::vec3 &GetTranslate() const { return m_translate; }

		void SetRotate(const glm::quat &r) { m_rotate = r; }
		const glm::quat &GetRotate() const { return m_rotate; }

		void SetScale(const glm::vec3 &s) { m_scale = s; }
		const glm::vec3 &GetScale() const { return m_scale; }

		void SetAnimationTranslate(const glm::vec3 &animTranslate) { m_translate = (animTranslate + m_translate); }

		void SetAnimationRotate(const glm::quat &animRotate)
		{
			// [Normalized Local Rotation](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm_animation-1.0/how_to_transform_human_pose.md)
			m_rotate = (animRotate * m_rotate);
		}

		void SetLocalTransform(const glm::mat4 &matrix)
		{
			m_translate = glm::vec3(matrix[3]);

			m_scale = glm::vec3(
				glm::length(glm::vec3(matrix[0])),
				glm::length(glm::vec3(matrix[1])),
				glm::length(glm::vec3(matrix[2])));
			assert(glm::all(glm::epsilonEqual(m_scale, glm::vec3(1.0F), 1E-3F)));

			m_rotate = glm::quat_cast(glm::mat3(
				glm::vec3(matrix[0]) / m_scale.x,
				glm::vec3(matrix[1]) / m_scale.y,
				glm::vec3(matrix[2]) / m_scale.z));
		}

		const glm::mat4 GetLocalTransform() const
		{
			assert(glm::all(glm::epsilonEqual(m_scale, glm::vec3(1.0F), 1E-3F)));

			return glm::translate(glm::mat4(1), m_translate) * glm::mat4_cast(m_rotate);
		}

		void SetGlobalTransform(const glm::mat4 &m) { m_global = m; }
		const glm::mat4 &GetGlobalTransform() const { return m_global; }

		void CalculateInverseInitTransform();
		const glm::mat4 &GetInverseInitTransform() const { return m_inverseInit; }

		// ノードの初期化時に呼び出す
		void SaveInitialTRS()
		{
			m_initTranslate = m_translate;
			m_initRotate = m_rotate;
			m_initScale = m_scale;
		}
		void LoadInitialTRS()
		{
			m_translate = m_initTranslate;
			m_rotate = m_initRotate;
			m_scale = m_initScale;
		}
		const glm::vec3 &GetInitialTranslate() const { return m_initTranslate; }
		const glm::quat &GetInitialRotate() const { return m_initRotate; }
		const glm::vec3 &GetInitialScale() const { return m_initScale; }

		void SaveBaseAnimation()
		{
			m_baseAnimTranslate = m_translate;
			m_baseAnimRotate = m_rotate;
		}

		void LoadBaseAnimation()
		{
			m_translate = m_baseAnimTranslate;
			m_rotate = m_baseAnimRotate;
		}

		void ClearBaseAnimation()
		{
			m_baseAnimTranslate = glm::vec3(0.0F);
			m_baseAnimRotate = glm::quat(1.0F, 0.0F, 0.0F, 0.0F);
		}

		const glm::vec3 &GetBaseAnimationTranslate() const
		{
			return m_baseAnimTranslate;
		};

		const glm::quat &GetBaseAnimationRotate() const
		{
			return m_baseAnimRotate;
		}

	protected:
		virtual void OnBeginUpdateTransform();
		virtual void OnEndUpdateTransfrom();

	protected:
		uint32_t m_index;
		std::string m_name;

		glm::vec3 m_translate;
		glm::quat m_rotate;
		glm::vec3 m_scale;

		glm::vec3 m_baseAnimTranslate;
		glm::quat m_baseAnimRotate;

		glm::mat4 m_global;
		glm::mat4 m_inverseInit;

		glm::vec3 m_initTranslate;
		glm::quat m_initRotate;
		glm::vec3 m_initScale;
	};
}

#endif // !SABA_MODEL_MMD_MMDNODE_H_
