//
// Copyright(c) 2016-2019 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "VMDCameraAnimation.h"
#include "VMDAnimationCommon.h"

#include <glm/gtc/matrix_transform.hpp>

namespace saba
{
	namespace
	{
		static inline void SetVMDBezier(VMDBezier &bezier, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
		{
			bezier.m_packed_k_1_x = static_cast<uint8_t>(std::max(static_cast<int8_t>(0), static_cast<int8_t>(x1)));
			bezier.m_packed_k_1_y = static_cast<uint8_t>(std::max(static_cast<int8_t>(0), static_cast<int8_t>(y1)));
			bezier.m_packed_k_2_x = static_cast<uint8_t>(std::max(static_cast<int8_t>(0), static_cast<int8_t>(x2)));
			bezier.m_packed_k_2_y = static_cast<uint8_t>(std::max(static_cast<int8_t>(0), static_cast<int8_t>(y2)));
		}

		glm::vec3 InvZ(const glm::vec3 &v)
		{
			return v;
		}
	} // namespace

	VMDCameraController::VMDCameraController()
		: m_startKeyIndex(0)
	{
	}

	void VMDCameraController::Evaluate(float t)
	{
		if (m_keys.empty())
		{
			return;
		}

		auto boundIt = FindBoundKey(m_keys, int32_t(t), m_startKeyIndex);
		if (boundIt == std::end(m_keys))
		{
			const auto &selectKey = m_keys[m_keys.size() - 1];
			m_camera.m_interest = selectKey.m_interest;
			m_camera.m_rotate = selectKey.m_rotate;
			m_camera.m_distance = selectKey.m_distance;
			m_camera.m_fov = selectKey.m_fov;
		}
		else
		{
			const auto &selectKey = (*boundIt);
			m_camera.m_interest = selectKey.m_interest;
			m_camera.m_rotate = selectKey.m_rotate;
			m_camera.m_distance = selectKey.m_distance;
			m_camera.m_fov = selectKey.m_fov;
			if (boundIt != std::begin(m_keys))
			{
				const auto &key0 = *(boundIt - 1);
				const auto &key1 = *boundIt;

				if ((key1.m_time - key0.m_time) > 1)
				{
					float timeRange = float(key1.m_time - key0.m_time);
					float time = (t - float(key0.m_time)) / timeRange;

					float ix_y = key0.m_ixBezier.XToY(time);
					float iy_y = key0.m_iyBezier.XToY(time);
					float iz_y = key0.m_izBezier.XToY(time);
					float rotate_y = key0.m_rotateBezier.XToY(time);
					float distance_y = key0.m_distanceBezier.XToY(time);
					float fov_y = key0.m_fovBezier.XToY(time);

					m_camera.m_interest = glm::mix(key0.m_interest, key1.m_interest, glm::vec3(ix_y, iy_y, iz_y));
					m_camera.m_rotate = glm::mix(key0.m_rotate, key1.m_rotate, rotate_y);
					m_camera.m_distance = glm::mix(key0.m_distance, key1.m_distance, distance_y);
					m_camera.m_fov = glm::mix(key0.m_fov, key1.m_fov, fov_y);
				}
				else
				{
					/*
					カメラアニメーションでキーが1フレーム間隔で打たれている場合、
					カメラの切り替えと判定し補間を行わないようにする（key0を使用する）
					*/
					m_camera.m_interest = key0.m_interest;
					m_camera.m_rotate = key0.m_rotate;
					m_camera.m_distance = key0.m_distance;
					m_camera.m_fov = key0.m_fov;
				}

				m_startKeyIndex = std::distance(m_keys.cbegin(), boundIt);
			}
		}
	}

	void VMDCameraController::SortKeys()
	{
		std::sort(
			std::begin(m_keys),
			std::end(m_keys),
			[](const KeyType &a, const KeyType &b)
			{ return a.m_time < b.m_time; });
	}

	VMDCameraAnimation::VMDCameraAnimation()
	{
		Destroy();
	}

	bool VMDCameraAnimation::Create(const VMDFile &vmd)
	{
		if (!vmd.m_vmd.m_cameras.empty())
		{
			m_cameraController = std::make_unique<VMDCameraController>();
			for (const auto &cam : vmd.m_vmd.m_cameras)
			{
				VMDCameraAnimationKey key;
				key.m_time = int32_t(cam.m_frame_number);
				key.m_interest = InvZ(glm::vec3(cam.m_focus_position.m_x, cam.m_focus_position.m_y, cam.m_focus_position.m_z));
				key.m_rotate = glm::vec3(cam.m_rotation.m_x, cam.m_rotation.m_y, cam.m_rotation.m_z);
				key.m_distance = cam.m_distance;
				key.m_fov = cam.m_fov_angle;

				SetVMDBezier(key.m_ixBezier, cam.m_focus_position_x_cubic_bezier[0], cam.m_focus_position_x_cubic_bezier[1], cam.m_focus_position_x_cubic_bezier[2], cam.m_focus_position_x_cubic_bezier[3]);
				SetVMDBezier(key.m_iyBezier, cam.m_focus_position_y_cubic_bezier[0], cam.m_focus_position_y_cubic_bezier[1], cam.m_focus_position_y_cubic_bezier[2], cam.m_focus_position_y_cubic_bezier[3]);
				SetVMDBezier(key.m_izBezier, cam.m_focus_position_z_cubic_bezier[0], cam.m_focus_position_z_cubic_bezier[1], cam.m_focus_position_z_cubic_bezier[2], cam.m_focus_position_z_cubic_bezier[3]);
				SetVMDBezier(key.m_rotateBezier, cam.m_rotation_cubic_bezier[0], cam.m_rotation_cubic_bezier[1], cam.m_rotation_cubic_bezier[2], cam.m_rotation_cubic_bezier[3]);
				SetVMDBezier(key.m_distanceBezier, cam.m_distance_cubic_bezier[0], cam.m_distance_cubic_bezier[1], cam.m_distance_cubic_bezier[2], cam.m_distance_cubic_bezier[3]);
				SetVMDBezier(key.m_fovBezier, cam.m_fov_angle_cubic_bezier[0], cam.m_fov_angle_cubic_bezier[1], cam.m_fov_angle_cubic_bezier[2], cam.m_fov_angle_cubic_bezier[3]);

				m_cameraController->AddKey(key);
			}
			m_cameraController->SortKeys();
		}
		else
		{
			return false;
		}
		return true;
	}

	void VMDCameraAnimation::Destroy()
	{
		m_cameraController.reset();
	}

	void VMDCameraAnimation::Evaluate(float t)
	{
		m_cameraController->Evaluate(t);
		m_camera = m_cameraController->GetCamera();
	}

}
