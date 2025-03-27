//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#include "VMDAnimation.h"
#include "VMDAnimationCommon.h"

#include <Saba/Base/Log.h>

#include <algorithm>
#include <iterator>
#include <map>
#include <glm/gtc/matrix_transform.hpp>

#include <DirectXMath.h>

struct internal_rigid_transform_key_frame_t
{
	brx_asset_import_rigid_transform m_rigid_transform;
	uint8_t m_translation_x_cubic_bezier[4];
	uint8_t m_translation_y_cubic_bezier[4];
	uint8_t m_translation_z_cubic_bezier[4];
	uint8_t m_rotation_cubic_bezier[4];
};

static inline float internal_cubic_bezier(uint8_t const in_packed_k_1_x, uint8_t const in_packed_k_1_y, uint8_t const in_packed_k_2_x, uint8_t const in_packed_k_2_y, float const in_x)
{
	// https://developer.mozilla.org/en-US/docs/Web/CSS/easing-function/cubic-bezier

	assert(in_packed_k_1_x <= static_cast<uint8_t>(INT8_MAX));
	assert(in_packed_k_1_y <= static_cast<uint8_t>(INT8_MAX));
	assert(in_packed_k_2_x <= static_cast<uint8_t>(INT8_MAX));
	assert(in_packed_k_2_y <= static_cast<uint8_t>(INT8_MAX));
	assert((in_x >= 0.0F) && (in_x <= 1.0F));

	// [_FnBezier.__find_roots](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/importer.py#L198)

	float out_y;
	if ((in_packed_k_1_x == in_packed_k_1_y) && (in_packed_k_2_x == in_packed_k_2_y))
	{
		out_y = in_x;
	}
	else
	{
		float const k_1_x = static_cast<float>(static_cast<double>(in_packed_k_1_x) / static_cast<double>(INT8_MAX));
		float const k_1_y = static_cast<float>(static_cast<double>(in_packed_k_1_y) / static_cast<double>(INT8_MAX));

		float const k_2_x = static_cast<float>(static_cast<double>(in_packed_k_2_x) / static_cast<double>(INT8_MAX));
		float const k_2_y = static_cast<float>(static_cast<double>(in_packed_k_2_y) / static_cast<double>(INT8_MAX));

		// https://developer.mozilla.org/en-US/docs/Web/CSS/easing-function/cubic-bezier
		// [_FnBezier.__find_roots](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/importer.py#L198)

		float t;
		{
			// 0 * (1 - t)^3 + 3 * k_1 * (1 - t)^2 * t + 3 * k_2 * (1 - t) * t^2 + 1 * t^3 - x = 0
			// (3 * k_1 - 3 * k_2 + 1) * t^3 + (3 * k_2 - 6 * k_1) * t^2 + (3 * k_1) * t - x = 0

			float const a = k_1_x * 3.0F - k_2_x * 3.0F + 1.0F;
			float const b = k_2_x * 3.0F - k_1_x * 6.0F;
			float const c = k_1_x * 3.0F;
			float const d = 0.0F - in_x;

			// solve cubic equation: a*t^3 + b*t^2 + c*t + d = 0

			constexpr float const internal_epsilon = 1E-6F;
			constexpr float const internal_verification_epsilon = 2E-3F;

			if (std::abs(a) > internal_epsilon)
			{
				// Cubic Equation

				float const p = (c / a) - ((b / a) * (b / a) * (1.0F / 3.0F));
				float const q = ((b / a) * (b / a) * (b / a) * (2.0F / 27.0F)) - ((b / a) * (c / a) * (1.0F / 3.0F)) + (d / a);

				float const delta = q * q * (1.0F / 4.0F) + (p * p * p) * (1.0F / 27.0F);

				if (delta > internal_epsilon)
				{
					float const sqrt_delta = std::sqrt(delta);
					float const u = std::cbrt(((0.0F - q) * (1.0F / 2.0F)) + sqrt_delta);
					float const v = std::cbrt(((0.0F - q) * (1.0F / 2.0F)) - sqrt_delta);
					float const t1 = (u + v) - ((b / a) * (1.0F / 3.0F));
					assert(std::abs(a * t1 * t1 * t1 + b * t1 * t1 + c * t1 + d) < internal_verification_epsilon);

					if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t1), 1.0F);
					}
					else
					{
						assert(false);
						t = in_x;
					}
				}
				else if (delta > -internal_epsilon)
				{
					float const u = std::cbrt((0.0F - q) * (1.0F / 2.0F));
					float const t1 = (u * 2.0F) - ((b / a) * (1.0F / 3.0F));
					float const t2 = (0.0F - u) - ((b / a) * (1.0F / 3.0F));
					assert(std::abs(a * t1 * t1 * t1 + b * t1 * t1 + c * t1 + d) < internal_verification_epsilon);
					assert(std::abs(a * t2 * t2 * t2 + b * t2 * t2 + c * t2 + d) < internal_verification_epsilon);

					if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)) && (t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)))
					{
						if (std::abs(t1 - in_x) < std::abs(t2 - in_x))
						{
							t = std::min(std::max(0.0F, t1), 1.0F);
						}
						else
						{
							t = std::min(std::max(0.0F, t2), 1.0F);
						}
					}
					else if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t1), 1.0F);
					}
					else if ((t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t1), 1.0F);
					}
					else
					{
						assert(false);
						t = in_x;
					}
				}
				else
				{

					assert(delta <= -internal_epsilon);
					assert(p <= -internal_epsilon);

					constexpr double const internal_pi = 3.14159265358979323846264338327950288;

					double const r = 2.0 * std::sqrt((0.0 - static_cast<double>(p)) * (1.0 / 3.0));
					double const phi = std::acos(((0.0 - static_cast<double>(q)) * 3.0) / (r * (0.0 - static_cast<double>(p))));
					float const t1 = static_cast<float>((r * std::cos(phi * (1.0 / 3.0))) - ((static_cast<double>(b) / static_cast<double>(a)) * (1.0 / 3.0)));
					float const t2 = static_cast<float>((r * std::cos((phi + 2.0 * internal_pi) * (1.0 / 3.0))) - ((static_cast<double>(b) / static_cast<double>(a)) * (1.0 / 3.0)));
					float const t3 = static_cast<float>((r * std::cos((phi + 2.0 * internal_pi * 2.0) * (1.0 / 3.0))) - ((static_cast<double>(b) / static_cast<double>(a)) * (1.0 / 3.0)));
					assert(std::abs(a * t1 * t1 * t1 + b * t1 * t1 + c * t1 + d) < internal_verification_epsilon);
					assert(std::abs(a * t2 * t2 * t2 + b * t2 * t2 + c * t2 + d) < internal_verification_epsilon);
					assert(std::abs(a * t3 * t3 * t3 + b * t3 * t3 + c * t3 + d) < internal_verification_epsilon);

					if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)) && (t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)) && (t3 >= (0.0F - internal_epsilon)) && (t3 <= (1.0F + internal_epsilon)))
					{
						if (std::abs(t1 - in_x) < std::abs(t2 - in_x) && std::abs(t1 - in_x) < std::abs(t3 - in_x))
						{
							t = std::min(std::max(0.0F, t1), 1.0F);
						}
						else if (std::abs(t2 - in_x) < std::abs(t1 - in_x) && std::abs(t2 - in_x) < std::abs(t3 - in_x))
						{
							t = std::min(std::max(0.0F, t2), 1.0F);
						}
						else
						{
							assert(std::abs(t3 - in_x) <= std::abs(t1 - in_x) && std::abs(t3 - in_x) <= std::abs(t2 - in_x));
							t = std::min(std::max(0.0F, t3), 1.0F);
						}
					}
					else if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)) && (t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)))
					{
						if (std::abs(t1 - in_x) < std::abs(t2 - in_x))
						{
							t = std::min(std::max(0.0F, t1), 1.0F);
						}
						else
						{
							assert(std::abs(t2 - in_x) <= std::abs(t1 - in_x));
							t = std::min(std::max(0.0F, t2), 1.0F);
						}
					}
					else if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)) && (t3 >= (0.0F - internal_epsilon)) && (t3 <= (1.0F + internal_epsilon)))
					{
						if (std::abs(t1 - in_x) < std::abs(t3 - in_x))
						{
							t = std::min(std::max(0.0F, t1), 1.0F);
						}
						else
						{
							assert(std::abs(t3 - in_x) <= std::abs(t1 - in_x));
							t = std::min(std::max(0.0F, t3), 1.0F);
						}
					}
					else if ((t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)) && (t3 >= (0.0F - internal_epsilon)) && (t3 <= (1.0F + internal_epsilon)))
					{
						if (std::abs(t2 - in_x) < std::abs(t3 - in_x))
						{
							t = std::min(std::max(0.0F, t2), 1.0F);
						}
						else
						{
							assert(std::abs(t3 - in_x) <= std::abs(t2 - in_x));
							t = std::min(std::max(0.0F, t3), 1.0F);
						}
					}
					else if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t1), 1.0F);
					}
					else if ((t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t2), 1.0F);
					}
					else if ((t3 >= (0.0F - internal_epsilon)) && (t3 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t3), 1.0F);
					}
					else
					{
						assert(false);
						t = in_x;
					}
				}
			}
			else if (std::abs(b) > internal_epsilon)
			{
				// Quadratic Equation

				float const delta = c * c - b * d * 4.0F;
				if (delta > internal_epsilon)
				{
					float const sqrt_delta = std::sqrt(delta);
					float const t1 = ((0.0F - c) + sqrt_delta) / (b * 2.0F);
					float const t2 = ((0.0F - c) - sqrt_delta) / (b * 2.0F);
					assert(std::abs(a * t1 * t1 * t1 + b * t1 * t1 + c * t1 + d) < internal_verification_epsilon);
					assert(std::abs(a * t2 * t2 * t2 + b * t2 * t2 + c * t2 + d) < internal_verification_epsilon);

					if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)) && (t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)))
					{
						if (std::abs(t1 - in_x) < std::abs(t2 - in_x))
						{
							t = std::min(std::max(0.0F, t1), 1.0F);
						}
						else
						{
							assert(std::abs(t2 - in_x) <= std::abs(t1 - in_x));
							t = std::min(std::max(0.0F, t2), 1.0F);
						}
					}
					else if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t2), 1.0F);
					}
					else if ((t2 >= (0.0F - internal_epsilon)) && (t2 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t1), 1.0F);
					}
					else
					{
						assert(false);
						t = in_x;
					}
				}
				else
				{
					assert(false);
					t = in_x;
				}
			}
			else
			{
				if (std::abs(c) > internal_epsilon)
				{
					// Linear Equation

					float const t1 = (0.0F - d) / c;
					assert(std::abs(a * t1 * t1 * t1 + b * t1 * t1 + c * t1 + d) < internal_verification_epsilon);

					if ((t1 >= (0.0F - internal_epsilon)) && (t1 <= (1.0F + internal_epsilon)))
					{
						t = std::min(std::max(0.0F, t1), 1.0F);
					}
					else
					{
						assert(false);
						t = in_x;
					}
				}
				else
				{
					assert(false);
					t = in_x;
				}
			}
		}

		out_y = (k_1_y * 3.0F - k_2_y * 3.0F + 1.0F) * t * t * t + (k_2_y * 3.0F - k_1_y * 6.0F) * t * t + (k_1_y * 3.0F) * t;
	}

	return out_y;
}

namespace saba
{
	float VMDBezier::XToY(float const x) const
	{
		return internal_cubic_bezier(this->m_packed_k_1_x, this->m_packed_k_1_y, this->m_packed_k_2_x, this->m_packed_k_2_y, x);
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
			auto rot0 = glm::mat3_cast(q);
			auto rot1 = InvZ(rot0);
			return glm::quat_cast(rot1);
		}
	} // namespace

	VMDAnimation::VMDAnimation()
	{
	}

	bool VMDAnimation::Create(std::shared_ptr<MMDModel> model)
	{
		m_model = model;
		return true;
	}

	bool VMDAnimation::Add(const VMDFile &vmd)
	{
		if (vmd.m_vmd.m_motions.empty() && vmd.m_vmd.m_morphs.empty())
		{
			return true;
		}

		mcrt_vector<BRX_ASSET_IMPORT_MORPH_TARGET_NAME> weight_channel_names;
		mcrt_vector<float> weights;
		mcrt_vector<mcrt_string> rigid_transform_channel_names;
		mcrt_vector<brx_asset_import_rigid_transform> rigid_transforms;
		mcrt_vector<BRX_ASSET_IMPORT_IK_NAME> ik_switch_channel_names;
		mcrt_vector<bool> ik_switches;
		{
			uint32_t max_frame_number = 0U;
			mcrt_unordered_map<mcrt_string, mcrt_map<uint32_t, float>> weight_channels;
			mcrt_unordered_map<mcrt_string, mcrt_map<uint32_t, internal_rigid_transform_key_frame_t>> rigid_transform_channels;
			mcrt_unordered_map<mcrt_string, mcrt_map<uint32_t, bool>> ik_switch_channels;
			{
				mmd_vmd_t const &mmd_vmd = vmd.m_vmd;

				for (mmd_vmd_morph_t const &mmd_vmd_morph : mmd_vmd.m_morphs)
				{
					max_frame_number = std::max(mmd_vmd_morph.m_frame_number, max_frame_number);

					assert(weight_channels[mmd_vmd_morph.m_name].end() == weight_channels[mmd_vmd_morph.m_name].find(mmd_vmd_morph.m_frame_number));
					weight_channels[mmd_vmd_morph.m_name][mmd_vmd_morph.m_frame_number] = mmd_vmd_morph.m_weight;
				}

				for (mmd_vmd_motion_t const &mmd_vmd_motion : mmd_vmd.m_motions)
				{
					max_frame_number = std::max(mmd_vmd_motion.m_frame_number, max_frame_number);

					assert(rigid_transform_channels[mmd_vmd_motion.m_name].end() == rigid_transform_channels[mmd_vmd_motion.m_name].find(mmd_vmd_motion.m_frame_number));
					rigid_transform_channels[mmd_vmd_motion.m_name][mmd_vmd_motion.m_frame_number] = internal_rigid_transform_key_frame_t{
						{
							{mmd_vmd_motion.m_rotation.m_x, mmd_vmd_motion.m_rotation.m_y, mmd_vmd_motion.m_rotation.m_z, mmd_vmd_motion.m_rotation.m_w},
							{mmd_vmd_motion.m_translation.m_x, mmd_vmd_motion.m_translation.m_y, mmd_vmd_motion.m_translation.m_z},
						},
						{mmd_vmd_motion.m_translation_x_cubic_bezier[0], mmd_vmd_motion.m_translation_x_cubic_bezier[1], mmd_vmd_motion.m_translation_x_cubic_bezier[2], mmd_vmd_motion.m_translation_x_cubic_bezier[3]},
						{mmd_vmd_motion.m_translation_y_cubic_bezier[0], mmd_vmd_motion.m_translation_y_cubic_bezier[1], mmd_vmd_motion.m_translation_y_cubic_bezier[2], mmd_vmd_motion.m_translation_y_cubic_bezier[3]},
						{mmd_vmd_motion.m_translation_z_cubic_bezier[0], mmd_vmd_motion.m_translation_z_cubic_bezier[1], mmd_vmd_motion.m_translation_z_cubic_bezier[2], mmd_vmd_motion.m_translation_z_cubic_bezier[3]},
						{mmd_vmd_motion.m_rotation_cubic_bezier[0], mmd_vmd_motion.m_rotation_cubic_bezier[1], mmd_vmd_motion.m_rotation_cubic_bezier[2], mmd_vmd_motion.m_rotation_cubic_bezier[3]}};
				}

				for (mmd_vmd_ik_t const &mmd_vmd_iks : mmd_vmd.m_iks)
				{
					max_frame_number = std::max(mmd_vmd_iks.m_frame_number, max_frame_number);

					assert(ik_switch_channels[mmd_vmd_iks.m_name].end() == ik_switch_channels[mmd_vmd_iks.m_name].find(mmd_vmd_iks.m_frame_number));
					ik_switch_channels[mmd_vmd_iks.m_name][mmd_vmd_iks.m_frame_number] = mmd_vmd_iks.m_enable;
				}
			}

			// Frame Rate = 2
			// Max Time = 2
			//
			// Time  0.0 0.5 1.0 1.5 2.0
			// Frame 0   1   2   3   4
			// Frame Count = 2 * 2 + 1 = 5
			//
			// Our Method
			// Time  0.0 0.5 1.0 1.5 2.0
			// Frame    0   1   2   3
			// Frame Count = 2 * 2 = 4
			uint32_t const frame_count = max_frame_number;

			constexpr uint32_t mmd_morph_target_name_count = BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_COUNT;
			mcrt_vector<mcrt_string> weight_channel_name_strings;
			constexpr uint32_t mmd_ik_name_count = BRX_ASSET_IMPORT_IK_NAME_MMD_COUNT;
			mcrt_vector<mcrt_string> ik_switch_channel_name_strings;
			{
				mcrt_vector<mcrt_vector<mcrt_string>> mmd_morph_target_name_strings(static_cast<size_t>(mmd_morph_target_name_count));
				{
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_HAPPY].emplace_back(u8"にこり");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_ANGRY].emplace_back(u8"怒り");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_ANGRY].emplace_back(u8"真面目");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_SAD].emplace_back(u8"困る");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_SURPRISED].emplace_back(u8"上");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK].emplace_back(u8"まばたき");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_L].emplace_back(u8"ウィンク２");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_L].emplace_back(u8"ウィンク");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_R].emplace_back(u8"ｳｨﾝｸ２右");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_BLINK_R].emplace_back(u8"ウィンク右");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_HAPPY].emplace_back(u8"笑い");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_ANGRY].emplace_back(u8"ｷﾘｯ");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_SAD].emplace_back(u8"じと目");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_EYE_SURPRISED].emplace_back(u8"びっくり");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_A].emplace_back(u8"あ");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_A].emplace_back(u8"あ２");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_I].emplace_back(u8"い");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_U].emplace_back(u8"う");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_E].emplace_back(u8"え");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_O].emplace_back(u8"お");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY].emplace_back(u8"にっこり");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY].emplace_back(u8"にやり");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_HAPPY].emplace_back(u8"にやり２");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_ANGRY].emplace_back(u8"∧");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_SAD].emplace_back(u8"口角下げ");
					mmd_morph_target_name_strings[BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_MOUTH_SURPRISED].emplace_back(u8"▲");
				}

				for (uint32_t mmd_morph_target_name = 0U; mmd_morph_target_name < mmd_morph_target_name_count; ++mmd_morph_target_name)
				{
					for (mcrt_string const &mmd_morph_target_name_string : mmd_morph_target_name_strings[mmd_morph_target_name])
					{
						auto &found_weight_channel = weight_channels.find(mmd_morph_target_name_string);
						if (weight_channels.end() != found_weight_channel)
						{
							weight_channel_names.push_back(static_cast<BRX_ASSET_IMPORT_MORPH_TARGET_NAME>(mmd_morph_target_name));
							weight_channel_name_strings.push_back(mmd_morph_target_name_string);
							break;
						}
					}
				}

				rigid_transform_channel_names.resize(static_cast<size_t>(rigid_transform_channels.size()));
				rigid_transforms.resize(static_cast<size_t>(rigid_transform_channels.size() * frame_count));

				uint32_t rigid_transform_channel_index = 0U;
				for (auto const &rigid_transform_channel : rigid_transform_channels)
				{
					rigid_transform_channel_names[rigid_transform_channel_index] = rigid_transform_channel.first;
					++rigid_transform_channel_index;
				}

				mcrt_vector<mcrt_vector<mcrt_string>> mmd_ik_name_strings(static_cast<size_t>(mmd_ik_name_count));
				{
					mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_RIGHT_FOOT].emplace_back(u8"右足ＩＫ");
					mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_RIGHT_TOE].emplace_back(u8"右つま先ＩＫ");
					mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_LEFT_FOOT].emplace_back(u8"左足ＩＫ");
					mmd_ik_name_strings[BRX_ASSET_IMPORT_IK_NAME_MMD_LEFT_TOE].emplace_back(u8"左つま先ＩＫ");
				}

				for (uint32_t mmd_ik_name = 0U; mmd_ik_name < mmd_ik_name_count; ++mmd_ik_name)
				{
					for (mcrt_string const &mmd_ik_name_string : mmd_ik_name_strings[mmd_ik_name])
					{
						auto &found_ik_switch_channel = ik_switch_channels.find(mmd_ik_name_string);
						if (ik_switch_channels.end() != found_ik_switch_channel)
						{
							ik_switch_channel_names.push_back(static_cast<BRX_ASSET_IMPORT_IK_NAME>(mmd_ik_name));
							ik_switch_channel_name_strings.push_back(mmd_ik_name_string);
							break;
						}
					}
				}
			}

			uint32_t const rigid_transform_channel_count = rigid_transform_channel_names.size();
			assert(rigid_transform_channels.size() == rigid_transform_channel_count);

			uint32_t const weight_channel_count = weight_channel_names.size();
			assert(weight_channel_name_strings.size() == weight_channel_count);
			assert(weight_channel_count <= mmd_morph_target_name_count);
			// Initilize "0.0F" for NOT found
			weights.resize(static_cast<size_t>(weight_channel_count * frame_count), 0.0F);

			uint32_t const ik_switch_channel_count = ik_switch_channel_names.size();
			assert(ik_switch_channel_name_strings.size() == ik_switch_channel_count);
			assert(ik_switch_channel_count <= mmd_morph_target_name_count);
			// Initilize "true" for NOT found
			ik_switches.resize(static_cast<size_t>(ik_switch_channel_count * frame_count), true);

			for (uint32_t frame_index = 0; frame_index < frame_count; ++frame_index)
			{
				for (uint32_t weight_channel_index = 0U; weight_channel_index < weight_channel_count; ++weight_channel_index)
				{
					auto &found_weight_channel = weight_channels.find(weight_channel_name_strings[weight_channel_index]);

					if (weight_channels.end() != found_weight_channel)
					{
						mcrt_map<uint32_t, float> const &weight_channel = found_weight_channel->second;

						auto const &key_upper_bound = weight_channel.upper_bound(frame_index);

						if (weight_channel.end() != key_upper_bound)
						{
							if (weight_channel.begin() != key_upper_bound)
							{
								auto const &key_next = key_upper_bound;
								auto const &key_previous = std::prev(key_upper_bound);

								float const sample_time = static_cast<float>(frame_index) + 0.5F;
								assert((0U == max_frame_number) || (sample_time < max_frame_number));

								assert(static_cast<float>(key_next->first) > sample_time);
								assert(sample_time > static_cast<float>(key_previous->first));

								float const normalized_time = (sample_time - static_cast<float>(key_previous->first)) / (static_cast<float>(key_next->first) - static_cast<float>(key_previous->first));
								assert((normalized_time >= 0.0F) && (normalized_time <= 1.0F));

								assert(0.0F == weights[weight_channel_count * frame_index + weight_channel_index]);
								weights[weight_channel_count * frame_index + weight_channel_index] = (key_next->second - key_previous->second) * normalized_time + key_previous->second;
							}
							else
							{
								auto const &key_next = key_upper_bound;

								float const sample_time = static_cast<float>(frame_index) + 0.5F;
								assert((0U == max_frame_number) || (sample_time < max_frame_number));

								assert(static_cast<float>(key_next->first) > sample_time);

								// weight of the bind pose is zero
								constexpr uint32_t const key_previous_frame_number = 0U;
								constexpr float const key_previous_weight = 0.0F;

								float const normalized_time = (sample_time - static_cast<float>(key_previous_frame_number)) / (static_cast<float>(key_next->first) - static_cast<float>(key_previous_frame_number));
								assert((normalized_time >= 0.0F) && (normalized_time <= 1.0F));

								assert(0.0F == weights[weight_channel_count * frame_index + weight_channel_index]);
								weights[weight_channel_count * frame_index + weight_channel_index] = (key_next->second - key_previous_weight) * normalized_time + key_previous_weight;
							}
						}
						else
						{
							auto const &key_previous = std::prev(key_upper_bound);
							assert(&(*weight_channel.rbegin()) == &(*key_previous));

							float const sample_time = static_cast<float>(frame_index) + 0.5F;
							assert((0U == max_frame_number) || (sample_time < max_frame_number));
							assert(sample_time > static_cast<float>(key_previous->first));

							assert(0.0F == weights[weight_channel_count * frame_index + weight_channel_index]);
							weights[weight_channel_count * frame_index + weight_channel_index] = key_previous->second;
						}
					}
					else
					{
						assert(false);
					}
				}

				for (uint32_t rigid_transform_channel_index = 0U; rigid_transform_channel_index < rigid_transform_channel_count; ++rigid_transform_channel_index)
				{
					auto const &rigid_transform_channel = rigid_transform_channels[rigid_transform_channel_names[rigid_transform_channel_index]];

					auto const &key_upper_bound = rigid_transform_channel.upper_bound(frame_index);

					if (rigid_transform_channel.end() != key_upper_bound)
					{
						if (rigid_transform_channel.begin() != key_upper_bound)
						{
							auto const &key_next = key_upper_bound;
							auto const &key_previous = std::prev(key_upper_bound);

							float const sample_time = static_cast<float>(frame_index) + 0.5F;
							assert((0U == max_frame_number) || (sample_time < max_frame_number));

							assert(static_cast<float>(key_next->first) > sample_time);
							assert(sample_time > static_cast<float>(key_previous->first));

							float const normalized_time = (sample_time - static_cast<float>(key_previous->first)) / (static_cast<float>(key_next->first) - static_cast<float>(key_previous->first));
							assert((normalized_time >= 0.0F) && (normalized_time <= 1.0F));

							float const rotation_lerp_factor = internal_cubic_bezier(key_previous->second.m_rotation_cubic_bezier[0], key_previous->second.m_rotation_cubic_bezier[1], key_previous->second.m_rotation_cubic_bezier[2], key_previous->second.m_rotation_cubic_bezier[3], normalized_time);
							float const translation_x_lerp_factor = internal_cubic_bezier(key_previous->second.m_translation_x_cubic_bezier[0], key_previous->second.m_translation_x_cubic_bezier[1], key_previous->second.m_translation_x_cubic_bezier[2], key_previous->second.m_translation_x_cubic_bezier[3], normalized_time);
							float const translation_y_lerp_factor = internal_cubic_bezier(key_previous->second.m_translation_y_cubic_bezier[0], key_previous->second.m_translation_y_cubic_bezier[1], key_previous->second.m_translation_y_cubic_bezier[2], key_previous->second.m_translation_y_cubic_bezier[3], normalized_time);
							float const translation_z_lerp_factor = internal_cubic_bezier(key_previous->second.m_translation_z_cubic_bezier[0], key_previous->second.m_translation_z_cubic_bezier[1], key_previous->second.m_translation_z_cubic_bezier[2], key_previous->second.m_translation_z_cubic_bezier[3], normalized_time);

							DirectX::XMFLOAT4 const rotation_previous(key_previous->second.m_rigid_transform.m_rotation[0], key_previous->second.m_rigid_transform.m_rotation[1], key_previous->second.m_rigid_transform.m_rotation[2], key_previous->second.m_rigid_transform.m_rotation[3]);
							DirectX::XMFLOAT4 const rotation_next(key_next->second.m_rigid_transform.m_rotation[0], key_next->second.m_rigid_transform.m_rotation[1], key_next->second.m_rigid_transform.m_rotation[2], key_next->second.m_rigid_transform.m_rotation[3]);

							DirectX::XMFLOAT4 sample_rotation;
							DirectX::XMStoreFloat4(&sample_rotation, DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(DirectX::XMQuaternionNormalize(DirectX::XMLoadFloat4(&rotation_previous)), DirectX::XMQuaternionNormalize(DirectX::XMLoadFloat4(&rotation_next)), rotation_lerp_factor)));

							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[0] = sample_rotation.x;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[1] = sample_rotation.y;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[2] = sample_rotation.z;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[3] = sample_rotation.w;

							DirectX::XMFLOAT3 const translation_previous(key_previous->second.m_rigid_transform.m_translation[0], key_previous->second.m_rigid_transform.m_translation[1], key_previous->second.m_rigid_transform.m_translation[2]);
							DirectX::XMFLOAT3 const translation_next(key_next->second.m_rigid_transform.m_translation[0], key_next->second.m_rigid_transform.m_translation[1], key_next->second.m_rigid_transform.m_translation[2]);
							DirectX::XMFLOAT3 const lerp_factor(translation_x_lerp_factor, translation_y_lerp_factor, translation_z_lerp_factor);

							DirectX::XMFLOAT3 sample_translation;
							DirectX::XMStoreFloat3(&sample_translation, DirectX::XMVectorLerpV(DirectX::XMLoadFloat3(&translation_previous), DirectX::XMLoadFloat3(&translation_next), DirectX::XMLoadFloat3(&lerp_factor)));

							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[0] = sample_translation.x;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[1] = sample_translation.y;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[2] = sample_translation.z;
						}
						else
						{
							auto const &key_next = key_upper_bound;

							float const sample_time = static_cast<float>(frame_index) + 0.5F;
							assert((0U == max_frame_number) || (sample_time < max_frame_number));

							assert(static_cast<float>(key_next->first) > sample_time);

							DirectX::XMFLOAT4 const rotation_next(key_next->second.m_rigid_transform.m_rotation[0], key_next->second.m_rigid_transform.m_rotation[1], key_next->second.m_rigid_transform.m_rotation[2], key_next->second.m_rigid_transform.m_rotation[3]);

							DirectX::XMFLOAT4 rotation_next_normalized;
							DirectX::XMStoreFloat4(&rotation_next_normalized, DirectX::XMQuaternionNormalize(DirectX::XMLoadFloat4(&rotation_next)));

							// TODO: lerp from the bind pose
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[0] = rotation_next_normalized.x;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[1] = rotation_next_normalized.y;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[2] = rotation_next_normalized.z;
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[3] = rotation_next_normalized.w;

							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[0] = key_next->second.m_rigid_transform.m_translation[0];
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[1] = key_next->second.m_rigid_transform.m_translation[1];
							rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[2] = key_next->second.m_rigid_transform.m_translation[2];
						}
					}
					else
					{
						auto const &key_previous = std::prev(key_upper_bound);
						assert(&(*rigid_transform_channel.rbegin()) == &(*key_previous));

						float const sample_time = static_cast<float>(frame_index) + 0.5F;
						assert((0U == max_frame_number) || (sample_time < max_frame_number));
						assert(sample_time > static_cast<float>(key_previous->first));

						DirectX::XMFLOAT4 const rotation_previous(key_previous->second.m_rigid_transform.m_rotation[0], key_previous->second.m_rigid_transform.m_rotation[1], key_previous->second.m_rigid_transform.m_rotation[2], key_previous->second.m_rigid_transform.m_rotation[3]);

						DirectX::XMFLOAT4 rotation_previous_normalized;
						DirectX::XMStoreFloat4(&rotation_previous_normalized, DirectX::XMQuaternionNormalize(DirectX::XMLoadFloat4(&rotation_previous)));

						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[0] = rotation_previous_normalized.x;
						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[1] = rotation_previous_normalized.y;
						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[2] = rotation_previous_normalized.z;
						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[3] = rotation_previous_normalized.w;

						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[0] = key_previous->second.m_rigid_transform.m_translation[0];
						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[1] = key_previous->second.m_rigid_transform.m_translation[1];
						rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[2] = key_previous->second.m_rigid_transform.m_translation[2];
					}
				}

				for (uint32_t ik_switch_channel_index = 0U; ik_switch_channel_index < ik_switch_channel_count; ++ik_switch_channel_index)
				{
					auto &found_ik_switch_channel = ik_switch_channels.find(ik_switch_channel_name_strings[ik_switch_channel_index]);

					if (ik_switch_channels.end() != found_ik_switch_channel)
					{
						mcrt_map<uint32_t, bool> const &ik_switch_channel = found_ik_switch_channel->second;

						auto const &key_upper_bound = ik_switch_channel.upper_bound(frame_index);

						if (ik_switch_channel.end() != key_upper_bound)
						{
							if (ik_switch_channel.begin() != key_upper_bound)
							{
								auto const &key_next = key_upper_bound;
								auto const &key_previous = std::prev(key_upper_bound);

								float const sample_time = static_cast<float>(frame_index) + 0.5F;
								assert((0U == max_frame_number) || (sample_time < max_frame_number));

								assert(static_cast<float>(key_next->first) > sample_time);
								assert(sample_time > static_cast<float>(key_previous->first));

								ik_switches[ik_switch_channel_count * frame_index + ik_switch_channel_index] = key_previous->second;
							}
							else
							{
								auto const &key_next = key_upper_bound;

								float const sample_time = static_cast<float>(frame_index) + 0.5F;
								assert((0U == max_frame_number) || (sample_time < max_frame_number));

								assert(static_cast<float>(key_next->first) > sample_time);

								ik_switches[ik_switch_channel_count * frame_index + ik_switch_channel_index] = key_next->second;
							}
						}
						else
						{
							auto const &key_previous = std::prev(key_upper_bound);
							assert(&(*ik_switch_channel.rbegin()) == &(*key_previous));

							float const sample_time = static_cast<float>(frame_index) + 0.5F;
							assert((0U == max_frame_number) || (sample_time < max_frame_number));
							assert(sample_time > static_cast<float>(key_previous->first));

							ik_switches[ik_switch_channel_count * frame_index + ik_switch_channel_index] = key_previous->second;
						}
					}
					else
					{
						assert(false);
					}
				}
			}
		}

		this->m_weight_channel_names = std::move(weight_channel_names);
		this->m_weights = std::move(weights);
		this->m_rigid_transform_channel_names = std::move(rigid_transform_channel_names);
		this->m_rigid_transforms = std::move(rigid_transforms);
		this->m_ik_switch_channel_names = std::move(ik_switch_channel_names);
		this->m_ik_switches = std::move(ik_switches);

		return true;
	}

	void VMDAnimation::Destroy()
	{
		m_model.reset();
	}

	void VMDAnimation::Evaluate(float t)
	{
		uint32_t const frame_count = this->get_frame_count();

		if (frame_count > 0U)
		{
			uint32_t const frame_index = static_cast<uint32_t>(std::min(std::max(static_cast<int64_t>(0), static_cast<int64_t>(t)), static_cast<int64_t>(frame_count - 1U)));

			uint32_t const weight_channel_count = this->m_weight_channel_names.size();
			for (uint32_t weight_channel_index = 0U; weight_channel_index < weight_channel_count; ++weight_channel_index)
			{
				BRX_ASSET_IMPORT_MORPH_TARGET_NAME const morph_target_name = this->m_weight_channel_names[weight_channel_index];
				float const weight = this->m_weights[weight_channel_count * frame_index + weight_channel_index];
				this->m_model->set_morph_target_name_weight(morph_target_name, weight);
			}

			uint32_t const rigid_transform_channel_count = this->m_rigid_transform_channel_names.size();
			for (uint32_t rigid_transform_channel_index = 0U; rigid_transform_channel_index < rigid_transform_channel_count; ++rigid_transform_channel_index)
			{
				MMDNode *const node = this->m_model->GetNodeManager()->GetMMDNode(this->m_rigid_transform_channel_names[rigid_transform_channel_index]);
				if (NULL != node)
				{
					// [Normalized Local Rotation](https://github.com/vrm-c/vrm-specification/blob/master/specification/VRMC_vrm_animation-1.0/how_to_transform_human_pose.md)
					glm::quat normalized_local_rotation(
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[3],
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[0],
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[1],
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_rotation[2]);

					glm::vec3 translation_offset(
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[0],
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[1],
						this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index].m_translation[2]);

					node->SetAnimationRotate(InvZ(normalized_local_rotation));
					node->SetAnimationTranslate(InvZ(translation_offset));
				}
			}

			uint32_t const ik_switch_channel_count = this->m_ik_switch_channel_names.size();
			for (uint32_t ik_switch_channel_index = 0U; ik_switch_channel_index < ik_switch_channel_count; ++ik_switch_channel_index)
			{
				BRX_ASSET_IMPORT_IK_NAME const ik_name = this->m_ik_switch_channel_names[ik_switch_channel_index];
				bool const _switch = this->m_ik_switches[ik_switch_channel_count * frame_index + ik_switch_channel_index];
				this->m_model->set_ik_name_switch(ik_name, _switch);
			}
		}
		else
		{
			assert(false);
		}
	}

	void VMDAnimation::SyncPhysics(float t, int frameCount)
	{
		/*
		すぐにアニメーションを反映すると、Physics が破たんする場合がある。
		例：足がスカートを突き破る等
		アニメーションを反映する際、初期状態から数フレームかけて、
		目的のポーズへ遷移させる。
		*/
		m_model->SaveBaseAnimation();

		// Physicsを反映する
		m_model->BeginAnimation();

		Evaluate((float)t);

		m_model->UpdateMorphAnimation();

		// Bullet Physics: set "maxSubSteps" to "120"
		m_model->UpdateNodeAnimation(true, float(frameCount) / 30.0f);

		m_model->EndAnimation();
	}

	uint32_t VMDAnimation::get_frame_count() const
	{
		if ((!this->m_weight_channel_names.empty()) && (!this->m_rigid_transform_channel_names.empty()) && (!this->m_ik_switch_channel_names.empty()))
		{
			assert(0U == (this->m_weights.size() % this->m_weight_channel_names.size()));
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));
			assert(0U == (this->m_ik_switches.size() % this->m_ik_switch_channel_names.size()));
			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();
			assert((this->m_weights.size() / this->m_weight_channel_names.size()) == frame_count);
			assert((this->m_ik_switches.size() / this->m_ik_switch_channel_names.size()) == frame_count);
			return frame_count;
		}
		else if ((!this->m_weight_channel_names.empty()) && (!this->m_rigid_transform_channel_names.empty()))
		{
			assert(this->m_ik_switch_channel_names.empty());
			assert(this->m_ik_switches.empty());
			assert(0U == (this->m_weights.size() % this->m_weight_channel_names.size()));
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));
			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();
			assert((this->m_weights.size() / this->m_weight_channel_names.size()) == frame_count);
			return frame_count;
		}
		else if ((!this->m_weight_channel_names.empty()) && (!this->m_ik_switch_channel_names.empty()))
		{
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(0U == (this->m_weights.size() % this->m_weight_channel_names.size()));
			assert(0U == (this->m_ik_switches.size() % this->m_ik_switch_channel_names.size()));
			uint32_t const frame_count = this->m_weights.size() / this->m_weight_channel_names.size();
			assert((this->m_ik_switches.size() / this->m_ik_switch_channel_names.size()) == frame_count);
			return frame_count;
		}
		else if ((!this->m_rigid_transform_channel_names.empty()) && (!this->m_ik_switch_channel_names.empty()))
		{
			assert(this->m_weights.empty());
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));
			assert(0U == (this->m_ik_switches.size() % this->m_ik_switch_channel_names.size()));
			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();
			assert((this->m_ik_switches.size() / this->m_ik_switch_channel_names.size()) == frame_count);
			return frame_count;
		}
		else if (!this->m_weight_channel_names.empty())
		{
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(this->m_ik_switch_channel_names.empty());
			assert(this->m_ik_switches.empty());
			assert(0U == (this->m_weights.size() % this->m_rigid_transform_channel_names.size()));
			uint32_t const frame_count = this->m_weights.size() / this->m_rigid_transform_channel_names.size();
			return frame_count;
		}
		else if (!this->m_rigid_transform_channel_names.empty())
		{
			assert(this->m_weights.empty());
			assert(this->m_ik_switch_channel_names.empty());
			assert(this->m_ik_switches.empty());
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));
			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();
			return frame_count;
		}
		else if (!this->m_ik_switch_channel_names.empty())
		{
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(this->m_ik_switch_channel_names.empty());
			assert(this->m_ik_switches.empty());
			assert(0U == (this->m_ik_switches.size() % this->m_ik_switch_channel_names.size()));
			uint32_t const frame_count = (this->m_ik_switches.size() / this->m_ik_switch_channel_names.size());
			return frame_count;
		}
		else
		{
			assert(this->m_weight_channel_names.empty());
			assert(this->m_weights.empty());
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(this->m_ik_switch_channel_names.empty());
			assert(this->m_ik_switches.empty());
			return 0U;
		}
	}
}
