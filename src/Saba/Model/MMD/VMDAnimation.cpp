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

static inline void *_internal_dynamic_link_open(wchar_t const *filename);
static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol);

#ifndef NDEBUG
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/McRT-Malloc");
void *const libjpeg_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libjpeg");
void *const libpng_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libpng");
void *const libwebp_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libwebp");
void *const libiconv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/libiconv");
void *const opencv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/opencv_world3410");
void *const asset_import_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Asset-Import");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/McRT-Malloc");
void *const libjpeg_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libjpeg");
void *const libpng_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libpng");
void *const libwebp_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libwebp");
void *const libiconv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libiconv");
void *const opencv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/opencv_world3410");
void *const asset_import_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Asset-Import");
#endif

decltype(brx_asset_import_create_file_input_stream_factory) *const asset_import_create_file_input_stream_factory = reinterpret_cast<decltype(brx_asset_import_create_file_input_stream_factory) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_create_file_input_stream_factory"));
decltype(brx_asset_import_destroy_file_input_stream_factory) *const asset_import_destroy_file_input_stream_factory = reinterpret_cast<decltype(brx_asset_import_destroy_file_input_stream_factory) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_destroy_file_input_stream_factory"));
decltype(brx_asset_import_create_scene_from_input_stream) *const asset_import_create_scene_from_input_stream = reinterpret_cast<decltype(brx_asset_import_create_scene_from_input_stream) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_create_scene_from_input_stream"));
decltype(brx_asset_import_destory_scene) *const asset_import_destory_scene = reinterpret_cast<decltype(brx_asset_import_destory_scene) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_destory_scene"));

namespace saba
{
	VMDAnimation::VMDAnimation()
	{
	}

	bool VMDAnimation::Create(std::shared_ptr<MMDModel> model)
	{
		m_model = model;
		return true;
	}

	bool VMDAnimation::Add(const VMDFile &vmd, const char *filename)
	{
		brx_asset_import_input_stream_factory *const input_stream_factory = asset_import_create_file_input_stream_factory();

		brx_asset_import_scene *const scene = asset_import_create_scene_from_input_stream(input_stream_factory, filename);

		assert(1U == scene->get_animation_count());
		brx_asset_import_animation const *const animation = scene->get_animation(0);

		uint32_t const frame_count = animation->get_frame_count();

		uint32_t const weight_channel_count = animation->get_weight_channel_count();

		for (uint32_t weight_channel_index = 0U; weight_channel_index < weight_channel_count; ++weight_channel_index)
		{
			this->m_weight_channel_names.push_back(static_cast<BRX_ASSET_IMPORT_MORPH_TARGET_NAME>(animation->get_weight_channel_name(weight_channel_index)));
		}

		for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
		{
			for (uint32_t weight_channel_index = 0U; weight_channel_index < weight_channel_count; ++weight_channel_index)
			{
				this->m_weights.push_back(animation->get_weight(frame_index, weight_channel_index));
			}
		}

		uint32_t const rigid_transform_channel_count = animation->get_rigid_transform_channel_count();

		for (uint32_t rigid_transform_channel_index = 0U; rigid_transform_channel_index < rigid_transform_channel_count; ++rigid_transform_channel_index)
		{
			this->m_rigid_transform_channel_names.push_back(animation->get_rigid_transform_channel_name(rigid_transform_channel_index));
		}

		for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
		{
			for (uint32_t rigid_transform_channel_index = 0U; rigid_transform_channel_index < rigid_transform_channel_count; ++rigid_transform_channel_index)
			{
				this->m_rigid_transforms.push_back(*animation->get_rigid_transform(frame_index, rigid_transform_channel_index));
			}
		}

		uint32_t const switch_channel_count = animation->get_switch_channel_count();

		for (uint32_t switch_channel_index = 0U; switch_channel_index < switch_channel_count; ++switch_channel_index)
		{
			this->m_switch_channel_names.push_back(animation->get_switch_channel_name(switch_channel_index));
		}

		for (uint32_t frame_index = 0U; frame_index < frame_count; ++frame_index)
		{
			for (uint32_t switch_channel_index = 0U; switch_channel_index < switch_channel_count; ++switch_channel_index)
			{
				this->m_switches.push_back(animation->get_switch(frame_index, switch_channel_index));
			}
		}

		asset_import_destory_scene(scene);

		asset_import_destroy_file_input_stream_factory(input_stream_factory);

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
				BRX_ASSET_IMPORT_SKELETON_JOINT_NAME const skeleton_joint_name = this->m_rigid_transform_channel_names[rigid_transform_channel_index];

				brx_asset_import_rigid_transform const rigid_transform = this->m_rigid_transforms[rigid_transform_channel_count * frame_index + rigid_transform_channel_index];

				this->m_model->set_skeleton_joint_name_rigid_transform(skeleton_joint_name, rigid_transform);
			}

			uint32_t const switch_channel_count = this->m_switch_channel_names.size();
			for (uint32_t switch_channel_index = 0U; switch_channel_index < switch_channel_count; ++switch_channel_index)
			{
				BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME const skeleton_joint_constraint_name = this->m_switch_channel_names[switch_channel_index];

				bool const _switch = this->m_switches[switch_channel_count * frame_index + switch_channel_index];

				this->m_model->set_skeleton_joint_constraint_name_switch(skeleton_joint_constraint_name, _switch);
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
		Evaluate((float)t);

		m_model->UpdateMorphAnimation();

		// Bullet Physics: set "maxSubSteps" to "120"
		m_model->UpdateNodeAnimation(true, float(frameCount) / 30.0f);
	}

	uint32_t VMDAnimation::get_frame_count() const
	{
		if ((!this->m_weight_channel_names.empty()) && (!this->m_rigid_transform_channel_names.empty()) && (!this->m_switch_channel_names.empty()))
		{
			assert(0U == (this->m_weights.size() % this->m_weight_channel_names.size()));
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));
			assert(0U == (this->m_switches.size() % this->m_switch_channel_names.size()));

			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();

			assert((this->m_weights.size() / this->m_weight_channel_names.size()) == frame_count);
			assert((this->m_switches.size() / this->m_switch_channel_names.size()) == frame_count);

			return frame_count;
		}
		else if ((!this->m_weight_channel_names.empty()) && (!this->m_rigid_transform_channel_names.empty()))
		{
			assert(this->m_switch_channel_names.empty());
			assert(this->m_switches.empty());
			assert(0U == (this->m_weights.size() % this->m_weight_channel_names.size()));
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));

			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();

			assert((this->m_weights.size() / this->m_weight_channel_names.size()) == frame_count);

			return frame_count;
		}
		else if ((!this->m_weight_channel_names.empty()) && (!this->m_switch_channel_names.empty()))
		{
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(0U == (this->m_weights.size() % this->m_weight_channel_names.size()));
			assert(0U == (this->m_switches.size() % this->m_switch_channel_names.size()));

			uint32_t const frame_count = this->m_weights.size() / this->m_weight_channel_names.size();

			assert((this->m_switches.size() / this->m_switch_channel_names.size()) == frame_count);

			return frame_count;
		}
		else if ((!this->m_rigid_transform_channel_names.empty()) && (!this->m_switch_channel_names.empty()))
		{
			assert(this->m_weights.empty());
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));
			assert(0U == (this->m_switches.size() % this->m_switch_channel_names.size()));

			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();

			assert((this->m_switches.size() / this->m_switch_channel_names.size()) == frame_count);

			return frame_count;
		}
		else if (!this->m_weight_channel_names.empty())
		{
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(this->m_switch_channel_names.empty());
			assert(this->m_switches.empty());
			assert(0U == (this->m_weights.size() % this->m_rigid_transform_channel_names.size()));

			uint32_t const frame_count = this->m_weights.size() / this->m_rigid_transform_channel_names.size();

			return frame_count;
		}
		else if (!this->m_rigid_transform_channel_names.empty())
		{
			assert(this->m_weights.empty());
			assert(this->m_switch_channel_names.empty());
			assert(this->m_switches.empty());
			assert(0U == (this->m_rigid_transforms.size() % this->m_rigid_transform_channel_names.size()));

			uint32_t const frame_count = this->m_rigid_transforms.size() / this->m_rigid_transform_channel_names.size();

			return frame_count;
		}
		else if (!this->m_switch_channel_names.empty())
		{
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(this->m_switch_channel_names.empty());
			assert(this->m_switches.empty());
			assert(0U == (this->m_switches.size() % this->m_switch_channel_names.size()));

			uint32_t const frame_count = (this->m_switches.size() / this->m_switch_channel_names.size());

			return frame_count;
		}
		else
		{
			assert(this->m_weight_channel_names.empty());
			assert(this->m_weights.empty());
			assert(this->m_rigid_transform_channel_names.empty());
			assert(this->m_rigid_transforms.empty());
			assert(this->m_switch_channel_names.empty());
			assert(this->m_switches.empty());

			return 0U;
		}
	}
}

#if defined(__GNUC__)
#error 1
#elif defined(_MSC_VER)
#define WIN32_LEAN_AND_MEAN 1
#include <Windows.h>
static inline void *_internal_dynamic_link_open(wchar_t const *filename)
{
	HMODULE dynamic_link_handle = GetModuleHandleW(filename);
	if (NULL == dynamic_link_handle)
	{
		assert(ERROR_MOD_NOT_FOUND == GetLastError());

		dynamic_link_handle = LoadLibraryW(filename);
		if (NULL == dynamic_link_handle)
		{
			assert(ERROR_MOD_NOT_FOUND == GetLastError());
			assert(false);
		}
	}

	return dynamic_link_handle;
}

static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol)
{
	return reinterpret_cast<void *>(GetProcAddress(static_cast<HINSTANCE>(handle), symbol));
}
#endif
