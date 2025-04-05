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
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/McRT-Malloc.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Asset-Import.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Motion.lib")
#else
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/McRT-Malloc.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Asset-Import.lib")
#pragma comment(lib, "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Motion.lib")
#endif

static inline BRX_MOTION_MORPH_TARGET_NAME const *wrap(BRX_ASSET_IMPORT_MORPH_TARGET_NAME const *morph_target_name)
{
	static_assert(static_cast<uint32_t>(BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_HAPPY) == static_cast<uint32_t>(BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_HAPPY), "");
	static_assert(static_cast<uint32_t>(BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_ANGRY) == static_cast<uint32_t>(BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_ANGRY), "");
	static_assert(static_cast<uint32_t>(BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SAD) == static_cast<uint32_t>(BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_SAD), "");
	static_assert(static_cast<uint32_t>(BRX_MOTION_MORPH_TARGET_NAME_MMD_BROW_SURPRISED) == static_cast<uint32_t>(BRX_ASSET_IMPORT_MORPH_TARGET_NAME_MMD_BROW_SURPRISED), "");
	return reinterpret_cast<BRX_MOTION_MORPH_TARGET_NAME const *>(morph_target_name);
}

static inline BRX_MOTION_SKELETON_JOINT_NAME const *wrap(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME const *skeleton_joint_name)
{
	static_assert(static_cast<uint32_t>(BRX_MOTION_SKELETON_JOINT_NAME_MMD_CONTROL_NODE) == static_cast<uint32_t>(BRX_ASSET_IMPORT_SKELETON_JOINT_NAME_MMD_CONTROL_NODE), "");
	return reinterpret_cast<BRX_MOTION_SKELETON_JOINT_NAME const *>(skeleton_joint_name);
}

static inline BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *wrap(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME const *skeleton_joint_constraint_name)
{
	static_assert(static_cast<uint32_t>(BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_RIGHT_ANKLE) == static_cast<uint32_t>(BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_NAME_MMD_IK_RIGHT_ANKLE), "");
	return reinterpret_cast<BRX_MOTION_SKELETON_JOINT_CONSTRAINT_NAME const *>(skeleton_joint_constraint_name);
}

static inline brx_motion_rigid_transform const *wrap(brx_asset_import_rigid_transform const *rigid_transform)
{
	static_assert(sizeof(brx_motion_rigid_transform) == sizeof(brx_asset_import_rigid_transform), "");
	return reinterpret_cast<brx_motion_rigid_transform const *>(rigid_transform);
}

namespace saba
{
	VMDAnimation::VMDAnimation() : m_motion_animation(NULL), m_motion_animation_instance(NULL), m_previous_frame_index(0.0F)
	{
	}

	VMDAnimation::~VMDAnimation()
	{
		Destroy();
	}

	bool VMDAnimation::Create(std::shared_ptr<MMDModel> model)
	{
		m_model = model;
		return true;
	}

	bool VMDAnimation::Add(const char *filename)
	{
		brx_asset_import_input_stream_factory *const input_stream_factory = brx_asset_import_create_file_input_stream_factory();

		brx_asset_import_scene *const asset_import_scene = brx_asset_import_create_scene_from_input_stream(input_stream_factory, filename);

		assert(1U == asset_import_scene->get_animation_count());
		brx_asset_import_animation const *const asset_import_animation = asset_import_scene->get_animation(0);

		assert(NULL == this->m_motion_animation);
		this->m_motion_animation = brx_motion_create_animation(asset_import_animation->get_frame_count(), asset_import_animation->get_weight_channel_count(), wrap(asset_import_animation->get_weight_channel_names()), asset_import_animation->get_weights(), asset_import_animation->get_rigid_transform_channel_count(), wrap(asset_import_animation->get_rigid_transform_channel_names()), wrap(asset_import_animation->get_rigid_transforms()), asset_import_animation->get_switch_channel_count(), wrap(asset_import_animation->get_switch_channel_names()), asset_import_animation->get_switches());

		brx_asset_import_destroy_scene(asset_import_scene);

		brx_asset_import_destroy_file_input_stream_factory(input_stream_factory);

		assert(NULL == this->m_motion_animation_instance);
		this->m_motion_animation_instance = brx_motion_create_animation_instance(this->m_motion_animation);

		return true;
	}

	void VMDAnimation::Destroy()
	{
		m_model.reset();
		assert(NULL != this->m_motion_animation_instance);
		brx_motion_destroy_animation_instance(this->m_motion_animation_instance);
		assert(NULL != this->m_motion_animation);
		brx_motion_destroy_animation(this->m_motion_animation);
	}

	void VMDAnimation::Evaluate(float current_frame_index)
	{
		float const delta_time = (current_frame_index - this->m_previous_frame_index) * (1.0 / 30.0F);

		this->m_previous_frame_index = current_frame_index;

		assert(NULL != this->m_motion_animation_instance);
		this->m_motion_animation_instance->step(delta_time);
	}

	void VMDAnimation::SyncPhysics(float current_frame_index, int frameCount)
	{
		assert(false);
	}

	float VMDAnimation::get_morph_target_weight(BRX_ASSET_IMPORT_MORPH_TARGET_NAME morph_target_name) const
	{
		assert(NULL != this->m_motion_animation_instance);
		return this->m_motion_animation_instance->get_morph_target_weight(*wrap(&morph_target_name));
	}

	brx_motion_animation_instance *VMDAnimation::get_motion_animation_instance() const
	{
		return this->m_motion_animation_instance;
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
