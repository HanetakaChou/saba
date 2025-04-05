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
void *const physics_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Physics-BT");
void *const mediapipe_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/mediapipe_c");
void *const motion_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Debug/BRX-Motion");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/McRT-Malloc");
void *const libjpeg_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libjpeg");
void *const libpng_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libpng");
void *const libwebp_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libwebp");
void *const libiconv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/libiconv");
void *const opencv_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/opencv_world3410");
void *const asset_import_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Asset-Import");
void *const physics_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Physics-BT");
void *const mediapipe_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/mediapipe_c");
void *const motion_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Brioche-Asset-Import/build-windows/bin/x64/Release/BRX-Motion");
#endif

decltype(brx_asset_import_create_file_input_stream_factory) *const asset_import_create_file_input_stream_factory = reinterpret_cast<decltype(brx_asset_import_create_file_input_stream_factory) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_create_file_input_stream_factory"));
decltype(brx_asset_import_destroy_file_input_stream_factory) *const asset_import_destroy_file_input_stream_factory = reinterpret_cast<decltype(brx_asset_import_destroy_file_input_stream_factory) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_destroy_file_input_stream_factory"));
decltype(brx_asset_import_create_scene_from_input_stream) *const asset_import_create_scene_from_input_stream = reinterpret_cast<decltype(brx_asset_import_create_scene_from_input_stream) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_create_scene_from_input_stream"));
decltype(brx_asset_import_destroy_scene) *const asset_import_destroy_scene = reinterpret_cast<decltype(brx_asset_import_destroy_scene) *>(_internal_dynamic_link_symbol(asset_import_dynamic_link_handle, "brx_asset_import_destroy_scene"));

decltype(brx_motion_create_animation) *const motion_create_animation = reinterpret_cast<decltype(brx_motion_create_animation) *>(_internal_dynamic_link_symbol(motion_dynamic_link_handle, "brx_motion_create_animation"));
decltype(brx_motion_destroy_animation) *const motion_destroy_animation = reinterpret_cast<decltype(brx_motion_destroy_animation) *>(_internal_dynamic_link_symbol(motion_dynamic_link_handle, "brx_motion_destroy_animation"));
decltype(brx_motion_create_animation_instance) *const motion_create_animation_instance = reinterpret_cast<decltype(brx_motion_create_animation_instance) *>(_internal_dynamic_link_symbol(motion_dynamic_link_handle, "brx_motion_create_animation_instance"));
decltype(brx_motion_destroy_animation_instance) *const motion_destroy_animation_instance = reinterpret_cast<decltype(brx_motion_destroy_animation_instance) *>(_internal_dynamic_link_symbol(motion_dynamic_link_handle, "brx_motion_destroy_animation_instance"));

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

	bool VMDAnimation::Add(const VMDFile &vmd, const char *filename)
	{
		brx_asset_import_input_stream_factory *const input_stream_factory = asset_import_create_file_input_stream_factory();

		brx_asset_import_scene *const asset_import_scene = asset_import_create_scene_from_input_stream(input_stream_factory, filename);

		assert(1U == asset_import_scene->get_animation_count());
		brx_asset_import_animation const *const asset_import_animation = asset_import_scene->get_animation(0);

		assert(NULL == this->m_motion_animation);
		this->m_motion_animation = motion_create_animation(asset_import_animation->get_frame_count(), asset_import_animation->get_weight_channel_count(), wrap(asset_import_animation->get_weight_channel_names()), asset_import_animation->get_weights(), asset_import_animation->get_rigid_transform_channel_count(), wrap(asset_import_animation->get_rigid_transform_channel_names()), wrap(asset_import_animation->get_rigid_transforms()), asset_import_animation->get_switch_channel_count(), wrap(asset_import_animation->get_switch_channel_names()), asset_import_animation->get_switches());

		asset_import_destroy_scene(asset_import_scene);

		asset_import_destroy_file_input_stream_factory(input_stream_factory);

		assert(NULL == this->m_motion_animation_instance);
		this->m_motion_animation_instance = motion_create_animation_instance(this->m_motion_animation);

		return true;
	}

	void VMDAnimation::Destroy()
	{
		m_model.reset();
		assert(NULL != this->m_motion_animation_instance);
		motion_destroy_animation_instance(this->m_motion_animation_instance);
		assert(NULL != this->m_motion_animation);
		motion_destroy_animation(this->m_motion_animation);
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
