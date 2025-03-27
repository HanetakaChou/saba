//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_MMD_VMDFILE_H_
#define SABA_MODEL_MMD_VMDFILE_H_

#include "MMDFileString.h"

#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <array>
#include <cstdint>

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

#ifndef MCRT
#define MCRT 1
using mcrt_string = std::string;

template <typename T>
using mcrt_vector = std::vector<T>;

template <typename Key, typename T, typename Compare = std::less<Key>>
using mcrt_map = std::map<Key, T, Compare>;

template <typename Key>
using mcrt_unordered_set = std::unordered_set<Key, std::hash<Key>, std::equal_to<Key>>;

template <typename Key, typename T>
using mcrt_unordered_map = std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>>;
#endif

#ifndef BRX_ASSET_IMPORT
#define BRX_ASSET_IMPORT 1
static constexpr uint32_t const BRX_ASSET_IMPORT_UINT32_INDEX_INVALID = static_cast<uint32_t>(~static_cast<uint32_t>(0U));

enum BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_TYPE : uint32_t
{
	BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_COPY_TRANSFORM = 0,
	BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_INVERSE_KINEMATICS = 1
};

enum BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_TYPE : uint32_t
{
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_SPHERE = 0,
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_BOX = 1,
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_CAPSULE = 2
};

enum BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_TYPE : uint32_t
{
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_FIXED = 0,
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_KEYFRAME = 1,
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_DYNAMIC = 2
};

enum BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_TYPE : uint32_t
{
	BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_FIXED = 0,
	BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_BALL_AND_SOCKET = 1,
	BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_HINGE = 2,
	BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_PRISMATIC = 3,
	BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_RAGDOLL = 4
};

struct brx_asset_import_vertex_position
{
	// R32G32B32_FLOAT
	float m_position[3];
};

struct brx_asset_import_vertex_varying
{
	// R16G16_SNORM (octahedron map)
	uint32_t m_normal;
	// R15G15B2_SNORM (octahedron map + tangent w)
	uint32_t m_tangent;
	// R16G16_UNORM
	uint32_t m_texcoord;
};

struct brx_asset_import_vertex_blending
{
	// R16G16B16A16_UINT (xy)
	uint32_t m_indices_xy;
	// R16G16B16A16_UINT (wz)
	uint32_t m_indices_wz;
	// R8G8B8A8_UNORM
	uint32_t m_weights;
};

struct brx_asset_import_rigid_transform
{
	float m_rotation[4];
	float m_translation[3];
};

struct brx_asset_import_skeleton_joint_constraint
{
	BRX_ASSET_IMPORT_SKELETON_JOINT_CONSTRAINT_TYPE m_constraint_type;

	union
	{
		struct
		{
			uint32_t m_source_joint_index;
			uint32_t m_source_weight_count;
			float *m_source_weights;
			uint32_t m_destination_joint_index;
			bool m_copy_rotation;
			bool m_copy_translation;
		} m_copy_transform;

		struct
		{
			uint32_t m_ik_end_effector_index;
			uint32_t m_ik_joint_count;
			uint32_t *m_ik_joint_indices;
			uint32_t m_target_joint_index;
			float m_ik_two_joints_hinge_joint_axis_local_space[3];
			float m_cosine_max_ik_two_joints_hinge_joint_angle;
			float m_cosine_min_ik_two_joints_hinge_joint_angle;
		} m_inverse_kinematics;
	};
};

struct brx_asset_import_physics_rigid_body
{
	brx_asset_import_rigid_transform m_model_space_transform;
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_SHAPE_TYPE m_shape_type;
	float m_shape_size[3];
	BRX_ASSET_IMPORT_PHYSICS_RIGID_BODY_MOTION_TYPE m_motion_type;
	uint32_t m_collision_filter_group;
	uint32_t m_collision_filter_mask;
	float m_mass;
	float m_linear_damping;
	float m_angular_damping;
	float m_friction;
	float m_restitution;
};

struct brx_asset_import_physics_constraint
{
	uint32_t m_rigid_body_a_index;
	uint32_t m_rigid_body_b_index;
	BRX_ASSET_IMPORT_PHYSICS_CONSTRAINT_TYPE m_constraint_type;
	float m_pivot[3];
	float m_twist_axis[3];
	float m_plane_axis[3];
	float m_normal_axis[3];
	float m_twist_limit[2];
	float m_plane_limit[2];
	float m_normal_limit[2];
};

struct brx_asset_import_ragdoll_direct_mapping
{
	uint32_t m_joint_index_a;
	uint32_t m_joint_index_b;
	float m_a_to_b_transform_model_space[4][4];
};
#endif

struct mmd_vmd_vec3_t
{
	float m_x;
	float m_y;
	float m_z;
};

struct mmd_vmd_vec4_t
{
	float m_x;
	float m_y;
	float m_z;
	float m_w;
};

struct mmd_vmd_header_t
{
	mcrt_string m_name;
};

struct mmd_vmd_motion_t
{
	mcrt_string m_name;
	uint32_t m_frame_number;
	mmd_vmd_vec3_t m_translation;
	mmd_vmd_vec4_t m_rotation;
	uint8_t m_translation_x_cubic_bezier[4];
	uint8_t m_translation_y_cubic_bezier[4];
	uint8_t m_translation_z_cubic_bezier[4];
	uint8_t m_rotation_cubic_bezier[4];
};

struct mmd_vmd_morph_t
{
	mcrt_string m_name;
	uint32_t m_frame_number;
	float m_weight;
};

struct mmd_vmd_camera_t
{
	uint32_t m_frame_number;
	mmd_vmd_vec3_t m_focus_position;
	mmd_vmd_vec3_t m_rotation;
	float m_distance;
	float m_fov_angle;
	bool m_orthographic;
	uint8_t m_focus_position_x_cubic_bezier[4];
	uint8_t m_focus_position_y_cubic_bezier[4];
	uint8_t m_focus_position_z_cubic_bezier[4];
	uint8_t m_rotation_cubic_bezier[4];
	uint8_t m_distance_cubic_bezier[4];
	uint8_t m_fov_angle_cubic_bezier[4];
};

struct mmd_vmd_ik_t
{
	mcrt_string m_name;
	uint32_t m_frame_number;
	bool m_enable;
};

struct mmd_vmd_t
{
	mmd_vmd_header_t m_header;
	mcrt_vector<mmd_vmd_motion_t> m_motions;
	mcrt_vector<mmd_vmd_morph_t> m_morphs;
	mcrt_vector<mmd_vmd_camera_t> m_cameras;
	mcrt_vector<mmd_vmd_ik_t> m_iks;
};

namespace saba
{
	struct VMDFile
	{
		mmd_vmd_t m_vmd;
	};

	bool ReadVMDFile(VMDFile *vmd, const char *filename);
}

#endif // !SABA_MODEL_MMD_VMDFILE_H_
