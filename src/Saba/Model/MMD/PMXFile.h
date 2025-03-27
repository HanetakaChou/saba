//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_PMXFILE_H_
#define SABA_MODEL_PMXFILE_H_

#include "MMDFileString.h"

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
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

struct mmd_pmx_vec2_t
{
	float m_x;
	float m_y;
};

struct mmd_pmx_vec3_t
{
	float m_x;
	float m_y;
	float m_z;
};

struct mmd_pmx_vec4_t
{
	float m_x;
	float m_y;
	float m_z;
	float m_w;
};

struct mmd_pmx_header_t
{
	mcrt_string m_name;
	mcrt_string m_comment;
};

struct mmd_pmx_vertex_t
{
	mmd_pmx_vec3_t m_position;
	mmd_pmx_vec3_t m_normal;
	mmd_pmx_vec2_t m_uv;
	uint32_t m_bone_indices[4];
	float m_bone_weights[4];
};

struct mmd_pmx_face_t
{
	uint32_t m_vertex_indices[3];
};

struct mmd_pmx_texture_t
{
	mcrt_string m_path;
};

struct mmd_pmx_material_t
{
	mcrt_string m_name;
	mmd_pmx_vec4_t m_diffuse;
	bool m_is_double_sided;
	uint32_t m_texture_index;
	uint32_t m_face_count;
};

struct mmd_pmx_bone_t
{
	mcrt_string m_name;
	mmd_pmx_vec3_t m_translation;
	uint32_t m_parent_index;
	uint32_t m_transformation_hierarchy;
	bool m_meta_physics;
	bool m_append_rotation;
	bool m_append_translation;
	bool m_append_local;
	uint32_t m_append_parent_index;
	float m_append_rate;
	bool m_ik;
	uint32_t m_ik_end_effector_index;
	mcrt_vector<uint32_t> m_ik_link_indices;
	bool m_ik_two_links_hinge_limit_angle;
	mmd_pmx_vec3_t m_ik_two_links_hinge_limit_angle_min;
	mmd_pmx_vec3_t m_ik_two_links_hinge_limit_angle_max;
};

union mmd_pmx_morph_offset_t
{
	struct
	{
		uint32_t m_morph_index;
		float m_morph_weight;
	} m_group;

	struct
	{
		uint32_t m_vertex_index;
		mmd_pmx_vec3_t m_vertex_position;
	} m_vertex_position;

	struct
	{
		uint32_t m_vertex_index;
		mmd_pmx_vec2_t m_vertex_uv;
	} m_vertex_uv;
};

struct mmd_pmx_morph_t
{
	mcrt_string m_name;
	// 0: group
	// 1: vertex position
	// 2: vertex uv
	// 3: empty
	uint32_t m_morph_type;
	mcrt_vector<mmd_pmx_morph_offset_t> m_offsets;
};

struct mmd_pmx_rigid_body_t
{
	mcrt_string m_name;
	uint32_t m_bone_index;
	uint32_t m_collision_filter_group;
	uint32_t m_collision_filter_mask;
	uint32_t m_shape_type;
	mmd_pmx_vec3_t m_shape_size;
	mmd_pmx_vec3_t m_translation;
	mmd_pmx_vec3_t m_rotation;
	float m_mass;
	float m_linear_damping;
	float m_angular_damping;
	float m_friction;
	float m_restitution;
	uint32_t m_rigid_body_type;
};

struct mmd_pmx_constraint_t
{
	mcrt_string m_name;
	uint32_t m_rigid_body_a_index;
	uint32_t m_rigid_body_b_index;
	mmd_pmx_vec3_t m_translation;
	mmd_pmx_vec3_t m_rotation;
	mmd_pmx_vec3_t m_translation_limit_min;
	mmd_pmx_vec3_t m_translation_limit_max;
	mmd_pmx_vec3_t m_rotation_limit_min;
	mmd_pmx_vec3_t m_rotation_limit_max;
};

struct mmd_pmx_t
{
	mmd_pmx_header_t m_header;
	mcrt_vector<mmd_pmx_vertex_t> m_vertices;
	mcrt_vector<mmd_pmx_face_t> m_faces;
	mcrt_vector<mmd_pmx_texture_t> m_textures;
	mcrt_vector<mmd_pmx_material_t> m_materials;
	mcrt_vector<mmd_pmx_bone_t> m_bones;
	mcrt_vector<mmd_pmx_morph_t> m_morphs;
	mcrt_vector<mmd_pmx_rigid_body_t> m_rigid_bodies;
	mcrt_vector<mmd_pmx_constraint_t> m_constraints;
};

namespace saba
{
	struct PMXFile
	{
		mmd_pmx_t m_pmx;
	};

	bool ReadPMXFile(PMXFile *pmdFile, const char *filename);
}

#endif // !SABA_MODEL_PMXFILE_H_
