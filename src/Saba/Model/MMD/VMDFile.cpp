//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#include "VMDFile.h"

#include <Saba/Base/Log.h>
#include <Saba/Base/File.h>

extern bool internal_data_read_mmd_vmd(void const *data_base, size_t data_size, mmd_vmd_t *out_mmd_vmd);

namespace saba
{
    bool ReadVMDFile(VMDFile *vmd, const char *filename)
    {
        File file;
        if (!file.Open(filename))
        {
            SABA_WARN("VMD File Open Fail. {}", filename);
            return false;
        }

        std::vector<uint8_t> data(static_cast<size_t>(file.GetSize()));
        file.Read(data.data(), file.GetSize());

        if (!internal_data_read_mmd_vmd(data.data(), data.size(), &vmd->m_vmd))
        {
            SABA_INFO("VMD File Read Fail. {}", filename);
            return false;
        }

        SABA_INFO("VMD File Read Successed. {}", filename);

        return true;
    }

}

// [Blender MMD Tools](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py)

#if defined(__GNUC__)
// GCC or CLANG
#define internal_likely(x) __builtin_expect(!!(x), 1)
#define internal_unlikely(x) __builtin_expect(!!(x), 0)
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN_) && __ORDER_LITTLE_ENDIAN_ == __BYTE_ORDER__
static inline uint32_t internal_bswap_32(uint32_t x) { return x; }
#elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && __ORDER_BIG_ENDIAN__ == __BYTE_ORDER__
static inline uint32_t internal_bswap_32(uint32_t x) { return __builtin_bswap32(x); }
#else
#error Unknown Byte Order
#endif
#elif defined(_MSC_VER)
static inline uint32_t internal_bswap_32(uint32_t x) { return x; }
#if defined(__clang__)
// CLANG-CL
#define internal_likely(x) __builtin_expect(!!(x), 1)
#define internal_unlikely(x) __builtin_expect(!!(x), 0)
#else
// MSVC
#define internal_likely(x) (!!(x))
#define internal_unlikely(x) (!!(x))
#endif
#else
#error Unknown Compiler
#endif

static inline bool internal_data_read_mmd_vmd_header(void const *data_base, size_t data_size, size_t &inout_data_offset, mmd_vmd_header_t *out_header);

static inline bool internal_data_read_mmd_vmd_motions(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_motion_t> &out_motions);

static inline bool internal_data_read_mmd_vmd_morphs(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_morph_t> &out_morphs);

static inline bool internal_data_read_mmd_vmd_cameras(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_camera_t> &out_cameras);

static inline bool internal_data_read_mmd_vmd_lights(void const *data_base, size_t data_size, size_t &inout_data_offset);

static inline bool internal_data_read_mmd_vmd_shadows(void const *data_base, size_t data_size, size_t &inout_data_offset);

static inline bool internal_data_read_mmd_vmd_iks(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_ik_t> &out_iks);

static inline bool internal_data_read_mmd_vmd_vec3(void const *data_base, size_t data_size, size_t &inout_data_offset, mmd_vmd_vec3_t *out_vec3);

static inline bool internal_data_read_mmd_vmd_vec4(void const *data_base, size_t data_size, size_t &inout_data_offset, mmd_vmd_vec4_t *out_vec3);

static inline bool internal_data_read_mmd_vmd_text(void const *data_base, size_t data_size, size_t &inout_data_offset, uint32_t length, mcrt_string &out_text);

static inline bool internal_data_read_uint8(void const *data_base, size_t data_size, size_t &inout_data_offset, uint8_t *out_uint8);

static inline bool internal_data_read_uint32(void const *data_base, size_t data_size, size_t &inout_data_offset, uint32_t *out_uint32);

static inline bool internal_data_read_float(void const *data_base, size_t data_size, size_t &inout_data_offset, float *out_float);

static inline bool internal_data_read_bytes(void const *data_base, size_t data_size, size_t &inout_data_offset, uint32_t length, void *out_bytes);

extern bool internal_data_read_mmd_vmd(void const *data_base, size_t data_size, mmd_vmd_t *out_mmd_vmd)
{
    size_t data_offset = 0U;

    if (internal_unlikely(!internal_data_read_mmd_vmd_header(data_base, data_size, data_offset, &out_mmd_vmd->m_header)))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_mmd_vmd_motions(data_base, data_size, data_offset, out_mmd_vmd->m_motions)))
    {
        return false;
    }

    if (data_offset < data_size)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_morphs(data_base, data_size, data_offset, out_mmd_vmd->m_morphs)))
        {
            return false;
        }
    }
    else
    {
        assert(data_size == data_offset);
    }

    if (data_offset < data_size)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_cameras(data_base, data_size, data_offset, out_mmd_vmd->m_cameras)))
        {
            return false;
        }
    }
    else
    {
        assert(data_size == data_offset);
    }

    if (data_offset < data_size)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_lights(data_base, data_size, data_offset)))
        {
            return false;
        }
    }
    else
    {
        assert(data_size == data_offset);
    }

    if (data_offset < data_size)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_shadows(data_base, data_size, data_offset)))
        {
            return false;
        }
    }
    else
    {
        assert(data_size == data_offset);
    }

    if (data_offset < data_size)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_iks(data_base, data_size, data_offset, out_mmd_vmd->m_iks)))
        {
            return false;
        }
    }
    else
    {
        assert(data_size == data_offset);
    }

    assert(data_size == data_offset);

    return true;
}

static inline bool internal_data_read_mmd_vmd_header(void const *data_base, size_t data_size, size_t &inout_data_offset, mmd_vmd_header_t *out_header)
{
    // [Header.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L30)

    mcrt_string signature;
    if (internal_unlikely(!internal_data_read_mmd_vmd_text(data_base, data_size, inout_data_offset, 30U, signature)))
    {
        return false;
    }

    // "Vocaloid Motion Data 0002"
    if (internal_unlikely(!((86U == signature[0]) && (111U == signature[1]) && (99U == signature[2]) && (97U == signature[3]) && (108U == signature[4]) && (111U == signature[5]) && (105U == signature[6]) && (100U == signature[7]) && (32U == signature[8]) && (77U == signature[9]) && (111U == signature[10]) && (116U == signature[11]) && (105U == signature[12]) && (111U == signature[13]) && (110U == signature[14]) && (32U == signature[15]) && (68U == signature[16]) && (97U == signature[17]) && (116U == signature[18]) && (97U == signature[19]) && (32U == signature[20]) && (48U == signature[21]) && (48U == signature[22]) && (48U == signature[23]) && (50U == signature[24]))))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_mmd_vmd_text(data_base, data_size, inout_data_offset, 20U, out_header->m_name)))
    {
        return false;
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_motions(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_motion_t> &out_motions)
{
    // [BoneFrameKey.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L51)

    assert(out_motions.empty());
    out_motions = {};

    uint32_t motion_count;
    if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &motion_count)))
    {
        return false;
    }

    if (internal_unlikely(0U == motion_count || (motion_count > static_cast<uint32_t>(INT32_MAX))))
    {
        // Tolerance
        return true;
    }

    out_motions.resize(motion_count);

    for (uint32_t motion_index = 0U; motion_index < motion_count; ++motion_index)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_text(data_base, data_size, inout_data_offset, 15U, out_motions[motion_index].m_name)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &out_motions[motion_index].m_frame_number)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_mmd_vmd_vec3(data_base, data_size, inout_data_offset, &out_motions[motion_index].m_translation)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_mmd_vmd_vec4(data_base, data_size, inout_data_offset, &out_motions[motion_index].m_rotation)))
        {
            return false;
        }

        uint8_t interpolation[4][16];
        static_assert(sizeof(interpolation) == 64, "");
        if (internal_unlikely(!internal_data_read_bytes(data_base, data_size, inout_data_offset, sizeof(interpolation), &interpolation[0][0])))
        {
            return false;
        }

        // https://blog.goo.ne.jp/torisu_tetosuki/e/bc9f1c4d597341b394bd02b64597499d
        // https://w.atwiki.jp/kumiho_k/pages/15.html

        // assert(interpolation[1][15] == 0U);
        // assert(interpolation[2][14] == 0U);
        // assert(interpolation[3][13] == 0U);
        out_motions[motion_index].m_translation_x_cubic_bezier[0] = interpolation[0][0];
        assert(interpolation[1][3] == interpolation[0][4]);
        assert(interpolation[2][2] == interpolation[0][4]);
        assert(interpolation[3][1] == interpolation[0][4]);
        out_motions[motion_index].m_translation_x_cubic_bezier[1] = interpolation[0][4];
        assert(interpolation[1][7] == interpolation[0][8]);
        assert(interpolation[2][6] == interpolation[0][8]);
        assert(interpolation[3][5] == interpolation[0][8]);
        out_motions[motion_index].m_translation_x_cubic_bezier[2] = interpolation[0][8];
        assert(interpolation[1][11] == interpolation[0][12]);
        assert(interpolation[2][10] == interpolation[0][12]);
        assert(interpolation[3][9] == interpolation[0][12]);
        out_motions[motion_index].m_translation_x_cubic_bezier[3] = interpolation[0][12];

        assert(interpolation[0][1] == interpolation[1][0]);
        // assert(interpolation[2][15] == 0U);
        // assert(interpolation[3][14] == 0U);
        out_motions[motion_index].m_translation_y_cubic_bezier[0] = interpolation[1][0];
        assert(interpolation[0][5] == interpolation[1][4]);
        assert(interpolation[2][3] == interpolation[1][4]);
        assert(interpolation[3][2] == interpolation[1][4]);
        out_motions[motion_index].m_translation_y_cubic_bezier[1] = interpolation[1][4];
        assert(interpolation[0][9] == interpolation[1][8]);
        assert(interpolation[2][7] == interpolation[1][8]);
        assert(interpolation[3][6] == interpolation[1][8]);
        out_motions[motion_index].m_translation_y_cubic_bezier[2] = interpolation[1][8];
        assert(interpolation[0][13] == interpolation[1][12]);
        assert(interpolation[2][11] == interpolation[1][12]);
        assert(interpolation[3][10] == interpolation[1][12]);
        out_motions[motion_index].m_translation_y_cubic_bezier[3] = interpolation[1][12];

        assert(interpolation[0][2] == 0U);
        assert(interpolation[1][1] == interpolation[2][0]);
        // assert(interpolation[3][15] == 0U);
        out_motions[motion_index].m_translation_z_cubic_bezier[0] = interpolation[2][0];
        assert(interpolation[0][6] == interpolation[2][4]);
        assert(interpolation[1][5] == interpolation[2][4]);
        assert(interpolation[3][3] == interpolation[2][4]);
        out_motions[motion_index].m_translation_z_cubic_bezier[1] = interpolation[2][4];
        assert(interpolation[0][10] == interpolation[2][8]);
        assert(interpolation[1][9] == interpolation[2][8]);
        assert(interpolation[3][7] == interpolation[2][8]);
        out_motions[motion_index].m_translation_z_cubic_bezier[2] = interpolation[2][8];
        assert(interpolation[0][14] == interpolation[2][12]);
        assert(interpolation[1][13] == interpolation[2][12]);
        assert(interpolation[3][11] == interpolation[2][12]);
        out_motions[motion_index].m_translation_z_cubic_bezier[3] = interpolation[2][12];

        assert(interpolation[0][3] == 0U);
        assert(interpolation[1][2] == interpolation[3][0]);
        assert(interpolation[2][1] == interpolation[3][0]);
        out_motions[motion_index].m_rotation_cubic_bezier[0] = interpolation[3][0];
        assert(interpolation[0][7] == interpolation[3][4]);
        assert(interpolation[1][6] == interpolation[3][4]);
        assert(interpolation[2][5] == interpolation[3][4]);
        out_motions[motion_index].m_rotation_cubic_bezier[1] = interpolation[3][4];
        assert(interpolation[0][11] == interpolation[3][8]);
        assert(interpolation[1][10] == interpolation[3][8]);
        assert(interpolation[2][9] == interpolation[3][8]);
        out_motions[motion_index].m_rotation_cubic_bezier[2] = interpolation[3][8];
        assert(interpolation[0][15] == interpolation[3][12]);
        assert(interpolation[1][14] == interpolation[3][12]);
        assert(interpolation[2][13] == interpolation[3][12]);
        out_motions[motion_index].m_rotation_cubic_bezier[3] = interpolation[3][12];

        for (uint32_t cubic_bezier_k_index = 0U; cubic_bezier_k_index < 4U; ++cubic_bezier_k_index)
        {
            if (internal_unlikely(out_motions[motion_index].m_translation_x_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_motions[motion_index].m_translation_x_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_motions[motion_index].m_translation_y_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_motions[motion_index].m_translation_y_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_motions[motion_index].m_translation_z_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_motions[motion_index].m_translation_z_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_motions[motion_index].m_rotation_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_motions[motion_index].m_rotation_cubic_bezier[cubic_bezier_k_index] = 0U;
            }
        }
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_morphs(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_morph_t> &out_morphs)
{
    // [ShapeKeyFrameKey.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L73)

    assert(out_morphs.empty());
    out_morphs = {};

    uint32_t morph_count;
    if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &morph_count)))
    {
        return false;
    }

    if (internal_unlikely(0U == morph_count || (morph_count > static_cast<uint32_t>(INT32_MAX))))
    {
        // Tolerance
        return true;
    }

    out_morphs.resize(morph_count);

    for (uint32_t morph_index = 0U; morph_index < morph_count; ++morph_index)
    {
        if (internal_unlikely(!internal_data_read_mmd_vmd_text(data_base, data_size, inout_data_offset, 15U, out_morphs[morph_index].m_name)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &out_morphs[morph_index].m_frame_number)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_morphs[morph_index].m_weight)))
        {
            return false;
        }
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_cameras(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_camera_t> &out_cameras)
{
    // [CameraKeyFrameKey.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L103)

    assert(out_cameras.empty());
    out_cameras = {};

    uint32_t camera_count;
    if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &camera_count)))
    {
        return false;
    }

    if (internal_unlikely(0U == camera_count || (camera_count > static_cast<uint32_t>(INT32_MAX))))
    {
        // Tolerance
        return true;
    }

    out_cameras.resize(camera_count);

    for (uint32_t camera_index = 0U; camera_index < camera_count; ++camera_index)
    {
        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &out_cameras[camera_index].m_frame_number)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_cameras[camera_index].m_distance)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_mmd_vmd_vec3(data_base, data_size, inout_data_offset, &out_cameras[camera_index].m_focus_position)))
        {
            return false;
        }

        if (internal_unlikely(!internal_data_read_mmd_vmd_vec3(data_base, data_size, inout_data_offset, &out_cameras[camera_index].m_rotation)))
        {
            return false;
        }

        uint8_t interpolation[6][4];
        static_assert(sizeof(interpolation) == 24, "");
        if (internal_unlikely(!internal_data_read_bytes(data_base, data_size, inout_data_offset, sizeof(interpolation), &interpolation[0][0])))
        {
            return false;
        }

        // https://blog.goo.ne.jp/torisu_tetosuki/e/bc9f1c4d597341b394bd02b64597499d
        // https://w.atwiki.jp/kumiho_k/pages/15.html

        out_cameras[camera_index].m_focus_position_x_cubic_bezier[0] = interpolation[0][0];
        out_cameras[camera_index].m_focus_position_x_cubic_bezier[1] = interpolation[0][2];
        out_cameras[camera_index].m_focus_position_x_cubic_bezier[2] = interpolation[0][1];
        out_cameras[camera_index].m_focus_position_x_cubic_bezier[3] = interpolation[0][3];

        out_cameras[camera_index].m_focus_position_y_cubic_bezier[0] = interpolation[1][0];
        out_cameras[camera_index].m_focus_position_y_cubic_bezier[1] = interpolation[1][2];
        out_cameras[camera_index].m_focus_position_y_cubic_bezier[2] = interpolation[1][1];
        out_cameras[camera_index].m_focus_position_y_cubic_bezier[3] = interpolation[1][3];

        out_cameras[camera_index].m_focus_position_z_cubic_bezier[0] = interpolation[2][0];
        out_cameras[camera_index].m_focus_position_z_cubic_bezier[1] = interpolation[2][2];
        out_cameras[camera_index].m_focus_position_z_cubic_bezier[2] = interpolation[2][1];
        out_cameras[camera_index].m_focus_position_z_cubic_bezier[3] = interpolation[2][3];

        out_cameras[camera_index].m_rotation_cubic_bezier[0] = interpolation[3][0];
        out_cameras[camera_index].m_rotation_cubic_bezier[1] = interpolation[3][2];
        out_cameras[camera_index].m_rotation_cubic_bezier[2] = interpolation[3][1];
        out_cameras[camera_index].m_rotation_cubic_bezier[3] = interpolation[3][3];

        out_cameras[camera_index].m_distance_cubic_bezier[0] = interpolation[4][0];
        out_cameras[camera_index].m_distance_cubic_bezier[1] = interpolation[4][2];
        out_cameras[camera_index].m_distance_cubic_bezier[2] = interpolation[4][1];
        out_cameras[camera_index].m_distance_cubic_bezier[3] = interpolation[4][3];

        out_cameras[camera_index].m_fov_angle_cubic_bezier[0] = interpolation[5][0];
        out_cameras[camera_index].m_fov_angle_cubic_bezier[1] = interpolation[5][2];
        out_cameras[camera_index].m_fov_angle_cubic_bezier[2] = interpolation[5][1];
        out_cameras[camera_index].m_fov_angle_cubic_bezier[3] = interpolation[5][3];

        for (uint32_t cubic_bezier_k_index = 0U; cubic_bezier_k_index < 4U; ++cubic_bezier_k_index)
        {
            if (internal_unlikely(out_cameras[camera_index].m_focus_position_x_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_cameras[camera_index].m_focus_position_x_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_cameras[camera_index].m_focus_position_y_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_cameras[camera_index].m_focus_position_y_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_cameras[camera_index].m_focus_position_z_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_cameras[camera_index].m_focus_position_z_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_cameras[camera_index].m_rotation_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_cameras[camera_index].m_rotation_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_cameras[camera_index].m_distance_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_cameras[camera_index].m_distance_cubic_bezier[cubic_bezier_k_index] = 0U;
            }

            if (internal_unlikely(out_cameras[camera_index].m_fov_angle_cubic_bezier[cubic_bezier_k_index] > static_cast<uint8_t>(INT8_MAX)))
            {
                // Tolerance
                out_cameras[camera_index].m_fov_angle_cubic_bezier[cubic_bezier_k_index] = 0U;
            }
        }

        uint32_t fov_angle;
        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &fov_angle)))
        {
            return false;
        }
        out_cameras[camera_index].m_fov_angle = static_cast<float>(static_cast<double>(fov_angle) * 0.01745329251994329576923690768489);

        uint8_t orthographic;
        if (internal_unlikely(!internal_data_read_uint8(data_base, data_size, inout_data_offset, &orthographic)))
        {
            return false;
        }
        out_cameras[camera_index].m_orthographic = (0U != orthographic);
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_lights(void const *data_base, size_t data_size, size_t &inout_data_offset)
{
    // [LampKeyFrameKey.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L133)

    uint32_t light_count;
    if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &light_count)))
    {
        return false;
    }

    for (uint32_t light_index = 0U; light_index < light_count; ++light_index)
    {
        uint32_t unused_frame_number;
        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &unused_frame_number)))
        {
            return false;
        }

        mmd_vmd_vec3_t unused_color;
        if (internal_unlikely(!internal_data_read_mmd_vmd_vec3(data_base, data_size, inout_data_offset, &unused_color)))
        {
            return false;
        }

        mmd_vmd_vec3_t unused_translation;
        if (internal_unlikely(!internal_data_read_mmd_vmd_vec3(data_base, data_size, inout_data_offset, &unused_translation)))
        {
            return false;
        }
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_shadows(void const *data_base, size_t data_size, size_t &inout_data_offset)
{
    // [SelfShadowFrameKey.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L163)

    uint32_t shadow_count;
    if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &shadow_count)))
    {
        return false;
    }

    for (uint32_t shadow_index = 0U; shadow_index < shadow_count; ++shadow_index)
    {
        uint32_t unused_frame_number;
        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &unused_frame_number)))
        {
            return false;
        }

        uint8_t unused_shadow_type;
        if (internal_unlikely(!internal_data_read_uint8(data_base, data_size, inout_data_offset, &unused_shadow_type)))
        {
            return false;
        }

        float unused_distance;
        if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &unused_distance)))
        {
            return false;
        }
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_iks(void const *data_base, size_t data_size, size_t &inout_data_offset, mcrt_vector<mmd_vmd_ik_t> &out_iks)
{
    // [PropertyFrameKey.load](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/vmd/__init__.py#L187)

    assert(out_iks.empty());
    out_iks = {};

    uint32_t ik_first_count;
    if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &ik_first_count)))
    {
        return false;
    }

    if (internal_unlikely(0U == ik_first_count || (ik_first_count > static_cast<uint32_t>(INT32_MAX))))
    {
        // Tolerance
        return true;
    }

    for (uint32_t ik_first_index = 0U; ik_first_index < ik_first_count; ++ik_first_index)
    {
        uint32_t frame_number;
        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &frame_number)))
        {
            return false;
        }

        uint8_t unused_ik_type;
        if (internal_unlikely(!internal_data_read_uint8(data_base, data_size, inout_data_offset, &unused_ik_type)))
        {
            return false;
        }

        uint32_t ik_second_count;
        if (internal_unlikely(!internal_data_read_uint32(data_base, data_size, inout_data_offset, &ik_second_count)))
        {
            return false;
        }

        if (internal_unlikely(0U == ik_second_count || (ik_second_count > static_cast<uint32_t>(INT32_MAX))))
        {
            // Tolerance
            assert(false);
        }
        else
        {
            for (uint32_t ik_second_index = 0U; ik_second_index < ik_second_count; ++ik_second_index)
            {
                mcrt_string name;
                if (internal_unlikely(!internal_data_read_mmd_vmd_text(data_base, data_size, inout_data_offset, 20U, name)))
                {
                    return false;
                }

                uint8_t enable;
                if (internal_unlikely(!internal_data_read_uint8(data_base, data_size, inout_data_offset, &enable)))
                {
                    return false;
                }

                out_iks.emplace_back(mmd_vmd_ik_t{std::move(name), frame_number, static_cast<bool>(0U != enable)});
            }
        }
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_vec3(void const *data_base, size_t data_size, size_t &inout_data_offset, mmd_vmd_vec3_t *out_vec3)
{
    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_x)))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_y)))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_z)))
    {
        return false;
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_vec4(void const *data_base, size_t data_size, size_t &inout_data_offset, mmd_vmd_vec4_t *out_vec3)
{
    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_x)))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_y)))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_z)))
    {
        return false;
    }

    if (internal_unlikely(!internal_data_read_float(data_base, data_size, inout_data_offset, &out_vec3->m_w)))
    {
        return false;
    }

    return true;
}

static inline bool internal_data_read_mmd_vmd_text(void const *data_base, size_t data_size, size_t &inout_data_offset, uint32_t length, mcrt_string &out_text)
{
    // use "out_text, s8" to display the UTF-8 string
    // https://learn.microsoft.com/en-us/visualstudio/debugger/format-specifiers-in-cpp

    assert(out_text.empty());
    out_text = {};

    // Tolerance
    mcrt_vector<uint8_t> value;
    value.resize(static_cast<size_t>(length + 1U), static_cast<uint8_t>(0U));
    if (internal_unlikely(!internal_data_read_bytes(data_base, data_size, inout_data_offset, length, value.data())))
    {
        return false;
    }

    std::u16string u16Str = saba::ConvertSjisToU16String(reinterpret_cast<char const *>(value.data()));

    std::string u8Str;
    saba::ConvU16ToU8(u16Str, u8Str);

    out_text = u8Str;

    return true;
}

static inline bool internal_data_read_uint8(void const *data_base, size_t data_size, size_t &inout_data_offset, uint8_t *out_uint8)
{
    if (data_size >= (inout_data_offset + sizeof(uint8_t)))
    {
        (*out_uint8) = (*reinterpret_cast<uint8_t const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset));
        inout_data_offset += sizeof(uint8_t);
        return true;
    }
    else
    {
        return false;
    }
}

static inline bool internal_data_read_uint32(void const *data_base, size_t data_size, size_t &inout_data_offset, uint32_t *out_uint32)
{
    if (data_size >= (inout_data_offset + sizeof(uint32_t)))
    {
        (*out_uint32) = internal_bswap_32(*reinterpret_cast<uint32_t const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset));
        inout_data_offset += sizeof(uint32_t);
        return true;
    }
    else
    {
        return false;
    }
}

static inline bool internal_data_read_float(void const *data_base, size_t data_size, size_t &inout_data_offset, float *out_float)
{
    if (data_size >= (inout_data_offset + sizeof(uint32_t)))
    {
        uint32_t float_as_uint = internal_bswap_32(*reinterpret_cast<uint32_t const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset));
        (*out_float) = (*reinterpret_cast<float *>(&float_as_uint));
        inout_data_offset += sizeof(uint32_t);
        return true;
    }
    else
    {
        return false;
    }
}

static inline bool internal_data_read_bytes(void const *data_base, size_t data_size, size_t &inout_data_offset, uint32_t length, void *out_bytes)
{
    if (data_size >= (inout_data_offset + (sizeof(uint8_t) * length)))
    {
        std::memcpy(out_bytes, reinterpret_cast<void const *>(reinterpret_cast<uintptr_t>(data_base) + inout_data_offset), length);
        inout_data_offset += (sizeof(uint8_t) * length);
        return true;
    }
    else
    {
        return false;
    }
}
