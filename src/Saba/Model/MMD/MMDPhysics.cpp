//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "MMDPhysics.h"

#include "MMDNode.h"
#include "MMDModel.h"
#include "Saba/Base/Log.h"

#include <glm/gtc/matrix_transform.hpp>

#include <DirectXMath.h>

static inline void *_internal_dynamic_link_open(wchar_t const *filename);
static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol);

#ifndef NDEBUG
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Debug/McRT-Malloc");
void *const jph_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Debug/BRX-Physics-BT");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Release/McRT-Malloc");
void *const jph_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Release/BRX-Physics-BT");
#endif
decltype(brx_physics_create_context) *const jph_physics_create_context = reinterpret_cast<decltype(brx_physics_create_context) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_context"));
decltype(brx_physics_destory_context) *const jph_physics_destory_context = reinterpret_cast<decltype(brx_physics_destory_context) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_context"));
decltype(brx_physics_create_world) *const jph_physics_create_world = reinterpret_cast<decltype(brx_physics_create_world) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_world"));
decltype(brx_physics_destory_world) *const jph_physics_destory_world = reinterpret_cast<decltype(brx_physics_destory_world) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_world"));
decltype(brx_physics_world_add_rigid_body) *const jph_physics_world_add_rigid_body = reinterpret_cast<decltype(brx_physics_world_add_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_add_rigid_body"));
decltype(brx_physics_world_remove_rigid_body) *const jph_physics_world_remove_rigid_body = reinterpret_cast<decltype(brx_physics_world_remove_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_remove_rigid_body"));
decltype(brx_physics_world_add_constraint) *const jph_physics_world_add_constraint = reinterpret_cast<decltype(brx_physics_world_add_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_add_constraint"));
decltype(brx_physics_world_remove_constraint) *const jph_physics_world_remove_constraint = reinterpret_cast<decltype(brx_physics_world_remove_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_remove_constraint"));
decltype(brx_physics_world_step) *const jph_physics_world_step = reinterpret_cast<decltype(brx_physics_world_step) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_step"));
decltype(brx_physics_create_rigid_body) *const jph_physics_create_rigid_body = reinterpret_cast<decltype(brx_physics_create_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_rigid_body"));
decltype(brx_physics_destory_rigid_body) *const jph_physics_destory_rigid_body = reinterpret_cast<decltype(brx_physics_destory_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_rigid_body"));
decltype(brx_physics_rigid_body_apply_key_frame) *const jph_physics_rigid_body_apply_key_frame = reinterpret_cast<decltype(brx_physics_rigid_body_apply_key_frame) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_apply_key_frame"));
decltype(brx_physics_rigid_body_get_transform) *const jph_physics_rigid_body_get_transform = reinterpret_cast<decltype(brx_physics_rigid_body_get_transform) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_get_transform"));
decltype(brx_physics_create_constraint) *const jph_physics_create_constraint = reinterpret_cast<decltype(brx_physics_create_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_constraint"));
decltype(brx_physics_destory_constraint) *const jph_physics_destory_constraint = reinterpret_cast<decltype(brx_physics_destory_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_constraint"));

static inline float internal_asin(float y, float z)
{
	constexpr float const INTERNAL_EPSILON = 1E-6F;

	assert(z > INTERNAL_EPSILON);

	if (std::abs(y) < INTERNAL_EPSILON)
	{
		return 0.0F;
	}
	else
	{
		float const sin = (y / z);
		return DirectX::XMScalarASin(sin);
	}
}

namespace saba
{
	namespace
	{
		glm::mat4 InvZ(const glm::mat4 &m)
		{
			const glm::mat4 invZ = glm::scale(glm::mat4(1), glm::vec3(1, 1, -1));
			glm::mat4 result = invZ * m * invZ;
			return result;
		}
	}

	MMDPhysics::MMDPhysics() : m_jph_physics_context(NULL), m_jph_physics_world(NULL)
	{
	}

	MMDPhysics::~MMDPhysics()
	{
		Destroy();
	}

	bool MMDPhysics::Create()
	{
		float gravity[3] = {0, -9.8F * 10.0F, 0};

		assert(NULL == m_jph_physics_context);
		assert(NULL == m_jph_physics_world);
		m_jph_physics_context = jph_physics_create_context();
		m_jph_physics_world = jph_physics_create_world(m_jph_physics_context, gravity);

		return true;
	}

	void MMDPhysics::Destroy()
	{
		assert(NULL != m_jph_physics_context);
		assert(NULL != m_jph_physics_world);
		jph_physics_destory_world(m_jph_physics_context, m_jph_physics_world);
		jph_physics_destory_context(m_jph_physics_context);
		m_jph_physics_context = NULL;
		m_jph_physics_world = NULL;
	}

	void MMDPhysics::SetFPS(float)
	{
		assert(false);
	}

	float MMDPhysics::GetFPS() const
	{
		return 120.0F;
	}

	void MMDPhysics::SetMaxSubStepCount(int)
	{
		assert(false);
	}

	int MMDPhysics::GetMaxSubStepCount() const
	{
		return 10;
	}

	void MMDPhysics::Update(float time)
	{
		if (NULL != m_jph_physics_world)
		{
			jph_physics_world_step(m_jph_physics_context, m_jph_physics_world, time);
		}
	}

	void MMDPhysics::AddRigidBody(MMDRigidBody *mmdRB)
	{
		jph_physics_world_add_rigid_body(m_jph_physics_context, m_jph_physics_world, mmdRB->m_jph_physics_rigid_body);
	}

	void MMDPhysics::RemoveRigidBody(MMDRigidBody *mmdRB)
	{
		jph_physics_world_remove_rigid_body(m_jph_physics_context, m_jph_physics_world, mmdRB->m_jph_physics_rigid_body);
	}

	void MMDPhysics::AddJoint(MMDJoint *mmdJoint)
	{
		jph_physics_world_add_constraint(m_jph_physics_context, m_jph_physics_world, mmdJoint->m_jph_physics_constraint);
	}

	void MMDPhysics::RemoveJoint(MMDJoint *mmdJoint)
	{
		jph_physics_world_remove_constraint(m_jph_physics_context, m_jph_physics_world, mmdJoint->m_jph_physics_constraint);
	}

	//*******************
	// MMDRigidBody
	//*******************

	MMDRigidBody::MMDRigidBody() : m_rigidBodyType(RigidBodyType::Kinematic), m_group(0), m_groupMask(0), m_node(0), m_root(0), m_offsetMat(1), m_jph_physics_rigid_body(NULL)
	{
	}

	MMDRigidBody::~MMDRigidBody()
	{
		assert(NULL == m_jph_physics_rigid_body);
	}

	bool MMDRigidBody::Create(const PMDRigidBodyExt &pmdRigidBody, MMDModel *model, MMDNode *node)
	{
		assert(false);
		return false;
	}

	bool MMDRigidBody::Create(const MMDPhysics &physics, const PMXRigidbody &pmxRigidBody, MMDModel *model, MMDNode *node)
	{
		assert(NULL == m_jph_physics_rigid_body);

		glm::mat4 physics_world_transform;
		{
			// YXZ
			// [FnRigidBody.new_rigid_body_object](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/rigid_body.py#L104)
			glm::mat4 ry = glm::rotate(glm::mat4(1), pmxRigidBody.m_rotate.y, glm::vec3(0, 1, 0));
			glm::mat4 rx = glm::rotate(glm::mat4(1), pmxRigidBody.m_rotate.x, glm::vec3(1, 0, 0));
			glm::mat4 rz = glm::rotate(glm::mat4(1), pmxRigidBody.m_rotate.z, glm::vec3(0, 0, 1));
			glm::mat4 rotMat = ry * rx * rz;

#if 0
			glm::mat4 rotMat3 = glm::transpose(rotMat);

			btMatrix3x3 rotMat2;
			rotMat2.setEulerZYX(pmxRigidBody.m_rotate.x, pmxRigidBody.m_rotate.y, pmxRigidBody.m_rotate.z);

			if (std::abs(rotMat2[0].getX() - rotMat3[0].x) > 1E-6F
				|| std::abs(rotMat2[0].getY() - rotMat3[0].y) > 1E-6F
				|| std::abs(rotMat2[0].getZ() - rotMat3[0].z) > 1E-6F
				|| std::abs(rotMat2[1].getX() - rotMat3[1].x) > 1E-6F
				|| std::abs(rotMat2[1].getY() - rotMat3[1].y) > 1E-6F
				|| std::abs(rotMat2[1].getZ() - rotMat3[1].z) > 1E-6F
				|| std::abs(rotMat2[2].getX() - rotMat3[2].x) > 1E-6F
				|| std::abs(rotMat2[2].getY() - rotMat3[2].y) > 1E-6F
				|| std::abs(rotMat2[2].getZ() - rotMat3[2].z) > 1E-6F
				)
			{
				assert(false);
			}
#endif

			glm::mat4 translateMat = glm::translate(glm::mat4(1), pmxRigidBody.m_translate);

			physics_world_transform = translateMat * rotMat;
		}

		m_initialMat = InvZ(physics_world_transform);

		assert(nullptr == m_node);
		assert(nullptr == m_root);
		if (node != nullptr)
		{
			m_node = node;
			m_offsetMat = glm::inverse(node->GetGlobalTransform()) * m_initialMat;
		}
		else
		{
			m_root = model->GetNodeManager()->GetMMDNode(0);
			m_offsetMat = glm::inverse(m_root->GetGlobalTransform()) * m_initialMat;
		}

		m_invOffsetMat = glm::inverse(m_offsetMat);

		m_rigidBodyType = (RigidBodyType)pmxRigidBody.m_op;
		m_group = pmxRigidBody.m_group;
		m_groupMask = pmxRigidBody.m_collisionGroup;
		m_name = pmxRigidBody.m_name;

		BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE brx_shape_type;
		switch (pmxRigidBody.m_shape)
		{
		case PMXRigidbody::Shape::Sphere:
			brx_shape_type = BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE;
			break;
		case PMXRigidbody::Shape::Box:
			brx_shape_type = BRX_PHYSICS_RIGID_BODY_SHAPE_BOX;
			break;
		default:
			assert(PMXRigidbody::Shape::Capsule == pmxRigidBody.m_shape);
			brx_shape_type = BRX_PHYSICS_RIGID_BODY_SHAPE_CAPSULE;
		}

		float brx_rotation[4];
		float brx_translation[3];
		{
			glm::vec3 translate = glm::vec3(physics_world_transform[3]);
			brx_translation[0] = translate.x;
			brx_translation[1] = translate.y;
			brx_translation[2] = translate.z;

			glm::vec3 scale = glm::vec3(
				glm::length(glm::vec3(physics_world_transform[0])),
				glm::length(glm::vec3(physics_world_transform[1])),
				glm::length(glm::vec3(physics_world_transform[2])));
			assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

			glm::quat rotate = glm::quat_cast(glm::mat3(
				glm::vec3(physics_world_transform[0]) / scale.x,
				glm::vec3(physics_world_transform[1]) / scale.y,
				glm::vec3(physics_world_transform[2]) / scale.z));
			brx_rotation[0] = rotate.x;
			brx_rotation[1] = rotate.y;
			brx_rotation[2] = rotate.z;
			brx_rotation[3] = rotate.w;
		}

		float brx_shape_size[3];
		{
			brx_shape_size[0] = pmxRigidBody.m_shapeSize.x;
			brx_shape_size[1] = pmxRigidBody.m_shapeSize.y;
			brx_shape_size[2] = pmxRigidBody.m_shapeSize.z;
		}

		BRX_PHYSICS_RIGID_BODY_MOTION_TYPE brx_motion_type;
		if (PMXRigidbody::Operation::Static == pmxRigidBody.m_op)
		{
			brx_motion_type = BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME;
		}
		else
		{
			assert(PMXRigidbody::Operation::Dynamic == pmxRigidBody.m_op || PMXRigidbody::Operation::DynamicAndBoneMerge == pmxRigidBody.m_op);
			brx_motion_type = BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC;
		}

		float brx_mass = pmxRigidBody.m_mass;

		float brx_linear_damping = pmxRigidBody.m_translateDimmer;
		float brx_angular_damping = pmxRigidBody.m_rotateDimmer;
		float brx_restitution = pmxRigidBody.m_repulsion;
		float brx_friction = pmxRigidBody.m_friction;

		assert(NULL == m_jph_physics_rigid_body);
		m_jph_physics_rigid_body = jph_physics_create_rigid_body(physics.m_jph_physics_context, physics.m_jph_physics_world, brx_rotation, brx_translation, brx_shape_type, brx_shape_size, brx_motion_type, pmxRigidBody.m_group, pmxRigidBody.m_collisionGroup, brx_mass, brx_linear_damping, brx_angular_damping, brx_friction, brx_restitution);

		return true;
	}

	void MMDRigidBody::Destroy(const MMDPhysics &physics)
	{
		if (NULL != m_jph_physics_rigid_body)
		{
			jph_physics_destory_rigid_body(physics.m_jph_physics_context, physics.m_jph_physics_world, m_jph_physics_rigid_body);

			m_jph_physics_rigid_body = NULL;
		}
	}

	uint16_t MMDRigidBody::GetGroup() const
	{
		return m_group;
	}

	uint16_t MMDRigidBody::GetGroupMask() const
	{
		return m_groupMask;
	}

	void MMDRigidBody::Reset(MMDPhysics *physics)
	{
		assert(false);
	}

	void MMDRigidBody::ResetTransform()
	{
		assert(false);

#if 0
		glm::mat4 mmd_world_transform = m_initialMat;

		glm::mat4 physics_world_transform = InvZ(mmd_world_transform);

		float brx_rotation[4];
		float brx_translation[3];
		{
			glm::vec3 translate = glm::vec3(physics_world_transform[3]);
			brx_translation[0] = translate.x;
			brx_translation[1] = translate.y;
			brx_translation[2] = translate.z;

			glm::vec3 scale = glm::vec3(
				glm::length(glm::vec3(physics_world_transform[0])),
				glm::length(glm::vec3(physics_world_transform[1])),
				glm::length(glm::vec3(physics_world_transform[2])));
			assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

			glm::quat rotate = glm::quat_cast(glm::mat3(
				glm::vec3(physics_world_transform[0]) / scale.x,
				glm::vec3(physics_world_transform[1]) / scale.y,
				glm::vec3(physics_world_transform[2]) / scale.z));
			brx_rotation[0] = rotate.x;
			brx_rotation[1] = rotate.y;
			brx_rotation[2] = rotate.z;
			brx_rotation[3] = rotate.w;
		}

		hk_physics_rigid_body_apply_key_frame(physics->m_hk_physics_context, physics->m_hk_physics_world, m_hk_physics_rigid_body, brx_rotation, brx_translation, time);

		jph_physics_rigid_body_apply_key_frame(physics->m_jph_physics_context, physics->m_jph_physics_world, m_jph_physics_rigid_body, brx_rotation, brx_translation, time);
#endif
	}

	void MMDRigidBody::SetActivation(bool activation, MMDPhysics *physics, float time)
	{
		if (activation)
		{
			// skeleton map: animation to ragdoll

			if (m_rigidBodyType == RigidBodyType::Kinematic)
			{
				glm::mat4 mmd_world_transform = ((nullptr != m_node) ? m_node->GetGlobalTransform() : m_root->GetGlobalTransform()) * m_offsetMat;

				glm::mat4 physics_world_transform = InvZ(mmd_world_transform);

				float brx_rotation[4];
				float brx_translation[3];
				{
					glm::vec3 translate = glm::vec3(physics_world_transform[3]);
					brx_translation[0] = translate.x;
					brx_translation[1] = translate.y;
					brx_translation[2] = translate.z;

					glm::vec3 scale = glm::vec3(
						glm::length(glm::vec3(physics_world_transform[0])),
						glm::length(glm::vec3(physics_world_transform[1])),
						glm::length(glm::vec3(physics_world_transform[2])));
					assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

					glm::quat rotate = glm::quat_cast(glm::mat3(
						glm::vec3(physics_world_transform[0]) / scale.x,
						glm::vec3(physics_world_transform[1]) / scale.y,
						glm::vec3(physics_world_transform[2]) / scale.z));
					brx_rotation[0] = rotate.x;
					brx_rotation[1] = rotate.y;
					brx_rotation[2] = rotate.z;
					brx_rotation[3] = rotate.w;
				}

				jph_physics_rigid_body_apply_key_frame(physics->m_jph_physics_context, physics->m_jph_physics_world, m_jph_physics_rigid_body, brx_rotation, brx_translation, time);
			}
		}
		else
		{
			assert(false);
#if 0
			if (m_rigidBodyType != RigidBodyType::Kinematic)
			{
				m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			}

			glm::mat4 physics_world_transform;
			{
				glm::mat4 mmd_world_transform;

				if (m_node != nullptr)
				{
					mmd_world_transform = m_node->GetGlobalTransform() * m_offsetMat;
				}
				else
				{
					mmd_world_transform = m_root->GetGlobalTransform() * m_offsetMat;
				}

				physics_world_transform = InvZ(mmd_world_transform);
			}
			static_cast<MMDMotionState *>(m_rigidBody->getMotionState())->setWorldTransform(physics_world_transform);

			if (RigidBodyType::Kinematic != m_rigidBodyType)
			{
				m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
			}

			float brx_rotation[4];
			float brx_translation[3];
			{
				glm::vec3 translate = glm::vec3(physics_world_transform[3]);
				brx_translation[0] = translate.x;
				brx_translation[1] = translate.y;
				brx_translation[2] = translate.z;

				glm::vec3 scale = glm::vec3(
					glm::length(glm::vec3(physics_world_transform[0])),
					glm::length(glm::vec3(physics_world_transform[1])),
					glm::length(glm::vec3(physics_world_transform[2])));
				assert(glm::all(glm::epsilonEqual(scale, glm::vec3(1.0F), 1E-3F)));

				glm::quat rotate = glm::quat_cast(glm::mat3(
					glm::vec3(physics_world_transform[0]) / scale.x,
					glm::vec3(physics_world_transform[1]) / scale.y,
					glm::vec3(physics_world_transform[2]) / scale.z));
				brx_rotation[0] = rotate.x;
				brx_rotation[1] = rotate.y;
				brx_rotation[2] = rotate.z;
				brx_rotation[3] = rotate.w;
			}

			hk_physics_rigid_body_apply_key_frame(physics->m_hk_physics_context, physics->m_hk_physics_world, m_hk_physics_rigid_body, brx_rotation, brx_translation, time);

			jph_physics_rigid_body_apply_key_frame(physics->m_jph_physics_context, physics->m_jph_physics_world, m_jph_physics_rigid_body, brx_rotation, brx_translation, time);
#endif
		}
	}

	void MMDRigidBody::ReflectGlobalTransform(const MMDPhysics &physics)
	{
		// skeleton map: ragdoll to animation

		if (nullptr != m_node)
		{
			if (RigidBodyType::Dynamic == m_rigidBodyType)
			{
				glm::mat4 physics_world_transform;
				{
					float brx_rotation[4];
					float brx_position[3];
					// hk_physics_rigid_body_get_transform(physics.m_hk_physics_context, physics.m_hk_physics_world, m_hk_physics_rigid_body, brx_rotation, brx_position);
					jph_physics_rigid_body_get_transform(physics.m_jph_physics_context, physics.m_jph_physics_world, m_jph_physics_rigid_body, brx_rotation, brx_position);

					glm::mat4 rotMat = glm::mat4_cast(glm::quat(brx_rotation[3], brx_rotation[0], brx_rotation[1], brx_rotation[2]));

					glm::mat4 translateMat = glm::translate(glm::mat4(1), glm::vec3(brx_position[0], brx_position[1], brx_position[2]));

					physics_world_transform = translateMat * rotMat;

#if 0
					if (!glm::all(glm::epsilonEqual(physics_world_transform[0], physics_world_transform_2[0], 1E-1F))
						|| !glm::all(glm::epsilonEqual(physics_world_transform[1], physics_world_transform_2[1], 1E-1F))
						|| !glm::all(glm::epsilonEqual(physics_world_transform[2], physics_world_transform_2[2], 1E-1F))
						|| !glm::all(glm::epsilonEqual(physics_world_transform[3], physics_world_transform_2[3], 1E-1F))
						)
					{
						int huhu = 0;
					}
#endif
				}

				glm::mat4 mmd_world_transform = InvZ(physics_world_transform) * m_invOffsetMat;

				m_node->SetGlobalTransform(mmd_world_transform);

				m_node->UpdateChildTransform();
			}
			else if (RigidBodyType::Aligned == m_rigidBodyType)
			{
				glm::mat4 physics_world_transform;

				{
					float brx_rotation[4];
					float brx_position[3];
					// hk_physics_rigid_body_get_transform(physics.m_hk_physics_context, physics.m_hk_physics_world, m_hk_physics_rigid_body, brx_rotation, brx_position);
					jph_physics_rigid_body_get_transform(physics.m_jph_physics_context, physics.m_jph_physics_world, m_jph_physics_rigid_body, brx_rotation, brx_position);

					glm::mat4 rotMat = glm::mat4_cast(glm::quat(brx_rotation[3], brx_rotation[0], brx_rotation[1], brx_rotation[2]));

					glm::mat4 translateMat = glm::translate(glm::mat4(1), glm::vec3(brx_position[0], brx_position[1], brx_position[2]));

					physics_world_transform = translateMat * rotMat;
				}

				glm::mat4 mmd_world_transform = InvZ(physics_world_transform) * m_invOffsetMat;

				glm::vec4 fixed_translation = m_node->GetGlobalTransform()[3];
				mmd_world_transform[3] = fixed_translation;

				m_node->SetGlobalTransform(mmd_world_transform);

				m_node->UpdateChildTransform();
			}
			else
			{
				assert(RigidBodyType::Kinematic == m_rigidBodyType);
			}
		}
	}

	void MMDRigidBody::CalcLocalTransform()
	{
		if (m_node != nullptr)
		{
			auto parent = m_node->GetParent();
			if (parent != nullptr)
			{
				auto local = glm::inverse(parent->GetGlobalTransform()) * m_node->GetGlobalTransform();
				m_node->SetLocalTransform(local);
			}
			else
			{
				m_node->SetLocalTransform(m_node->GetGlobalTransform());
			}
		}
	}

	//*******************
	// MMDJoint
	//*******************
	MMDJoint::MMDJoint() : m_jph_physics_constraint(NULL)
	{
	}

	MMDJoint::~MMDJoint()
	{
	}

	bool MMDJoint::CreateJoint(const PMDJointExt &pmdJoint, MMDRigidBody *rigidBodyA, MMDRigidBody *rigidBodyB)
	{
		assert(false);
		return false;
	}

	bool MMDJoint::CreateJoint(const MMDPhysics &physics, const PMXJoint &pmxJoint, MMDRigidBody *rigidBodyA, MMDRigidBody *rigidBodyB)
	{
		assert(NULL == m_jph_physics_constraint);

		float mmd_translation_limit_min_x = std::min(pmxJoint.m_translateLowerLimit.x, pmxJoint.m_translateUpperLimit.x);
		float mmd_translation_limit_max_x = std::max(pmxJoint.m_translateLowerLimit.x, pmxJoint.m_translateUpperLimit.x);
		float mmd_translation_limit_min_y = std::min(pmxJoint.m_translateLowerLimit.y, pmxJoint.m_translateUpperLimit.y);
		float mmd_translation_limit_max_y = std::max(pmxJoint.m_translateLowerLimit.y, pmxJoint.m_translateUpperLimit.y);
		float mmd_translation_limit_min_z = std::min(pmxJoint.m_translateLowerLimit.z, pmxJoint.m_translateUpperLimit.z);
		float mmd_translation_limit_max_z = std::max(pmxJoint.m_translateLowerLimit.z, pmxJoint.m_translateUpperLimit.z);

		float mmd_rotation_limit_min_x = std::min(pmxJoint.m_rotateLowerLimit.x, pmxJoint.m_rotateUpperLimit.x);
		float mmd_rotation_limit_max_x = std::max(pmxJoint.m_rotateLowerLimit.x, pmxJoint.m_rotateUpperLimit.x);
		float mmd_rotation_limit_min_y = std::min(pmxJoint.m_rotateLowerLimit.y, pmxJoint.m_rotateUpperLimit.y);
		float mmd_rotation_limit_max_y = std::max(pmxJoint.m_rotateLowerLimit.y, pmxJoint.m_rotateUpperLimit.y);
		float mmd_rotation_limit_min_z = std::min(pmxJoint.m_rotateLowerLimit.z, pmxJoint.m_rotateUpperLimit.z);
		float mmd_rotation_limit_max_z = std::max(pmxJoint.m_rotateLowerLimit.z, pmxJoint.m_rotateUpperLimit.z);

		DirectX::XMFLOAT3 mmd_joint_local_origin;
		{
			mmd_joint_local_origin.x = pmxJoint.m_translate.x;
			mmd_joint_local_origin.y = pmxJoint.m_translate.y;
			mmd_joint_local_origin.z = pmxJoint.m_translate.z;
		}

		DirectX::XMFLOAT3 mmd_joint_local_axis_x;
		DirectX::XMFLOAT3 mmd_joint_local_axis_y;
		DirectX::XMFLOAT3 mmd_joint_local_axis_z;
		{
			// YXZ
			// [FnRigidBody.new_joint_object](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/rigid_body.py#L202)
			glm::mat4 ry = glm::rotate(glm::mat4(1), pmxJoint.m_rotate.y, glm::vec3(0, 1, 0));
			glm::mat4 rx = glm::rotate(glm::mat4(1), pmxJoint.m_rotate.x, glm::vec3(1, 0, 0));
			glm::mat4 rz = glm::rotate(glm::mat4(1), pmxJoint.m_rotate.z, glm::vec3(0, 0, 1));
			glm::mat4 rotMat = ry * rx * rz;

			glm::vec3 _mmd_joint_local_axis_x = glm::vec3(rotMat * glm::vec4(1.0, 0.0, 0.0, 0.0));
			glm::vec3 _mmd_joint_local_axis_y = glm::vec3(rotMat * glm::vec4(0.0, 1.0, 0.0, 0.0));
			glm::vec3 _mmd_joint_local_axis_z = glm::vec3(rotMat * glm::vec4(0.0, 0.0, 1.0, 0.0));

			mmd_joint_local_axis_x = DirectX::XMFLOAT3(_mmd_joint_local_axis_x.x, _mmd_joint_local_axis_x.y, _mmd_joint_local_axis_x.z);
			mmd_joint_local_axis_y = DirectX::XMFLOAT3(_mmd_joint_local_axis_y.x, _mmd_joint_local_axis_y.y, _mmd_joint_local_axis_y.z);
			mmd_joint_local_axis_z = DirectX::XMFLOAT3(_mmd_joint_local_axis_z.x, _mmd_joint_local_axis_z.y, _mmd_joint_local_axis_z.z);
		}

		DirectX::XMFLOAT4 rigid_body_b_rotation;
		DirectX::XMFLOAT3 rigid_body_b_translation;
		jph_physics_rigid_body_get_transform(physics.m_jph_physics_context, physics.m_jph_physics_world, rigidBodyB->m_jph_physics_rigid_body, &rigid_body_b_rotation.x, &rigid_body_b_translation.x);

		// NOTE: when the simulate step is too large (e.g., the debug version), the constaints may not work correctly

		BRX_PHYSICS_CONSTRAINT_TYPE brx_constraint_type;
		DirectX::XMFLOAT3 brx_pivot;
		DirectX::XMFLOAT3 brx_twist_axis;
		DirectX::XMFLOAT3 brx_plane_axis;
		DirectX::XMFLOAT3 brx_normal_axis;
		float brx_twist_limit[2];
		float brx_plane_limit[2];
		float brx_normal_limit[2];
		{
			brx_pivot = mmd_joint_local_origin;

			constexpr float const INTERNAL_EPSILON = 1E-6F;
			constexpr float const INTERNAL_PI = DirectX::XM_PI;
			constexpr float const INTERNAL_NEAR_PI_DIV_2 = DirectX::XM_PIDIV2 - INTERNAL_EPSILON;

			float mmd_translation_limit_abs_x = std::max(std::abs(mmd_translation_limit_min_x), std::abs(mmd_translation_limit_max_x));
			float mmd_translation_limit_abs_y = std::max(std::abs(mmd_translation_limit_min_y), std::abs(mmd_translation_limit_max_y));
			float mmd_translation_limit_abs_z = std::max(std::abs(mmd_translation_limit_min_z), std::abs(mmd_translation_limit_max_z));

			float mmd_rotation_limit_abs_x = std::max(std::abs(mmd_rotation_limit_min_x), std::abs(mmd_rotation_limit_max_x));
			float mmd_rotation_limit_abs_y = std::max(std::abs(mmd_rotation_limit_min_y), std::abs(mmd_rotation_limit_max_y));
			float mmd_rotation_limit_abs_z = std::max(std::abs(mmd_rotation_limit_min_z), std::abs(mmd_rotation_limit_max_z));

			if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_x <= INTERNAL_EPSILON && mmd_translation_limit_abs_y <= INTERNAL_EPSILON && mmd_translation_limit_abs_z <= INTERNAL_EPSILON)
			{
				brx_constraint_type = BRX_PHYSICS_CONSTRAINT_FIXED;

				brx_twist_axis = mmd_joint_local_axis_x;

				brx_plane_axis = mmd_joint_local_axis_y;

				brx_normal_axis = mmd_joint_local_axis_z;

				brx_twist_limit[0] = 0.0F;
				brx_twist_limit[1] = 0.0F;

				brx_plane_limit[0] = 0.0F;
				brx_plane_limit[1] = 0.0F;

				brx_normal_limit[0] = 0.0F;
				brx_normal_limit[1] = 0.0F;
			}
			else if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_x <= INTERNAL_EPSILON && mmd_translation_limit_abs_y <= INTERNAL_EPSILON)
			{
				brx_constraint_type = BRX_PHYSICS_CONSTRAINT_PRISMATIC;

				brx_twist_axis = mmd_joint_local_axis_x;

				brx_plane_axis = mmd_joint_local_axis_y;

				brx_normal_axis = mmd_joint_local_axis_z;

				brx_twist_limit[0] = 0.0F;
				brx_twist_limit[1] = 0.0F;

				brx_plane_limit[0] = 0.0F;
				brx_plane_limit[1] = 0.0F;

				brx_normal_limit[0] = mmd_translation_limit_min_z;
				brx_normal_limit[1] = mmd_translation_limit_max_z;
			}
			else if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_y <= INTERNAL_EPSILON && mmd_translation_limit_abs_z <= INTERNAL_EPSILON)
			{
				brx_constraint_type = BRX_PHYSICS_CONSTRAINT_PRISMATIC;

				brx_twist_axis = mmd_joint_local_axis_y;

				brx_plane_axis = mmd_joint_local_axis_z;

				brx_normal_axis = mmd_joint_local_axis_x;

				brx_twist_limit[0] = 0.0F;
				brx_twist_limit[1] = 0.0F;

				brx_plane_limit[0] = 0.0F;
				brx_plane_limit[1] = 0.0F;

				brx_normal_limit[0] = mmd_translation_limit_min_x;
				brx_normal_limit[1] = mmd_translation_limit_max_x;
			}
			else if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_z <= INTERNAL_EPSILON && mmd_translation_limit_abs_x <= INTERNAL_EPSILON)
			{
				brx_constraint_type = BRX_PHYSICS_CONSTRAINT_PRISMATIC;

				brx_twist_axis = mmd_joint_local_axis_z;

				brx_plane_axis = mmd_joint_local_axis_x;

				brx_normal_axis = mmd_joint_local_axis_y;

				brx_twist_limit[0] = 0.0F;
				brx_twist_limit[1] = 0.0F;

				brx_plane_limit[0] = 0.0F;
				brx_plane_limit[1] = 0.0F;

				brx_normal_limit[0] = mmd_translation_limit_min_y;
				brx_normal_limit[1] = mmd_translation_limit_max_y;
			}
			else
			{
				// convert from translation to rotation
				if (mmd_translation_limit_abs_x >= INTERNAL_EPSILON || mmd_translation_limit_abs_y >= INTERNAL_EPSILON || mmd_translation_limit_abs_z >= INTERNAL_EPSILON)
				{
					assert((mmd_rotation_limit_abs_x >= INTERNAL_EPSILON || mmd_rotation_limit_abs_y >= INTERNAL_EPSILON || mmd_rotation_limit_abs_z >= INTERNAL_EPSILON) || (mmd_translation_limit_abs_x >= INTERNAL_EPSILON && mmd_translation_limit_abs_y >= INTERNAL_EPSILON && mmd_translation_limit_abs_z >= INTERNAL_EPSILON));

					float rigid_body_b_body_space_translation_length = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(DirectX::XMLoadFloat3(&rigid_body_b_translation), DirectX::XMLoadFloat3(&mmd_joint_local_origin))));

					if (std::abs(mmd_rotation_limit_abs_x - mmd_rotation_limit_abs_y) <= INTERNAL_EPSILON && std::abs(mmd_rotation_limit_abs_y - mmd_rotation_limit_abs_z) <= INTERNAL_EPSILON && std::abs(mmd_rotation_limit_abs_z - mmd_rotation_limit_abs_x) <= INTERNAL_EPSILON)
					{
						if (mmd_translation_limit_abs_x <= mmd_translation_limit_abs_y && mmd_translation_limit_abs_x <= mmd_translation_limit_abs_z)
						{
							mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));

							mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
						}
						else if (mmd_translation_limit_abs_y <= mmd_translation_limit_abs_z && mmd_translation_limit_abs_y <= mmd_translation_limit_abs_x)
						{
							mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

							mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
						}
						else
						{
							assert(mmd_translation_limit_abs_z <= mmd_translation_limit_abs_x && mmd_translation_limit_abs_z <= mmd_translation_limit_abs_y);

							mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

							mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
							mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
						}
					}
					else if (mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_y && mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_z)
					{
						mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
						mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));

						mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
						mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
					}
					else if (mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_z && mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_x)
					{
						mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_z, rigid_body_b_body_space_translation_length));
						mmd_rotation_limit_min_z = std::min(mmd_rotation_limit_min_z, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

						mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_z, rigid_body_b_body_space_translation_length));
						mmd_rotation_limit_max_z = std::max(mmd_rotation_limit_max_z, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
					}
					else
					{
						assert(mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_x && mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_y);

						mmd_rotation_limit_min_x = std::min(mmd_rotation_limit_min_x, internal_asin(mmd_translation_limit_min_y, rigid_body_b_body_space_translation_length));
						mmd_rotation_limit_min_y = std::min(mmd_rotation_limit_min_y, internal_asin(mmd_translation_limit_min_x, rigid_body_b_body_space_translation_length));

						mmd_rotation_limit_max_x = std::max(mmd_rotation_limit_max_x, internal_asin(mmd_translation_limit_max_y, rigid_body_b_body_space_translation_length));
						mmd_rotation_limit_max_y = std::max(mmd_rotation_limit_max_y, internal_asin(mmd_translation_limit_max_x, rigid_body_b_body_space_translation_length));
					}

					mmd_rotation_limit_abs_x = std::max(std::abs(mmd_rotation_limit_min_x), std::abs(mmd_rotation_limit_max_x));
					mmd_rotation_limit_abs_y = std::max(std::abs(mmd_rotation_limit_min_y), std::abs(mmd_rotation_limit_max_y));
					mmd_rotation_limit_abs_z = std::max(std::abs(mmd_rotation_limit_min_z), std::abs(mmd_rotation_limit_max_z));

					mmd_translation_limit_min_x = 0.0F;
					mmd_translation_limit_max_x = 0.0F;
					mmd_translation_limit_min_y = 0.0F;
					mmd_translation_limit_max_y = 0.0F;
					mmd_translation_limit_min_z = 0.0F;
					mmd_translation_limit_max_z = 0.0F;

					mmd_translation_limit_abs_x = std::max(std::abs(mmd_translation_limit_min_x), std::abs(mmd_translation_limit_max_x));
					mmd_translation_limit_abs_y = std::max(std::abs(mmd_translation_limit_min_y), std::abs(mmd_translation_limit_max_y));
					mmd_translation_limit_abs_z = std::max(std::abs(mmd_translation_limit_min_z), std::abs(mmd_translation_limit_max_z));
				}

				assert(mmd_rotation_limit_abs_x >= INTERNAL_EPSILON || mmd_rotation_limit_abs_y >= INTERNAL_EPSILON || mmd_rotation_limit_abs_z >= INTERNAL_EPSILON);

				if (mmd_rotation_limit_abs_x <= INTERNAL_EPSILON && mmd_rotation_limit_abs_y <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_PHYSICS_CONSTRAINT_HINGE;

					brx_twist_axis = mmd_joint_local_axis_x;

					brx_plane_axis = mmd_joint_local_axis_y;

					brx_normal_axis = mmd_joint_local_axis_z;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = mmd_rotation_limit_min_z;
					brx_normal_limit[1] = mmd_rotation_limit_max_z;
				}
				else if (mmd_rotation_limit_abs_y <= INTERNAL_EPSILON && mmd_rotation_limit_abs_z <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_PHYSICS_CONSTRAINT_HINGE;

					brx_twist_axis = mmd_joint_local_axis_y;

					brx_plane_axis = mmd_joint_local_axis_z;

					brx_normal_axis = mmd_joint_local_axis_x;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = mmd_rotation_limit_min_x;
					brx_normal_limit[1] = mmd_rotation_limit_max_x;
				}
				else if (mmd_rotation_limit_abs_z <= INTERNAL_EPSILON && mmd_rotation_limit_abs_x <= INTERNAL_EPSILON)
				{
					brx_constraint_type = BRX_PHYSICS_CONSTRAINT_HINGE;

					brx_twist_axis = mmd_joint_local_axis_z;

					brx_plane_axis = mmd_joint_local_axis_x;

					brx_normal_axis = mmd_joint_local_axis_y;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = mmd_rotation_limit_min_y;
					brx_normal_limit[1] = mmd_rotation_limit_max_y;
				}
				else if ((std::abs(mmd_rotation_limit_abs_x) >= INTERNAL_NEAR_PI_DIV_2 && std::abs(mmd_rotation_limit_abs_y) >= INTERNAL_NEAR_PI_DIV_2) || (std::abs(mmd_rotation_limit_abs_y) >= INTERNAL_NEAR_PI_DIV_2 && std::abs(mmd_rotation_limit_abs_z) >= INTERNAL_NEAR_PI_DIV_2) || (std::abs(mmd_rotation_limit_abs_z) >= INTERNAL_NEAR_PI_DIV_2 && std::abs(mmd_rotation_limit_abs_x) >= INTERNAL_NEAR_PI_DIV_2))
				{
					brx_constraint_type = BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET;

					brx_twist_axis = mmd_joint_local_axis_x;

					brx_plane_axis = mmd_joint_local_axis_y;

					brx_normal_axis = mmd_joint_local_axis_z;

					brx_twist_limit[0] = 0.0F;
					brx_twist_limit[1] = 0.0F;

					brx_plane_limit[0] = 0.0F;
					brx_plane_limit[1] = 0.0F;

					brx_normal_limit[0] = 0.0F;
					brx_normal_limit[1] = 0.0F;
				}
				else
				{
					brx_constraint_type = BRX_PHYSICS_CONSTRAINT_RAGDOLL;

					if (mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_y && mmd_rotation_limit_abs_x <= mmd_rotation_limit_abs_z)
					{
						brx_twist_axis = mmd_joint_local_axis_x;

						brx_plane_axis = mmd_joint_local_axis_y;

						brx_normal_axis = mmd_joint_local_axis_z;

						brx_twist_limit[0] = mmd_rotation_limit_min_x;
						brx_twist_limit[1] = mmd_rotation_limit_max_x;

						brx_plane_limit[0] = mmd_rotation_limit_min_y;
						brx_plane_limit[1] = mmd_rotation_limit_max_y;

						brx_normal_limit[0] = mmd_rotation_limit_min_z;
						brx_normal_limit[1] = mmd_rotation_limit_max_z;
					}
					else if (mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_z && mmd_rotation_limit_abs_y <= mmd_rotation_limit_abs_x)
					{

						brx_twist_axis = mmd_joint_local_axis_y;

						brx_plane_axis = mmd_joint_local_axis_z;

						brx_normal_axis = mmd_joint_local_axis_x;

						brx_twist_limit[0] = mmd_rotation_limit_min_y;
						brx_twist_limit[1] = mmd_rotation_limit_max_y;

						brx_plane_limit[0] = mmd_rotation_limit_min_z;
						brx_plane_limit[1] = mmd_rotation_limit_max_z;

						brx_normal_limit[0] = mmd_rotation_limit_min_x;
						brx_normal_limit[1] = mmd_rotation_limit_max_x;
					}
					else
					{
						assert(mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_x && mmd_rotation_limit_abs_z <= mmd_rotation_limit_abs_y);

						brx_twist_axis = mmd_joint_local_axis_z;

						brx_plane_axis = mmd_joint_local_axis_x;

						brx_normal_axis = mmd_joint_local_axis_y;

						brx_twist_limit[0] = mmd_rotation_limit_min_z;
						brx_twist_limit[1] = mmd_rotation_limit_max_z;

						brx_plane_limit[0] = mmd_rotation_limit_min_x;
						brx_plane_limit[1] = mmd_rotation_limit_max_x;

						brx_normal_limit[0] = mmd_rotation_limit_min_y;
						brx_normal_limit[1] = mmd_rotation_limit_max_y;
					}
				}
			}
		}

		m_jph_physics_constraint = jph_physics_create_constraint(physics.m_jph_physics_context, physics.m_jph_physics_world, rigidBodyA->m_jph_physics_rigid_body, rigidBodyB->m_jph_physics_rigid_body, brx_constraint_type, &brx_pivot.x, &brx_twist_axis.x, &brx_plane_axis.x, &brx_normal_axis.x, brx_twist_limit, brx_plane_limit, brx_normal_limit);

		return true;
	}

	void MMDJoint::Destroy(const MMDPhysics &physics)
	{
		if (NULL != m_jph_physics_constraint)
		{
			jph_physics_destory_constraint(physics.m_jph_physics_context, physics.m_jph_physics_world, m_jph_physics_constraint);

			m_jph_physics_constraint = NULL;
		}
	}
}

#if defined(__GNUC__)
#error 1
#elif defined(_MSC_VER)
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
