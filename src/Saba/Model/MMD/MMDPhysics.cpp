//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#include "MMDPhysics.h"

#include "MMDNode.h"
#include "MMDModel.h"
#include "Saba/Base/Log.h"

#include <glm/gtc/matrix_transform.hpp>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

static inline void *_internal_dynamic_link_open(wchar_t const *filename);
static inline void *_internal_dynamic_link_symbol(void *handle, char const *symbol);

#ifndef NDEBUG
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Debug/McRT-Malloc");
void *const hk_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Debug/BRX-Physics-HK");
void *const jph_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Debug/BRX-Physics-JPH");
#else
void *const mcrt_malloc_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Release/McRT-Malloc");
void *const hk_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Release/BRX-Physics-HK");
void *const jph_dynamic_link_handle = _internal_dynamic_link_open(L"C:/Users/HanetakaChou/Documents/GitHub/Havok-Wrapper/build-windows/bin/Win32/Release/BRX-Physics-JPH");
#endif
decltype(brx_physics_create_context) *const hk_physics_create_context = reinterpret_cast<decltype(brx_physics_create_context) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_create_context"));
decltype(brx_physics_create_context) *const jph_physics_create_context = reinterpret_cast<decltype(brx_physics_create_context) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_context"));
decltype(brx_physics_destory_context) *const hk_physics_destory_context = reinterpret_cast<decltype(brx_physics_destory_context) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_destory_context"));
decltype(brx_physics_destory_context) *const jph_physics_destory_context = reinterpret_cast<decltype(brx_physics_destory_context) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_context"));
decltype(brx_physics_create_world) *const hk_physics_create_world = reinterpret_cast<decltype(brx_physics_create_world) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_create_world"));
decltype(brx_physics_create_world) *const jph_physics_create_world = reinterpret_cast<decltype(brx_physics_create_world) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_world"));
decltype(brx_physics_destory_world) *const hk_physics_destory_world = reinterpret_cast<decltype(brx_physics_destory_world) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_destory_world"));
decltype(brx_physics_destory_world) *const jph_physics_destory_world = reinterpret_cast<decltype(brx_physics_destory_world) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_world"));
decltype(brx_physics_world_add_rigid_body) *const hk_physics_world_add_rigid_body = reinterpret_cast<decltype(brx_physics_world_add_rigid_body) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_world_add_rigid_body"));
decltype(brx_physics_world_add_rigid_body) *const jph_physics_world_add_rigid_body = reinterpret_cast<decltype(brx_physics_world_add_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_add_rigid_body"));
decltype(brx_physics_world_remove_rigid_body) *const hk_physics_world_remove_rigid_body = reinterpret_cast<decltype(brx_physics_world_remove_rigid_body) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_world_remove_rigid_body"));
decltype(brx_physics_world_remove_rigid_body) *const jph_physics_world_remove_rigid_body = reinterpret_cast<decltype(brx_physics_world_remove_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_remove_rigid_body"));
decltype(brx_physics_world_add_constraint) *const hk_physics_world_add_constraint = reinterpret_cast<decltype(brx_physics_world_add_constraint) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_world_add_constraint"));
decltype(brx_physics_world_add_constraint) *const jph_physics_world_add_constraint = reinterpret_cast<decltype(brx_physics_world_add_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_add_constraint"));
decltype(brx_physics_world_remove_constraint) *const hk_physics_world_remove_constraint = reinterpret_cast<decltype(brx_physics_world_remove_constraint) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_world_remove_constraint"));
decltype(brx_physics_world_remove_constraint) *const jph_physics_world_remove_constraint = reinterpret_cast<decltype(brx_physics_world_remove_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_remove_constraint"));
decltype(brx_physics_world_step) *const hk_physics_world_step = reinterpret_cast<decltype(brx_physics_world_step) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_world_step"));
decltype(brx_physics_world_step) *const jph_physics_world_step = reinterpret_cast<decltype(brx_physics_world_step) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_world_step"));
decltype(brx_physics_create_rigid_body) *const hk_physics_create_rigid_body = reinterpret_cast<decltype(brx_physics_create_rigid_body) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_create_rigid_body"));
decltype(brx_physics_create_rigid_body) *const jph_physics_create_rigid_body = reinterpret_cast<decltype(brx_physics_create_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_rigid_body"));
decltype(brx_physics_destory_rigid_body) *const hk_physics_destory_rigid_body = reinterpret_cast<decltype(brx_physics_destory_rigid_body) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_destory_rigid_body"));
decltype(brx_physics_destory_rigid_body) *const jph_physics_destory_rigid_body = reinterpret_cast<decltype(brx_physics_destory_rigid_body) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_rigid_body"));
decltype(brx_physics_rigid_body_apply_key_frame) *const hk_physics_rigid_body_apply_key_frame = reinterpret_cast<decltype(brx_physics_rigid_body_apply_key_frame) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_rigid_body_apply_key_frame"));
decltype(brx_physics_rigid_body_apply_key_frame) *const jph_physics_rigid_body_apply_key_frame = reinterpret_cast<decltype(brx_physics_rigid_body_apply_key_frame) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_apply_key_frame"));
decltype(brx_physics_rigid_body_get_transform) *const hk_physics_rigid_body_get_transform = reinterpret_cast<decltype(brx_physics_rigid_body_get_transform) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_rigid_body_get_transform"));
decltype(brx_physics_rigid_body_get_transform) *const jph_physics_rigid_body_get_transform = reinterpret_cast<decltype(brx_physics_rigid_body_get_transform) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_rigid_body_get_transform"));
decltype(brx_physics_create_constraint) *const hk_physics_create_constraint = reinterpret_cast<decltype(brx_physics_create_constraint) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_create_constraint"));
decltype(brx_physics_create_constraint) *const jph_physics_create_constraint = reinterpret_cast<decltype(brx_physics_create_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_create_constraint"));
decltype(brx_physics_destory_constraint) *const hk_physics_destory_constraint = reinterpret_cast<decltype(brx_physics_destory_constraint) *>(_internal_dynamic_link_symbol(hk_dynamic_link_handle, "brx_physics_destory_constraint"));
decltype(brx_physics_destory_constraint) *const jph_physics_destory_constraint = reinterpret_cast<decltype(brx_physics_destory_constraint) *>(_internal_dynamic_link_symbol(jph_dynamic_link_handle, "brx_physics_destory_constraint"));

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

	struct MMDFilterCallback : public btOverlapFilterCallback
	{
		bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const override
		{
			auto findIt = std::find_if(
				m_nonFilterProxy.begin(),
				m_nonFilterProxy.end(),
				[proxy0, proxy1](const auto &x)
				{ return x == proxy0 || x == proxy1; });
			if (findIt != m_nonFilterProxy.end())
			{
				return true;
			}
			bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
			collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}

		std::vector<btBroadphaseProxy *> m_nonFilterProxy;
	};

	MMDPhysics::MMDPhysics() : m_fps(120.0f), m_maxSubStepCount(10), m_hk_physics_context(NULL), m_jph_physics_context(NULL), m_hk_physics_world(NULL), m_jph_physics_world(NULL), m_hk_ground_rigid_body(NULL), m_jph_ground_rigid_body(NULL)
	{
	}

	MMDPhysics::~MMDPhysics()
	{
		Destroy();
	}

	bool MMDPhysics::Create()
	{
		float gravity[3] = {0, -9.8F * 10.0F, 0};

		m_broadphase = std::make_unique<btDbvtBroadphase>();
		m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
		m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());

		m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();

		m_world = std::make_unique<btDiscreteDynamicsWorld>(
			m_dispatcher.get(),
			m_broadphase.get(),
			m_solver.get(),
			m_collisionConfig.get());

		m_world->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));

		m_groundShape = std::make_unique<btStaticPlaneShape>(btVector3(0, 1, 0), 0.0f);

		btTransform groundTransform;
		groundTransform.setIdentity();

		m_groundMS = std::make_unique<btDefaultMotionState>(groundTransform);

		btRigidBody::btRigidBodyConstructionInfo groundInfo(0, m_groundMS.get(), m_groundShape.get(), btVector3(0, 0, 0));
		m_groundRB = std::make_unique<btRigidBody>(groundInfo);

		m_world->addRigidBody(m_groundRB.get());

		auto filterCB = std::make_unique<MMDFilterCallback>();
		filterCB->m_nonFilterProxy.push_back(m_groundRB->getBroadphaseProxy());
		m_world->getPairCache()->setOverlapFilterCallback(filterCB.get());
		m_filterCB = std::move(filterCB);

		assert(NULL == m_hk_physics_context);
		assert(NULL == m_hk_physics_world);
		m_hk_physics_context = hk_physics_create_context();
		m_hk_physics_world = hk_physics_create_world(m_hk_physics_context, gravity);

		assert(NULL == m_jph_physics_context);
		assert(NULL == m_jph_physics_world);
		m_jph_physics_context = jph_physics_create_context();
		m_jph_physics_world = jph_physics_create_world(m_jph_physics_context, gravity);

		{
			float const ground_rotation[4] = {0.0F, 0.0F, 0.0F, 1.0F};
			float const ground_position[3] = {0.0F, -1.0F, 0.0F};
			float const ground_shape_size[3] = {50.0F, 2.0F, 50.0F};

			assert(NULL == m_hk_ground_rigid_body);
			m_hk_ground_rigid_body = hk_physics_create_rigid_body(m_hk_physics_context, m_hk_physics_world, ground_rotation, ground_position, BRX_PHYSICS_RIGID_BODY_SHAPE_BOX, ground_shape_size, BRX_PHYSICS_RIGID_BODY_MOTION_FIXED, 0XFFFFU, 0XFFFFU, 0.0, groundInfo.m_linearDamping, groundInfo.m_angularDamping, groundInfo.m_friction, groundInfo.m_restitution);
			// hk_physics_world_add_rigid_body(m_hk_physics_context, m_hk_physics_world, m_hk_ground_rigid_body);

			assert(NULL == m_jph_ground_rigid_body);
			m_jph_ground_rigid_body = jph_physics_create_rigid_body(m_jph_physics_context, m_jph_physics_world, ground_rotation, ground_position, BRX_PHYSICS_RIGID_BODY_SHAPE_BOX, ground_shape_size, BRX_PHYSICS_RIGID_BODY_MOTION_FIXED, 0XFFFFU, 0XFFFFU, 0.0, groundInfo.m_linearDamping, groundInfo.m_angularDamping, groundInfo.m_friction, groundInfo.m_restitution);
			// jph_physics_world_add_rigid_body(m_jph_physics_context, m_jph_physics_world, m_jph_ground_rigid_body);
		}

		return true;
	}

	void MMDPhysics::Destroy()
	{
		assert(NULL != m_hk_ground_rigid_body);
		assert(NULL != m_hk_physics_context);
		assert(NULL != m_hk_physics_world);
		// hk_physics_world_remove_rigid_body(m_hk_physics_context, m_hk_physics_world, m_hk_ground_rigid_body);
		hk_physics_destory_rigid_body(m_hk_physics_context, m_hk_physics_world, m_hk_ground_rigid_body);
		hk_physics_destory_world(m_hk_physics_context, m_hk_physics_world);
		hk_physics_destory_context(m_hk_physics_context);
		m_hk_physics_context = NULL;
		m_hk_physics_world = NULL;
		m_hk_ground_rigid_body = NULL;

		assert(NULL != m_jph_ground_rigid_body);
		assert(NULL != m_jph_physics_context);
		assert(NULL != m_jph_physics_world);
		// jph_physics_world_remove_rigid_body(m_jph_physics_context, m_jph_physics_world, m_jph_ground_rigid_body);
		jph_physics_destory_rigid_body(m_jph_physics_context, m_jph_physics_world, m_jph_ground_rigid_body);
		jph_physics_destory_world(m_jph_physics_context, m_jph_physics_world);
		jph_physics_destory_context(m_jph_physics_context);
		m_jph_physics_context = NULL;
		m_jph_physics_world = NULL;
		m_jph_ground_rigid_body = NULL;

		if (m_world != nullptr && m_groundRB != nullptr)
		{
			m_world->removeRigidBody(m_groundRB.get());
		}

		m_broadphase = nullptr;
		m_collisionConfig = nullptr;
		m_dispatcher = nullptr;
		m_solver = nullptr;
		m_world = nullptr;
		m_groundShape = nullptr;
		m_groundMS = nullptr;
		m_groundRB = nullptr;
	}

	void MMDPhysics::SetFPS(float fps)
	{
		m_fps = fps;
	}

	float MMDPhysics::GetFPS() const
	{
		return static_cast<float>(m_fps);
	}

	void MMDPhysics::SetMaxSubStepCount(int numSteps)
	{
		m_maxSubStepCount = numSteps;
	}

	int MMDPhysics::GetMaxSubStepCount() const
	{
		return m_maxSubStepCount;
	}

	void MMDPhysics::Update(float time)
	{
		if (m_world != nullptr)
		{
			m_world->stepSimulation(time, m_maxSubStepCount, static_cast<btScalar>(1.0 / m_fps));
		}

		if (NULL != m_hk_physics_world)
		{
			hk_physics_world_step(m_hk_physics_context, m_hk_physics_world, time);
		}

		if (NULL != m_jph_physics_world)
		{
			// jph_physics_world_step(m_jph_physics_context, m_jph_physics_world, time);
		}
	}

	void MMDPhysics::AddRigidBody(MMDRigidBody *mmdRB)
	{
		m_world->addRigidBody(
			mmdRB->GetRigidBody(),
			1 << mmdRB->GetGroup(),
			mmdRB->GetGroupMask());

		hk_physics_world_add_rigid_body(m_hk_physics_context, m_hk_physics_world, mmdRB->m_hk_physics_rigid_body);

		jph_physics_world_add_rigid_body(m_jph_physics_context, m_jph_physics_world, mmdRB->m_jph_physics_rigid_body);
	}

	void MMDPhysics::RemoveRigidBody(MMDRigidBody *mmdRB)
	{
		hk_physics_world_remove_rigid_body(m_hk_physics_context, m_hk_physics_world, mmdRB->m_hk_physics_rigid_body);

		jph_physics_world_remove_rigid_body(m_jph_physics_context, m_jph_physics_world, mmdRB->m_jph_physics_rigid_body);

		m_world->removeRigidBody(mmdRB->GetRigidBody());
	}

	void MMDPhysics::AddJoint(MMDJoint *mmdJoint)
	{
		hk_physics_world_add_constraint(m_hk_physics_context, m_hk_physics_world, mmdJoint->m_hk_physics_constraint);

		jph_physics_world_add_constraint(m_jph_physics_context, m_jph_physics_world, mmdJoint->m_jph_physics_constraint);

		if (mmdJoint->GetConstraint() != nullptr)
		{
			m_world->addConstraint(mmdJoint->GetConstraint());
		}
	}

	void MMDPhysics::RemoveJoint(MMDJoint *mmdJoint)
	{
		hk_physics_world_remove_constraint(m_hk_physics_context, m_hk_physics_world, mmdJoint->m_hk_physics_constraint);

		jph_physics_world_remove_constraint(m_jph_physics_context, m_jph_physics_world, mmdJoint->m_jph_physics_constraint);

		if (mmdJoint->GetConstraint() != nullptr)
		{
			m_world->removeConstraint(mmdJoint->GetConstraint());
		}
	}

	btDiscreteDynamicsWorld *MMDPhysics::GetDynamicsWorld() const
	{
		return m_world.get();
	}

	//*******************
	// MMDRigidBody
	//*******************

	class MMDMotionState : public btMotionState
	{
	public:
		MMDMotionState(const glm::mat4 &transform)
		{
			m_transform.setFromOpenGLMatrix(&transform[0][0]);
		}

		void getWorldTransform(glm::mat4 &transform) const
		{
			alignas(16) glm::mat4 temp;
			m_transform.getOpenGLMatrix(&temp[0][0]);
			transform = temp;
		}

		void setWorldTransform(const glm::mat4 &transform)
		{
			alignas(16) glm::mat4 temp = transform;
			m_transform.setFromOpenGLMatrix(&temp[0][0]);
		}

	private:
		btTransform m_transform;

		void getWorldTransform(btTransform &transform) const override
		{
			transform = m_transform;
		}

		void setWorldTransform(const btTransform &transform) override
		{
			m_transform = transform;
		}
	};

	MMDRigidBody::MMDRigidBody() : m_rigidBodyType(RigidBodyType::Kinematic), m_group(0), m_groupMask(0), m_node(0), m_root(0), m_offsetMat(1), m_hk_physics_rigid_body(NULL), m_jph_physics_rigid_body(NULL)
	{
	}

	MMDRigidBody::~MMDRigidBody()
	{
		assert(NULL == m_hk_physics_rigid_body);
		assert(NULL == m_jph_physics_rigid_body);
	}

	bool MMDRigidBody::Create(const PMDRigidBodyExt &pmdRigidBody, MMDModel *model, MMDNode *node)
	{
		assert(false);
		return false;
	}

	bool MMDRigidBody::Create(const MMDPhysics &physics, const PMXRigidbody &pmxRigidBody, MMDModel *model, MMDNode *node)
	{
		assert(nullptr == m_shape.get());
		assert(NULL == m_hk_physics_rigid_body);
		assert(NULL == m_jph_physics_rigid_body);

		switch (pmxRigidBody.m_shape)
		{
		case PMXRigidbody::Shape::Sphere:
			m_shape = std::make_unique<btSphereShape>(pmxRigidBody.m_shapeSize.x);
			break;
		case PMXRigidbody::Shape::Box:
			m_shape = std::make_unique<btBoxShape>(btVector3(
				pmxRigidBody.m_shapeSize.x,
				pmxRigidBody.m_shapeSize.y,
				pmxRigidBody.m_shapeSize.z));
			break;
		case PMXRigidbody::Shape::Capsule:
			m_shape = std::make_unique<btCapsuleShape>(
				pmxRigidBody.m_shapeSize.x,
				pmxRigidBody.m_shapeSize.y);
			break;
		default:
			break;
		}

		if (m_shape == nullptr)
		{
			return false;
		}

		btScalar mass(0.0f);
		if (pmxRigidBody.m_op != PMXRigidbody::Operation::Static)
		{
			mass = pmxRigidBody.m_mass;
		}

		btVector3 localInteria(0, 0, 0);
		if (mass != 0)
		{
			m_shape->calculateLocalInertia(mass, localInteria);
		}

		glm::mat4 physics_world_transform;
		{
			// YXZ
			// [FnRigidBody.new_rigid_body_object](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/rigid_body.py#L104)
			glm::mat4 rx = glm::rotate(glm::mat4(1), pmxRigidBody.m_rotate.x, glm::vec3(1, 0, 0));
			glm::mat4 ry = glm::rotate(glm::mat4(1), pmxRigidBody.m_rotate.y, glm::vec3(0, 1, 0));
			glm::mat4 rz = glm::rotate(glm::mat4(1), pmxRigidBody.m_rotate.z, glm::vec3(0, 0, 1));
			glm::mat4 rotMat = ry * rx * rz;

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

		m_activeMotionState = std::make_unique<MMDMotionState>(physics_world_transform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, m_activeMotionState.get(), m_shape.get(), localInteria);
		rbInfo.m_linearDamping = pmxRigidBody.m_translateDimmer;
		rbInfo.m_angularDamping = pmxRigidBody.m_rotateDimmer;
		rbInfo.m_restitution = pmxRigidBody.m_repulsion;
		rbInfo.m_friction = pmxRigidBody.m_friction;
		rbInfo.m_additionalDamping = true;

		m_rigidBody = std::make_unique<btRigidBody>(rbInfo);
		m_rigidBody->setUserPointer(this);
		m_rigidBody->setSleepingThresholds(0.01f, glm::radians(0.1f));
		m_rigidBody->setActivationState(DISABLE_DEACTIVATION);
		if (pmxRigidBody.m_op == PMXRigidbody::Operation::Static)
		{
			m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		}

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
		float brx_mass;
		if (PMXRigidbody::Operation::Static == pmxRigidBody.m_op)
		{
			brx_motion_type = BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME;
			brx_mass = 1.0F;
		}
		else
		{
			assert(PMXRigidbody::Operation::Dynamic == pmxRigidBody.m_op || PMXRigidbody::Operation::DynamicAndBoneMerge == pmxRigidBody.m_op);
			brx_motion_type = BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC;
			brx_mass = std::max(mass, 1E-7F);
		}

		float brx_linear_damping = pmxRigidBody.m_translateDimmer;
		float brx_angular_damping = pmxRigidBody.m_rotateDimmer;
		float brx_restitution = pmxRigidBody.m_repulsion;
		float brx_friction = pmxRigidBody.m_friction;

		assert(NULL == m_hk_physics_rigid_body);
		m_hk_physics_rigid_body = hk_physics_create_rigid_body(physics.m_hk_physics_context, physics.m_hk_physics_world, brx_rotation, brx_translation, brx_shape_type, brx_shape_size, brx_motion_type, (1U << pmxRigidBody.m_group), pmxRigidBody.m_collisionGroup, brx_mass, brx_linear_damping, brx_angular_damping, brx_friction, brx_restitution);

		assert(NULL == m_jph_physics_rigid_body);
		m_jph_physics_rigid_body = jph_physics_create_rigid_body(physics.m_jph_physics_context, physics.m_jph_physics_world, brx_rotation, brx_translation, brx_shape_type, brx_shape_size, brx_motion_type, (1U << pmxRigidBody.m_group), pmxRigidBody.m_collisionGroup, brx_mass, brx_linear_damping, brx_angular_damping, brx_friction, brx_restitution);

		return true;
	}

	void MMDRigidBody::Destroy(const MMDPhysics &physics)
	{
		m_shape = nullptr;

		if (NULL != m_hk_physics_rigid_body)
		{
			hk_physics_destory_rigid_body(physics.m_hk_physics_context, physics.m_hk_physics_world, m_hk_physics_rigid_body);

			m_hk_physics_rigid_body = NULL;
		}

		if (NULL != m_jph_physics_rigid_body)
		{
			jph_physics_destory_rigid_body(physics.m_jph_physics_context, physics.m_jph_physics_world, m_jph_physics_rigid_body);

			m_jph_physics_rigid_body = NULL;
		}
	}

	btRigidBody *MMDRigidBody::GetRigidBody() const
	{
		return m_rigidBody.get();
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
		auto cache = physics->GetDynamicsWorld()->getPairCache();
		if (cache != nullptr)
		{
			auto dispatcher = physics->GetDynamicsWorld()->getDispatcher();
			cache->cleanProxyFromPairs(m_rigidBody->getBroadphaseHandle(), dispatcher);
		}
		m_rigidBody->setAngularVelocity(btVector3(0, 0, 0));
		m_rigidBody->setLinearVelocity(btVector3(0, 0, 0));
		m_rigidBody->clearForces();
	}

	void MMDRigidBody::ResetTransform()
	{
		glm::mat4 mmd_world_transform = m_initialMat;

		glm::mat4 physics_world_transform = InvZ(mmd_world_transform);

		static_cast<MMDMotionState *>(m_rigidBody->getMotionState())->setWorldTransform(physics_world_transform);
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

				static_cast<MMDMotionState *>(m_rigidBody->getMotionState())->setWorldTransform(physics_world_transform);

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
			}
		}
		else
		{
			assert(false);
#if 1
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

	void MMDRigidBody::ReflectGlobalTransform()
	{
		// skeleton map: ragdoll to animation

		if (nullptr != m_node)
		{
			if (RigidBodyType::Dynamic == m_rigidBodyType)
			{
				glm::mat4 physics_world_transform;
				static_cast<MMDMotionState *>(m_rigidBody->getMotionState())->getWorldTransform(physics_world_transform);

				glm::mat4 mmd_world_transform = InvZ(physics_world_transform) * m_invOffsetMat;

				m_node->SetGlobalTransform(mmd_world_transform);

				m_node->UpdateChildTransform();
			}
			else if (RigidBodyType::Aligned == m_rigidBodyType)
			{
				glm::mat4 physics_world_transform;
				static_cast<MMDMotionState *>(m_rigidBody->getMotionState())->getWorldTransform(physics_world_transform);

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

	glm::mat4 MMDRigidBody::GetTransform()
	{
		btTransform transform = m_rigidBody->getCenterOfMassTransform();
		alignas(16) glm::mat4 mat;
		transform.getOpenGLMatrix(&mat[0][0]);
		return InvZ(mat);
	}

	//*******************
	// MMDJoint
	//*******************
	MMDJoint::MMDJoint() : m_hk_physics_constraint(NULL), m_jph_physics_constraint(NULL)
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
		assert(nullptr == m_constraint.get());
		assert(NULL == m_hk_physics_constraint);
		assert(NULL == m_jph_physics_constraint);

		float brx_twist_axis[3];
		float brx_plane_axis[3];
		float brx_normal_axis[3];
		float brx_twist_angular_limit[2];
		float brx_plane_angular_limit[2];
		float brx_cone_angular_limit[2];
		{
			// YXZ
			// [FnRigidBody.new_joint_object](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/core/rigid_body.py#L202)
			glm::mat4 ry = glm::rotate(glm::mat4(1), pmxJoint.m_rotate.y, glm::vec3(0, 1, 0));
			glm::mat4 rx = glm::rotate(glm::mat4(1), pmxJoint.m_rotate.x, glm::vec3(1, 0, 0));
			glm::mat4 rz = glm::rotate(glm::mat4(1), pmxJoint.m_rotate.z, glm::vec3(0, 0, 1));
			glm::mat4 rotMat = ry * rx * rz;

			glm::vec3 axis_x = glm::vec3(rotMat * glm::vec4(1.0, 0.0, 0.0, 0.0));
			glm::vec3 axis_y = glm::vec3(rotMat * glm::vec4(0.0, 1.0, 0.0, 0.0));
			glm::vec3 axis_z = glm::vec3(rotMat * glm::vec4(0.0, 0.0, 1.0, 0.0));

			float rotation_limit_x = std::max(std::abs(pmxJoint.m_rotateLowerLimit.x), std::abs(pmxJoint.m_rotateUpperLimit.x));
			float rotation_limit_y = std::max(std::abs(pmxJoint.m_rotateLowerLimit.y), std::abs(pmxJoint.m_rotateUpperLimit.y));
			float rotation_limit_z = std::max(std::abs(pmxJoint.m_rotateLowerLimit.z), std::abs(pmxJoint.m_rotateUpperLimit.z));

			if ((rotation_limit_x < rotation_limit_y) && (rotation_limit_x < rotation_limit_z))
			{
				brx_twist_axis[0] = axis_x.x;
				brx_twist_axis[1] = axis_x.y;
				brx_twist_axis[2] = axis_x.z;

				brx_plane_axis[0] = axis_y.x;
				brx_plane_axis[1] = axis_y.y;
				brx_plane_axis[2] = axis_y.z;

				brx_normal_axis[0] = axis_z.x;
				brx_normal_axis[0] = axis_z.y;
				brx_normal_axis[0] = axis_z.z;

				brx_twist_angular_limit[0] = pmxJoint.m_rotateLowerLimit.x;
				brx_twist_angular_limit[1] = pmxJoint.m_rotateUpperLimit.x;

				brx_plane_angular_limit[0] = pmxJoint.m_rotateLowerLimit.y;
				brx_plane_angular_limit[1] = pmxJoint.m_rotateUpperLimit.y;

				brx_cone_angular_limit[0] = pmxJoint.m_rotateLowerLimit.z;
				brx_cone_angular_limit[1] = pmxJoint.m_rotateUpperLimit.z;
			}
			else if ((rotation_limit_y < rotation_limit_z) && (rotation_limit_y < rotation_limit_x))
			{

				brx_twist_axis[0] = axis_y.x;
				brx_twist_axis[1] = axis_y.y;
				brx_twist_axis[2] = axis_y.z;

				brx_plane_axis[0] = axis_z.x;
				brx_plane_axis[1] = axis_z.y;
				brx_plane_axis[2] = axis_z.z;

				brx_normal_axis[0] = axis_x.x;
				brx_normal_axis[0] = axis_x.y;
				brx_normal_axis[0] = axis_x.z;

				brx_twist_angular_limit[0] = pmxJoint.m_rotateLowerLimit.y;
				brx_twist_angular_limit[1] = pmxJoint.m_rotateUpperLimit.y;

				brx_plane_angular_limit[0] = pmxJoint.m_rotateLowerLimit.z;
				brx_plane_angular_limit[1] = pmxJoint.m_rotateUpperLimit.z;

				brx_cone_angular_limit[0] = pmxJoint.m_rotateLowerLimit.x;
				brx_cone_angular_limit[1] = pmxJoint.m_rotateUpperLimit.x;
			}
			else
			{
				assert((rotation_limit_z <= rotation_limit_x) && (rotation_limit_z <= rotation_limit_y));

				brx_twist_axis[0] = axis_z.x;
				brx_twist_axis[1] = axis_z.y;
				brx_twist_axis[2] = axis_z.z;

				brx_plane_axis[0] = axis_x.x;
				brx_plane_axis[1] = axis_x.y;
				brx_plane_axis[2] = axis_x.z;

				brx_normal_axis[0] = axis_y.x;
				brx_normal_axis[0] = axis_y.y;
				brx_normal_axis[0] = axis_y.z;

				brx_twist_angular_limit[0] = pmxJoint.m_rotateLowerLimit.z;
				brx_twist_angular_limit[1] = pmxJoint.m_rotateUpperLimit.z;

				brx_plane_angular_limit[0] = pmxJoint.m_rotateLowerLimit.x;
				brx_plane_angular_limit[1] = pmxJoint.m_rotateUpperLimit.x;

				brx_cone_angular_limit[0] = pmxJoint.m_rotateLowerLimit.y;
				brx_cone_angular_limit[1] = pmxJoint.m_rotateUpperLimit.y;
			}
		}

		if (brx_twist_angular_limit[0] >= brx_twist_angular_limit[1])
		{
			std::swap(brx_twist_angular_limit[0], brx_twist_angular_limit[1]);
		}

		if (brx_plane_angular_limit[0] >= brx_plane_angular_limit[1])
		{
			std::swap(brx_plane_angular_limit[0], brx_plane_angular_limit[1]);
		}

		if (brx_cone_angular_limit[0] >= brx_cone_angular_limit[1])
		{
			std::swap(brx_cone_angular_limit[0], brx_cone_angular_limit[1]);
		}

		{
			btMatrix3x3 rotMat;
			rotMat.setEulerZYX(pmxJoint.m_rotate.x, pmxJoint.m_rotate.y, pmxJoint.m_rotate.z);

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(btVector3(
				pmxJoint.m_translate.x,
				pmxJoint.m_translate.y,
				pmxJoint.m_translate.z));
			transform.setBasis(rotMat);

			btTransform invA = rigidBodyA->GetRigidBody()->getWorldTransform().inverse();
			btTransform invB = rigidBodyB->GetRigidBody()->getWorldTransform().inverse();
			invA = invA * transform;
			invB = invB * transform;

#if 1
			float rotation_limit_x = std::max(std::abs(pmxJoint.m_rotateLowerLimit.x), std::abs(pmxJoint.m_rotateUpperLimit.x));
			float rotation_limit_y = std::max(std::abs(pmxJoint.m_rotateLowerLimit.y), std::abs(pmxJoint.m_rotateUpperLimit.y));
			float rotation_limit_z = std::max(std::abs(pmxJoint.m_rotateLowerLimit.z), std::abs(pmxJoint.m_rotateUpperLimit.z));

			auto constraint = std::make_unique<btConeTwistConstraint>(
				*rigidBodyA->GetRigidBody(),
				*rigidBodyB->GetRigidBody(),
				invA,
				invB);

			// btScalar twist_span = std::max(rotation_limit_x, 1E-6F);
			// btScalar swing_span1 = std::max(rotation_limit_y, 1E-6F);
			// btScalar swing_span2 = std::max(rotation_limit_z, 1E-6F);

			btScalar twist_span = std::max(std::abs(brx_twist_angular_limit[0]), std::abs(brx_twist_angular_limit[1]));
			btScalar swing_span1 = std::max(std::abs(brx_plane_angular_limit[0]), std::abs(brx_plane_angular_limit[1]));
			btScalar swing_span2 = std::max(std::abs(brx_cone_angular_limit[0]), std::abs(brx_cone_angular_limit[1]));

			constraint->setLimit(swing_span1, swing_span2, twist_span);

#else
			auto constraint = std::make_unique<btGeneric6DofSpringConstraint>(
				*rigidBodyA->GetRigidBody(),
				*rigidBodyB->GetRigidBody(),
				invA,
				invB,
				true);
			constraint->setLinearLowerLimit(btVector3(
				pmxJoint.m_translateLowerLimit.x,
				pmxJoint.m_translateLowerLimit.y,
				pmxJoint.m_translateLowerLimit.z));
			constraint->setLinearUpperLimit(btVector3(
				pmxJoint.m_translateUpperLimit.x,
				pmxJoint.m_translateUpperLimit.y,
				pmxJoint.m_translateUpperLimit.z));

			constraint->setAngularLowerLimit(btVector3(
				pmxJoint.m_rotateLowerLimit.x,
				pmxJoint.m_rotateLowerLimit.y,
				pmxJoint.m_rotateLowerLimit.z));
			constraint->setAngularUpperLimit(btVector3(
				pmxJoint.m_rotateUpperLimit.x,
				pmxJoint.m_rotateUpperLimit.y,
				pmxJoint.m_rotateUpperLimit.z));

			if (pmxJoint.m_springTranslateFactor.x != 0)
			{
				constraint->enableSpring(0, true);
				constraint->setStiffness(0, pmxJoint.m_springTranslateFactor.x);
			}
			if (pmxJoint.m_springTranslateFactor.y != 0)
			{
				constraint->enableSpring(1, true);
				constraint->setStiffness(1, pmxJoint.m_springTranslateFactor.y);
			}
			if (pmxJoint.m_springTranslateFactor.z != 0)
			{
				constraint->enableSpring(2, true);
				constraint->setStiffness(2, pmxJoint.m_springTranslateFactor.z);
			}
			if (pmxJoint.m_springRotateFactor.x != 0)
			{
				constraint->enableSpring(3, true);
				constraint->setStiffness(3, pmxJoint.m_springRotateFactor.x);
			}
			if (pmxJoint.m_springRotateFactor.y != 0)
			{
				constraint->enableSpring(4, true);
				constraint->setStiffness(4, pmxJoint.m_springRotateFactor.y);
			}
			if (pmxJoint.m_springRotateFactor.z != 0)
			{
				constraint->enableSpring(5, true);
				constraint->setStiffness(5, pmxJoint.m_springRotateFactor.z);
			}
#endif

			m_constraint = std::move(constraint);
		}

		float brx_pivot[3];
		{
			brx_pivot[0] = pmxJoint.m_translate.x;
			brx_pivot[1] = pmxJoint.m_translate.y;
			brx_pivot[2] = pmxJoint.m_translate.z;
		}

		m_hk_physics_constraint = hk_physics_create_constraint(physics.m_hk_physics_context, physics.m_hk_physics_world, rigidBodyA->m_hk_physics_rigid_body, rigidBodyB->m_hk_physics_rigid_body, BRX_PHYSICS_CONSTRAINT_RAGDOLL, brx_pivot, brx_twist_axis, brx_plane_axis, brx_twist_angular_limit, brx_plane_angular_limit, brx_cone_angular_limit);

		m_jph_physics_constraint = jph_physics_create_constraint(physics.m_jph_physics_context, physics.m_jph_physics_world, rigidBodyA->m_jph_physics_rigid_body, rigidBodyB->m_jph_physics_rigid_body, BRX_PHYSICS_CONSTRAINT_RAGDOLL, brx_pivot, brx_twist_axis, brx_plane_axis, brx_twist_angular_limit, brx_plane_angular_limit, brx_cone_angular_limit);

		return true;
	}

	void MMDJoint::Destroy(const MMDPhysics &physics)
	{
		m_constraint = nullptr;

		if (NULL != m_hk_physics_constraint)
		{
			hk_physics_destory_constraint(physics.m_hk_physics_context, physics.m_hk_physics_world, m_hk_physics_constraint);

			m_hk_physics_constraint = NULL;
		}

		if (NULL != m_jph_physics_constraint)
		{
			jph_physics_destory_constraint(physics.m_jph_physics_context, physics.m_jph_physics_world, m_jph_physics_constraint);

			m_jph_physics_constraint = NULL;
		}
	}

	btTypedConstraint *MMDJoint::GetConstraint() const
	{
		return m_constraint.get();
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
