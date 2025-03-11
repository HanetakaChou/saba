//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_MODEL_MMDMODEL_MMDPHYSICS_H_
#define SABA_MODEL_MMDMODEL_MMDPHYSICS_H_

#include "C:/Users/HanetakaChou/Documents/GitHub/Brioche-Physics/thirdparty/Brioche-Physics/include/brx_physics.h"

#include "PMDFile.h"
#include "PMXFile.h"

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include <vector>
#include <memory>
#include <cinttypes>

namespace saba
{
	class MMDPhysics;
	class MMDModel;
	class MMDNode;

	class MMDMotionState;

	class MMDRigidBody
	{
	public:
		MMDRigidBody();
		~MMDRigidBody();
		MMDRigidBody(const MMDRigidBody &rhs) = delete;
		MMDRigidBody &operator=(const MMDRigidBody &rhs) = delete;

		bool Create(const PMDRigidBodyExt &pmdRigidBody, MMDModel *model, MMDNode *node);
		bool Create(const MMDPhysics &physics, const PMXRigidbody &pmxRigidBody, MMDModel *model, MMDNode *node);
		void Destroy(const MMDPhysics &physics);

		uint16_t GetGroup() const;
		uint16_t GetGroupMask() const;

		void SetActivation(bool activation, MMDPhysics *physics, float time);
		void ResetTransform();
		void Reset(MMDPhysics *physics);

		void ReflectGlobalTransform(const MMDPhysics &physics);
		void CalcLocalTransform();

	private:
		enum class RigidBodyType
		{
			Kinematic,
			Dynamic,
			Aligned,
		};

	private:
		RigidBodyType m_rigidBodyType;
		uint16_t m_group;
		uint16_t m_groupMask;

		MMDNode *m_node;
		MMDNode *m_root;

	public:
		glm::mat4 m_initialMat;

	private:
		glm::mat4 m_offsetMat;
		glm::mat4 m_invOffsetMat;

		std::string m_name;

	public:
		brx_physics_rigid_body *m_jph_physics_rigid_body;
	};

	class MMDJoint
	{
	public:
		MMDJoint();
		~MMDJoint();
		MMDJoint(const MMDJoint &rhs) = delete;
		MMDJoint &operator=(const MMDJoint &rhs) = delete;

		bool CreateJoint(const PMDJointExt &pmdJoint, MMDRigidBody *rigidBodyA, MMDRigidBody *rigidBodyB);
		bool CreateJoint(const MMDPhysics &physics, const PMXJoint &pmxJoint, MMDRigidBody *rigidBodyA, MMDRigidBody *rigidBodyB);
		void Destroy(const MMDPhysics &physics);

	private:
	public:
		brx_physics_constraint *m_jph_physics_constraint;
	};

	class MMDPhysics
	{
	public:
		MMDPhysics();
		~MMDPhysics();

		MMDPhysics(const MMDPhysics &rhs) = delete;
		MMDPhysics &operator=(const MMDPhysics &rhs) = delete;

		bool Create();
		void Destroy();

		void SetFPS(float fps);
		float GetFPS() const;
		void SetMaxSubStepCount(int numSteps);
		int GetMaxSubStepCount() const;
		void Update(float time);

		void AddRigidBody(MMDRigidBody *mmdRB);
		void RemoveRigidBody(MMDRigidBody *mmdRB);
		void AddJoint(MMDJoint *mmdJoint);
		void RemoveJoint(MMDJoint *mmdJoint);

	public:
		brx_physics_context *m_jph_physics_context;
		brx_physics_world *m_jph_physics_world;
	};

}
#endif // SABA_MODEL_MMDMODEL_MMDPHYSICS_H_
