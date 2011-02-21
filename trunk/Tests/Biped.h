#ifndef BIPED_H
#define BIPED_H

#include "Box2D.h"
#include "BipedDef.h"

// Ragdoll class thanks to darkzerox.
class Biped
{
public:
	Biped(b2World*, const b2Vec2& position);
	~Biped();
	void SetMotorQ();
	void SetMotorW();
	void SetMotorO();
	void SetMotorP();

public:
	b2World* m_world;
	BipedDef def;

	b2Body				*LFoot, *RFoot, *LCalf, *RCalf, *LThigh, *RThigh,
						*Pelvis, *Chest, *Neck, *Head,
						*LUpperArm, *RUpperArm, *LForearm, *RForearm, *LHand, *RHand;

	b2RevoluteJoint		*LAnkle, *RAnkle, *LKnee, *RKnee, *LHip, *RHip,
						*Abs, *LowerNeck, *UpperNeck,
						*LShoulder, *RShoulder, *LElbow, *RElbow, *LWrist, *RWrist;
};

#endif
