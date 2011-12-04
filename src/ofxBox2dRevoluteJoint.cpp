/*
 *  ofxBox2dRevoluteJoint.cpp
 *  jointExample
 *
 *  Created by Nick Hardeman on 1/19/11.
 *  Copyright 2011 Nick Hardeman. All rights reserved.
 *
 */

#include "ofxBox2dRevoluteJoint.h"

//----------------------------------------
ofxBox2dRevoluteJoint::ofxBox2dRevoluteJoint() {
	world = NULL;
	alive = false;
	targetAngle=0;
}

//----------------------------------------
ofxBox2dRevoluteJoint::ofxBox2dRevoluteJoint(b2World* b2world, b2Body* body1, b2Body* body2, b2Vec2 anchor1, b2Vec2 anchor2, float lowerAngle, float upperAngle, bool bCollideConnected) {
	ofxBox2dRevoluteJoint();
	setup(b2world, body1, body2, anchor1, anchor2,lowerAngle, upperAngle, bCollideConnected);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setup(b2World* b2world, b2Body* body1, b2Body* body2, b2Vec2 anchor1, b2Vec2 anchor2, float lowerAngle, float upperAngle, bool bCollideConnected) {
	setWorld(b2world);
	b2RevoluteJointDef jointDef;
	jointDef.Initialize(body1, body2, anchor1);//, anchor2);
	jointDef.collideConnected	= bCollideConnected;
	jointDef.lowerAngle = lowerAngle;
	jointDef.upperAngle = upperAngle;
	jointDef.localAnchorA = anchor1;
	jointDef.localAnchorB = anchor2;
	jointDef.enableLimit=true;
	jointDef.enableMotor=true;
	jointDef.maxMotorTorque=999.0f;
	jointDef.enableMotor=true;
	//jointDef.motorSpeed=1.0f;

	joint						= (b2RevoluteJoint*)world->CreateJoint(&jointDef);


	alive						= true;
}

//----------------------------------------
void ofxBox2dRevoluteJoint::setWorld(b2World* w) {
	if(w == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: setWorld : - box2d world needed -");	
		return;
	}
	world = w;
}

//----------------------------------------
bool ofxBox2dRevoluteJoint::isSetup() {
	if (world == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: world must be set!");
		return false;
	}
	if (joint == NULL) {
		ofLog(OF_LOG_NOTICE, "ofxBox2dRevoluteJoint :: setup function must be called!");
		return false;
	}
	return true;
}

void ofxBox2dRevoluteJoint::setTargetAngle(float angle)
{
	targetAngle = angle;
}

void ofxBox2dRevoluteJoint::goToTargetAngle()
{
	int diff = targetAngle-joint->GetJointAngle();
	joint->SetMotorSpeed(diff==0?0:diff/abs(diff));
}

//----------------------------------------
void ofxBox2dRevoluteJoint::draw() {
	if(!alive) return;
	
	b2Vec2 p1 = joint->GetAnchorA();
	b2Vec2 p2 = joint->GetAnchorB();
	
	p1 *= OFX_BOX2D_SCALE;
	p2 *= OFX_BOX2D_SCALE;
	ofLine(p1.x, p1.y, p2.x, p2.y);
}

//----------------------------------------
void ofxBox2dRevoluteJoint::destroy() {
	if (!isSetup()) return;
	world->DestroyJoint(joint);
	joint = NULL;
	alive = false;
}


//----------------------------------------


//----------------------------------------
ofVec2f ofxBox2dRevoluteJoint::getReactionForce(float inv_dt) const {
	b2Vec2 vec = getReactionForceB2D(inv_dt);
	return ofVec2f(vec.x, vec.y);
}
b2Vec2 ofxBox2dRevoluteJoint::getReactionForceB2D(float inv_dt) const {
	return joint->GetReactionForce(inv_dt);
}
float ofxBox2dRevoluteJoint::getReactionTorque(float inv_dt) const {
	return (float)joint->GetReactionTorque(inv_dt);
}




