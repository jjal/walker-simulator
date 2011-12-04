#pragma once
#include "ofMain.h"
#include "Box2D.h"
#include "ofxBox2dUtils.h"
#include "ofxBox2dRevoluteJoint.h"

class ofxBox2dRevoluteJoint {
	
public:
	
	b2World			*	world;
	b2RevoluteJoint *	joint;
	int					jointType;
	bool				alive;
	float				targetAngle;

	//----------------------------------------
	ofxBox2dRevoluteJoint();
	ofxBox2dRevoluteJoint(b2World* b2world, b2Body* body1, b2Body* body2, b2Vec2 anchor1, b2Vec2 anchor2, float lowerAngle, float upperAngle, bool bCollideConnected=true);
	
	//----------------------------------------
	void setWorld(b2World * w);
	void setup(b2World* b2world, b2Body* body1, b2Body* body2, b2Vec2 anchor1, b2Vec2 anchor3, float lowerAngle, float upperAngle, bool bCollideConnected=true);
	
	void setTargetAngle(float angle);
	void goToTargetAngle();

	//----------------------------------------
	bool isSetup();
	void draw();
	void destroy();
	
	
	//----------------------------------------
	ofVec2f getReactionForce(float inv_dt) const;
	b2Vec2  getReactionForceB2D(float inv_dt) const;
	float   getReactionTorque(float inv_dt) const;
};












