/*-----------------------------------------------------------
  Simulation Source File
  -----------------------------------------------------------*/
#include"stdafx.h"
#include"simulation.h"

/*-----------------------------------------------------------
  macros
  -----------------------------------------------------------*/
#define SMALL_VELOCITY		(0.01f)

/*-----------------------------------------------------------
  globals
  -----------------------------------------------------------*/
vec2	gPlaneNormal_Left(1.0,0.0);
vec2	gPlaneNormal_Top(0.0,1.0);
vec2	gPlaneNormal_Right(-1.0,0.0);
vec2	gPlaneNormal_Bottom(0.0,-1.0);

table gTable;

static const float gRackPositionX[] = {0.0f,0.0f,(BALL_RADIUS*2.0f),(-BALL_RADIUS*2.0f),(BALL_RADIUS*4.0f)}; 
static const float gRackPositionZ[] = {0.5f,0.0f,(-BALL_RADIUS*3.0f),(-BALL_RADIUS*3.0f)}; 

float gCoeffRestitution = 0.5f;
float gCoeffFriction = 0.03f;
float gGravityAccn = 9.8f;
/*-----------------------------------------------------------
  ball class members
  -----------------------------------------------------------*/
int ball::ballIndexCnt = 0;

void ball::Reset(void)
{
	//set velocity to zero
	velocity = 0.0;

	//work out rack position
	if(index==0)
	{
		position(1) = 0.5;
		position(0) = 0.0;
		return;
	}
	
	static const float sep = (BALL_RADIUS*3.0f);
	static const float rowSep = (BALL_RADIUS*2.5f);
	int row = 1;
	int rowIndex = index;
	while(rowIndex > row)
	{
		rowIndex -= row;
		row++;
	}
	position(1) =  -(rowSep * (row-1));
	position(0) = (((row-1)*sep)/2.0f) - (sep*(row-rowIndex));
}

void ball::ApplyImpulse(vec2 imp)
{
	velocity = imp;
}

void ball::ApplyFrictionForce(int ms)
{
	if(velocity.Magnitude()<=0.0) return;

	//accelaration is opposite to direction of motion
	vec2 accelaration = -velocity.Normalised();
	//friction force = constant * mg
	//F=Ma, so accelaration = force/mass = constant*g
	accelaration *= (gCoeffFriction * gGravityAccn);
	//integrate velocity : find change in velocity
	vec2 velocityChange = ((accelaration * ms)/1000.0f);
	//cap magnitude of change in velocity to remove integration errors
	if(velocityChange.Magnitude() > velocity.Magnitude()) velocity = 0.0;
	else velocity += velocityChange;
}

void ball::DoPlaneCollisions(void)
{
	//test each plane for collision
	if(HasHitPlane1()) HitPlane1();
	if(HasHitPlane2()) HitPlane2();
	if(HasHitPlane3()) HitPlane3();
	if(HasHitPlane4()) HitPlane4();
}

void ball::DoBallCollision(ball &b)
{
	if(HasHitBall(b)) HitBall(b);
}

void ball::Update(int ms)
{
	//apply friction
	ApplyFrictionForce(ms);
	//integrate position
	position += ((velocity * ms)/1000.0f);
	//set small velocities to zero
	if(velocity.Magnitude()<SMALL_VELOCITY) velocity = 0.0;
}

bool ball::HasHitPlane1(void) const
{
	//if moving away from plane, cannot hit
	if(velocity(0) >= 0.0) return false;
	//if in front of plane, then have not hit
	if((position(0)-radius) > (-TABLE_X)) return false;
	return true;
}

bool ball::HasHitPlane2(void) const
{
	if(velocity(1) >= 0.0) return false;
	if((position(1)-radius) > (-TABLE_Z)) return false;
	return true;
}

bool ball::HasHitPlane3(void) const
{
	if(velocity(0) <= 0.0) return false;
	if((position(0)+radius) < (TABLE_X)) return false;
	return true;
}

bool ball::HasHitPlane4(void) const
{
	if(velocity(1) <= 0.0) return false;
	if((position(1)+radius) < (TABLE_Z)) return false;
	return true;
}

bool ball::HasHitBall(const ball &b) const
{
	//work out relative position of ball from other ball,
	//distance between balls
	//and relative velocity
	vec2 relPosn = position - b.position;
	float dist = (float) relPosn.Magnitude();
	vec2 relPosnNorm = relPosn.Normalised();
	vec2 relVelocity = velocity - b.velocity;

	//if moving apart, cannot have hit
	if(relVelocity.Dot(relPosnNorm) >= 0.0) return false;
	//if distnce is more than sum of radii, have not hit
	if(dist > (radius+b.radius)) return false;
	return true;
}

void ball::HitPlane1(void)
{
	//assume elastic collision
	//find plane normal
	vec2 planeNorm = gPlaneNormal_Left;
	//split velocity into 2 components:
	//find velocity component perpendicular to plane
	vec2 perp = planeNorm*(velocity.Dot(planeNorm));
	//find velocity component parallel to plane
	vec2 parallel = velocity - perp;
	//reverse perpendicular component
	//parallel component is unchanged
	velocity = parallel + (-perp)*gCoeffRestitution;
}

void ball::HitPlane2(void)
{
	vec2 planeNorm = gPlaneNormal_Top;
	vec2 perp = planeNorm*(velocity.Dot(planeNorm));
	vec2 parallel = velocity - perp;
	velocity = parallel + (-perp)*gCoeffRestitution;
}

void ball::HitPlane3(void)
{
	vec2 planeNorm = gPlaneNormal_Right;
	vec2 perp = planeNorm*(velocity.Dot(planeNorm));
	vec2 parallel = velocity - perp;
	velocity = parallel + (-perp)*gCoeffRestitution;
}

void ball::HitPlane4(void)
{
	vec2 planeNorm = gPlaneNormal_Bottom;
	vec2 perp = planeNorm*(velocity.Dot(planeNorm));
	vec2 parallel = velocity - perp;
	velocity = parallel + (-perp)*gCoeffRestitution;
}

void ball::HitBall(ball &b)
{
	//find direction from other ball to this ball
	vec2 relDir = (position - b.position).Normalised();

	//split velocities into 2 parts:  one component perpendicular, and one parallel to 
	//the collision plane, for both balls
	//(NB the collision plane is defined by the point of contact and the contact normal)
	float perpV = (float)velocity.Dot(relDir);
	float perpV2 = (float)b.velocity.Dot(relDir);
	vec2 parallelV = velocity-(relDir*perpV);
	vec2 parallelV2 = b.velocity-(relDir*perpV2);
	
	//Calculate new perpendicluar components:
	//v1 = (2*m2 / m1+m2)*u2 + ((m1 - m2)/(m1+m2))*u1;
	//v2 = (2*m1 / m1+m2)*u1 + ((m2 - m1)/(m1+m2))*u2;
	float sumMass = mass + b.mass;
	float perpVNew = (float)((perpV*(mass-b.mass))/sumMass) + (float)((perpV2*(2.0*b.mass))/sumMass);
	float perpVNew2 = (float)((perpV2*(b.mass-mass))/sumMass) + (float)((perpV*(2.0*mass))/sumMass);
	
	//find new velocities by adding unchanged parallel component to new perpendicluar component
	velocity = parallelV + (relDir*perpVNew);
	b.velocity = parallelV2 + (relDir*perpVNew2);
}

/*-----------------------------------------------------------
  table class members
  -----------------------------------------------------------*/
void table::Update(int ms)
{
	//check for collisions with planes, for all balls
	for(int i=0;i<NUM_BALLS;i++) balls[i].DoPlaneCollisions();
	
	//check for collisions between pairs of balls
	for(int i=0;i<NUM_BALLS;i++) 
	{
		for(int j=(i+1);j<NUM_BALLS;j++) 
		{
			balls[i].DoBallCollision(balls[j]);
		}
	}
	
	//update all balls
	for(int i=0;i<NUM_BALLS;i++) balls[i].Update(ms);
}

bool table::AnyBallsMoving(void) const
{
	//return true if any ball has a non-zero velocity
	for(int i=0;i<NUM_BALLS;i++) 
	{
		if(balls[i].velocity(0)!=0.0) return true;
		if(balls[i].velocity(1)!=0.0) return true;
	}
	return false;
}
