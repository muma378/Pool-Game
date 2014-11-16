/*-----------------------------------------------------------
  Simulation Source File
  -----------------------------------------------------------*/
#include"stdafx.h"
#include"simulation.h"
#include <iostream>
using namespace std;
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

void ball::DoPlaneCollisions(cushion* c)
{
	//test each plane for collision
	for(int i=0;i<NUM_CUSHION;i++){
		if(HasHitPlane(*(c+i))){ 
			HitPlane(*(c+i));
			particleSetMgr* psm = particleSetMgr::Instance();
			psm->Firework(this->CollisionPos(*(c+i)));
		}
	}
}

void ball::DoBallCollision(ball &b)
{
	if(HasHitBall(b)){
		HitBall(b);
		particleSetMgr* psm = particleSetMgr::Instance();
		psm->Firework(this->CollisionPos(b));
	}
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

bool ball::HasHitPlane(cushion &c) const
{
	//if moving away from plane, cannot hit
	if(velocity.Dot(c.normal) >= 0.0 ) return false;
	//if in front of plane, then have not hit
	if((position-(c.end)).Dot(c.normal) > radius) return false;
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

void ball::HitPlane(cushion &c)
{
	//assume elastic collision
	//find plane normal
	vec2 planeNorm = c.normal;
	//split velocity into 2 components:
	//find velocity component perpendicular to plane
	vec2 perp = planeNorm*(velocity.Dot(planeNorm));
	//find velocity component parallel to plane
	vec2 parallel = velocity - perp;
	//reverse perpendicular component
	//parallel component is unchanged
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

vec2 ball::CollisionPos(const ball &b) const
{
	return position.MiddlePlace(b.position);
}

vec2 ball::CollisionPos(const cushion &c) const
{	
	//the vector of cushion, (P2-P1)
	vec2 plane = c.end - c.start;
	//the vector of sphere center to cushion's end, (P2-P3)
	vec2 ball_to_end = c.end - position;
	//the rate of projected point to cushion's end over cushion'start to end
	//LET k = |P2-P0|/|P2-P1|
	//k = (P2-P3)*(P2-P1)/|P2-P1|
	double k = ball_to_end.Dot(plane.Normalised())/plane.Magnitude();
	//P0 = P2 - (P2-P1)*k
	return c.end - plane*k;
	
	//return position + c.normal * radius;

}


/*-----------------------------------------------------------
  table class members
  -----------------------------------------------------------*/
void table::Update(int ms)
{
	//check for collisions with planes, for all balls
	for(int i=0;i<NUM_BALLS;i++) balls[i].DoPlaneCollisions(cushions);
	
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


/*-----------------------------------------------------------
  cushion class members
  -----------------------------------------------------------*/
vec2 cushion::GetNormal(void)
{
	vec2 line = start - end;
	return vec2(line.elem[1], -line.elem[0]);
}

void cushion::SetPosition(double start_x, double start_y, double end_x, double end_y)
{	
	vec2 s(start_x, start_y);
	vec2 e(end_x, end_y);
	assert( s != e );
	start = s, end = e;
	normal = GetNormal().Normalised();
}

/*-----------------------------------------------------------
  particle class members
  -----------------------------------------------------------*/

void particle::Reset(const vec2 start_pos){
		position = vec3(start_pos(0), BALL_RADIUS/2.0 ,start_pos(1));
		velocity = vec3(((rand() % 200)-100)/200.0, 2.0*((rand() % 100)/100.0), ((rand() % 200)-100)/200.0);
};

void particle::ApplyGravity(int ms){
	//vec3 velocityChange(0.0f, -(gGravityAccn * ms)/1000.0f, 0.0f);
	vec3 velocityChange(0.0f, -(4 * ms)/1000.0f, 0.0f);
	velocity += velocityChange;
}

bool particle::HaveCollision(){
	if(position(1)<0) return true;	//touch the ground
	//TODO:Add more disappear condition
	return false;
}

void particle::Update(int ms){
	position += ((velocity * ms)/1000.0f);
	ApplyGravity(ms);
	if(HaveCollision()) Disappear();
}


void particleSet::Initial(vec2 start_pos){
		srand(time(NULL));
		size = rand()%(MAX_PARTICLES - MIN_PARTICLES) + MIN_PARTICLES;
		cout << size << " particles are allocated." << endl;  
 		particles = new particle[size];
		for(int i=0;i<size;i++){
			particles[i].Reset(start_pos);
		}
	}

bool particleSet::HasNextParticle()
{
	if(!visible) return false;
	while(particle_index<size && !particles[particle_index].visible){
		particle_index++;
		invisible_num++;
	}
	if(invisible_num==size){
		delete [] particles;
		visible = false;
		return false;
	}
	if(particle_index>=size) return false;
	
	return true;
}

particle* particleSet::GetNextParticle(){

	return particles + particle_index++;
};


particleSetMgr* particleSetMgr::_instance = 0;


particleSetMgr* particleSetMgr::Instance(){
	if(_instance==0){
		_instance = new particleSetMgr; 
	}
	return _instance;
}

void particleSetMgr::Firework(vec2 position)
{	
	if( particle_set_size>=particle_set_num ){
		particle_set_num *= PARTICLE_SET_SCALE;
		particleSet *temp_particles = new particleSet[particle_set_num];
		cout << "Newed:" << particle_set_num <<endl;
		memcpy(temp_particles, particle_sets, sizeof(particleSet)*particle_set_size);
		delete [] particle_sets;
		particle_sets = temp_particles;
	};
	particle_sets[particle_set_size++].Initial(position);
	cout << "Firework happend " << particle_set_size <<endl;
	
}

void particleSetMgr::Update(int ms)
{	
	if(particle_set_size > 0){
		particleSet* ps;
		particle* p;
		for(ParticleSetBegin();HasNextParticleSet();){
			ps = GetNextParticleSet();
			for(ps->ParticleIteratorBegin();ps->HasNextParticle();){
				p = ps->GetNextParticle();
				p->Update(ms);	
			}
		}
	}
}

void particleSetMgr::ParticleSetBegin()
{
	index = 0;
	invisible_num = 0;
}


void particleSetMgr::ResetVisible(){
	for(int i=0;i<particle_set_size;i++)
		particle_sets[i].SetVisible();
}

bool particleSetMgr::HasNextParticleSet()
{
	while(index<particle_set_size && particle_sets[index].AllInvisible()){
		index++;
		invisible_num++;
	}
	if(invisible_num!=0 && invisible_num==particle_set_size){		
		ResetVisible();
		particle_set_size = 0;	//no need to delete, just reset the index	
		return false;
	}
	if(index>=particle_set_size) return false;
	 
	return true;
}

particleSet* particleSetMgr::GetNextParticleSet(){
	return particle_sets + index++;
};
