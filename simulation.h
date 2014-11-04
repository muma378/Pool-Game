/*-----------------------------------------------------------
  Simulation Header File
  -----------------------------------------------------------*/
#include <assert.h>
#include"vecmath.h"
#include <time.h>
#include <stdlib.h>

/*-----------------------------------------------------------
  Macros
  -----------------------------------------------------------*/
#define TABLE_X			(0.6f) 
#define TABLE_Z			(1.2f)
#define BALL_RADIUS		(0.05f)
#define BALL_MASS		(0.1f)
#define TWO_PI			(6.2832f)
#define	SIM_UPDATE_MS	(10)
#define NUM_BALLS		(7)		
#define NUM_CUSHION		(4)
#define MAX_PARTICLES	(10)
#define MIN_PARTICLES	(2)
#define MAX_SPEED		(5)
#define PARTICLE_SET_SCALE	(2)
#define PARTICLE_RADIUS	(0.001f)

/*-----------------------------------------------------------
  plane normals
  -----------------------------------------------------------*/
extern vec2	gPlaneNormal_Left;
extern vec2	gPlaneNormal_Top;
extern vec2	gPlaneNormal_Right;
extern vec2	gPlaneNormal_Bottom;

/*-----------------------------------------------------------
  cushion class
  -----------------------------------------------------------*/
class cushion
{
public:
	vec2 start;
	vec2 end;
	vec2 normal; 

	void SetPosition(double start_x, double start_y, double end_x, double end_y);
	vec2 GetNormal(void);
};

/*----------------------------------------------------------
  particle class
 ----------------------------------------------------------*/

class particle
{
	static int particleIndexCnt;
private:
	int index;
	vec3 velocity;
	 
public:
	vec3 position;
	float radius;
	bool visible;

	particle():radius(PARTICLE_RADIUS), visible(true){
		index = particleIndexCnt++;
	};

	static int random_speed(){
		return rand() % MAX_SPEED;
	}
	void Disappear(){ visible = false; }
	void Reset(const vec2);
	void ApplyGravity(int ms);
	void Update(int ms);
	bool HaveCollision();

};


class particleSet
{	
private:
	bool visible;
	int size;
	int particle_index;
	int invisible_num;

public:
	particle *particles;

	particleSet():visible(true),size(0), invisible_num(0){};
	void Initial(vec2 start_pos);
	
	void ParticleIteratorBegin(){ particle_index = 0; }
	bool HasNextParticle();
	particle GetNextParticle();

	int GetSize(){ return size;}
	bool AllInvisible(){ return !visible; }
};


/*-----------------------------------------------------------
  ball class
  -----------------------------------------------------------*/

class ball
{
	static int ballIndexCnt;
private:
	static int particle_set_num;
	static int particle_set_index;

public:
	vec2	position;
	vec2	velocity;
	float	radius;
	float	mass;
	int		index;
	
	static particleSet *particle_sets;

	ball(): position(0.0), velocity(0.0), radius(BALL_RADIUS), 
		mass(BALL_MASS){index = ballIndexCnt++; Reset();}
	
	void Reset(void);
	void ApplyImpulse(vec2 imp);
	void ApplyFrictionForce(int ms);
	void DoPlaneCollisions(cushion* c);
	void DoBallCollision(ball &b);
	void Update(int ms);
	
	bool HasHitPlane(cushion &c) const;
	bool HasHitBall(const ball &b) const;

	void HitPlane(cushion &c);
	void HitBall(ball &b);

	vec2 CollisionPos(const ball &b) const;
	vec2 CollisionPos(const cushion &c) const;
	void Firework(vec2 position);
	static int GetParticleSetSize(){ return particle_set_index; }
	void ParticleSetBegin();
	bool HasNextParticleSet();
	particleSet GetNextParticleSet();
};


/*-----------------------------------------------------------
  table class
  -----------------------------------------------------------*/
class table
{
public:
	ball balls[NUM_BALLS];	
	cushion cushions[NUM_CUSHION];

	table(){	
		cushions[0].SetPosition(TABLE_X, TABLE_Z, -TABLE_X, TABLE_Z);
		cushions[1].SetPosition(-TABLE_X, TABLE_Z, -TABLE_X, -TABLE_Z);
		cushions[2].SetPosition(-TABLE_X, -TABLE_Z, TABLE_X, -TABLE_Z);
		cushions[3].SetPosition(TABLE_X, -TABLE_Z, TABLE_X, TABLE_Z);
	}
	
	void Update(int ms);	
	bool AnyBallsMoving(void) const;
};


/*-----------------------------------------------------------
  global table
  -----------------------------------------------------------*/
extern table gTable;
