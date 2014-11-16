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
#define MAX_PARTICLES	(100)
#define MIN_PARTICLES	(10)
#define MAX_SPEED		(200)
#define PARTICLE_SET_SCALE	(2)
#define PARTICLE_RADIUS	(0.002f)

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
private:
	vec3 velocity;
	 
public:
	vec3 position;
	float radius;
	bool visible;

	particle():radius(PARTICLE_RADIUS), visible(true){};

	void Disappear(){ 
		visible = false;
	}
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

	particleSet():visible(true),size(0){};
	void Initial(vec2 start_pos);
	
	void ParticleIteratorBegin(){ particle_index = 0; invisible_num = 0; }
	bool HasNextParticle();
	particle* GetNextParticle();

	int GetSize(){ return size;}
	bool AllInvisible(){ return !visible; }
	void SetVisible(){ this->visible = true; }
};


/*-----------------------------------------------------------
  ball class
  -----------------------------------------------------------*/

class ball
{
	static int ballIndexCnt;
public:
	vec2	position;
	vec2	velocity;
	float	radius;
	float	mass;
	int		index;
	
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

//this class is used to manager the multiple particles' set
//this should be a singleton
class particleSetMgr
{
private:
	int particle_set_num;	//the number of slots
	int particle_set_size;	//the size
	static particleSetMgr* _instance;
	int index;
	int invisible_num;

public:
	particleSet *particle_sets;

	particleSetMgr():particle_set_num(PARTICLE_SET_SCALE), particle_set_size(0){
	particle_sets = new particleSet[particle_set_num];
}
	~particleSetMgr(){ delete [] particle_sets; }
	static particleSetMgr* Instance();
	void Update(int ms);
	void Firework(vec2 position);
	void ParticleSetBegin();
	bool HasNextParticleSet();
	void ResetVisible();
	particleSet* GetNextParticleSet();

};



/*-----------------------------------------------------------
  global table
  -----------------------------------------------------------*/
extern table gTable;
//extern particleSetMgr gParticleSetMgr;

