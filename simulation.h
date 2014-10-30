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
#define MIN_PARTICLES	(20)
#define MAX_SPEED		(50)

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
		mass(BALL_MASS) {index = ballIndexCnt++; Reset();}
	
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

/*----------------------------------------------------------
  particle class
 ----------------------------------------------------------*/

class particle
{
	static int particleIndexCnt;
private:
	float radius;
	vec3 velocity;
	vec3 position;
	int index;
	bool visible;

public:
	particle():radius(BALL_RADIUS), visible(true){
		index = particleIndexCnt++;
	};

	static int random_speed(){
		return rand() % MAX_SPEED;
	}
	void Reset(const vec2);
	
};


class particleSet
{
private:
	particle *particle_set;

public:
	particleSet(vec2 start_pos){
		srand(time(NULL));
		int particle_num = rand()%(MAX_PARTICLES - MIN_PARTICLES) + MIN_PARTICLES;
		particle_set = new particle[particle_num];
		for(int i=0;i<particle_num;i++){
			particle_set[i].Reset(start_pos);
		}
	}

	~particleSet(){
		delete[] particle_set;
	};

};

/*-----------------------------------------------------------
  global table
  -----------------------------------------------------------*/
extern table gTable;
