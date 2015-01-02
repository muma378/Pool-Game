/*-----------------------------------------------------------
  Simulation Header File
  -----------------------------------------------------------*/
#include <assert.h>
#include <cmath>
#include <string>
#include"vecmath.h"

/*-----------------------------------------------------------
  Macros
  -----------------------------------------------------------*/
#define TABLE_X			(0.6f) 
#define TABLE_Z			(1.2f)
#define BALL_RADIUS		(0.05f)
#define POCKET_RADIUS	(BALL_RADIUS*1.4)		//nonsense
#define CUSHION_THICK	(BALL_RADIUS*1.5)
#define BALL_MASS		(0.1f)
#define TWO_PI			(6.2832f)
#define	SIM_UPDATE_MS	(10)
#define NUM_BALLS		(7)		
#define NUM_CUSHION		(18)
#define NUM_POCKET		(6)
#define NUM_PLAYER		(2)
#define PI				(3.14159265f)
//the angle between vertical cushions and the cushions connected to the middle pockets
#define MID_ANGLE	(PI/4)		
//the angle between vertical or horizontal cushions and the cushions connected to the corner pockets
#define COR_ANGLE	(PI/3)
#define OFFSET(a)	(tan(a) * CUSHION_THICK)
#define CONNECTED_EDGES		(6)
#define	CONNECTED_POINTS	(4)


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

	cushion(){};
	void SetPosition(double start_x, double start_y, double end_x, double end_y);
	vec2 GetNormal(void);
	bool InRange(vec2 position);
};

/*-----------------------------------------------------------
  pocket class
  -----------------------------------------------------------*/

class pocket
{
private:
	vec2 position;
	double radius;

public:
	int newDroptBalls;
	bool punish;

	pocket():newDroptBalls(0), punish(false){
	};
	vec2 GetCenter(){return position;}
	double GetRadius(){return radius;}
	void SetPosition(vec2 vertex1, vec2 vertex2);
	void Reset();
};

/*----------------------------------------------
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
	bool	dropped;

	ball(): position(0.0), velocity(0.0), radius(BALL_RADIUS), 
		mass(BALL_MASS), dropped(false) {index = ballIndexCnt++; Reset();}
	
	void Reset(void);
	void ApplyImpulse(vec2 imp);
	void ApplyFrictionForce(int ms);
	void DoPlaneCollisions(cushion* c);
	void DoBallCollision(ball &b);
	void DoDropInPocket(pocket* p);
	void Update(int ms);
	
	bool HasHitPlane(cushion &c) const;
	bool HasHitBall(const ball &b) const;
	bool CenterOnPocket(pocket &p) const;

	void HitPlane(cushion &c);
	void HitBall(ball &b);
	void DropInPocket(pocket &p);
};

/*-----------------------------------------------------------
  table class
  -----------------------------------------------------------*/
class table
{
public:
	ball balls[NUM_BALLS];	
	cushion cushions[NUM_CUSHION];
	pocket pockets[NUM_POCKET];

	table();
	void Update(int ms);	
	bool AnyBallsMoving(void) const;
};


/*-----------------------------------------------------------
  player class
  -----------------------------------------------------------*/
class player
{
	static int playerIndexCnt;
public:
	std::string name;
	int scores;
	int index;

	player():scores(0){
		index = playerIndexCnt++;
		name = "Player" + char(index);
	}
	bool GetScores(pocket* p);

};

/*-----------------------------------------------------------
  global table
  -----------------------------------------------------------*/
extern table gTable;


/*-----------------------------------------------------------
  game class
  -----------------------------------------------------------*/
class game
{
public:
	player players[NUM_PLAYER];
	int totalScores;
	table& gameTable;
	int curPlayerNo;	//current playing player index
	bool checked;

	game():gameTable(gTable), totalScores(0), curPlayerNo(0), checked(true){
	
	}
	int NextPlayer();
	bool ReadyNextHit(){
		return !gameTable.AnyBallsMoving();
	}
	void Update(int ms);
	void PrintScores(void);
};

extern game gGame;

