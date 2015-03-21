#include <iomanip>
#include <sstream>
#include "OpenSteer/PolylineSegmentedPathwaySingleRadius.h"
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Proximity.h"
#include "OpenSteer/Color.h"

#include "pedestrian.h"

namespace ops
{
	using namespace OpenSteer;

	// ----------------------------------------------------------------------------
	typedef AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
	typedef AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;
	// ----------------------------------------------------------------------------

	// creates a path for the PlugIn
	PolylineSegmentedPathwaySingleRadius* getTestPath(void);
	PolylineSegmentedPathwaySingleRadius* gTestPath = NULL;
	SphereObstacle gObstacle1;
	SphereObstacle gObstacle2;
	ObstacleGroup gObstacles;
	Vec3 gEndpoint0;
	Vec3 gEndpoint1;
	bool gUseDirectedPathFollowing = false;
	// ------------------------------------ xxxcwr11-1-04 fixing steerToAvoid
	RectangleObstacle gObstacle3(7, 7);
	// ------------------------------------ xxxcwr11-1-04 fixing steerToAvoid

	// this was added for debugging tool, but I might as well leave it in
	bool gWanderSwitch = true;

	// constructor
	pedestrian::pedestrian(ProximityDatabase& pd, ops::steer_world* steer_world_ptr, PedestrianConfig config)
	{
		// allocate a token for this boid in the proximity database
		proximityToken = NULL;
		newPD(pd);

		mySteerWorldPtr = steer_world_ptr;
		// reset Pedestrian state
		reset(config);

		if (!config.scatterOnPath)
		{
			setPosition(config.pos);
			setForward(config.forward);
		}
		setRadius(config.radius);
		pathDirection = config.direction;
		wander = config.wander;

	}

	// destructor
	pedestrian::~pedestrian()
	{
		// delete this boid's token in the proximity database
		delete proximityToken;
	}

	// reset all instance state
	void pedestrian::reset(PedestrianConfig config)
	{
		// reset the vehicle
		SimpleVehicle::reset();

		// max speed and max steering force (maneuverability)
		setMaxSpeed(2.0);
		setMaxForce(8.0);

		// initially stopped
		setSpeed(0);

		// size of bounding sphere, for obstacle avoidance, etc.
		setRadius(0.5); // width = 0.7, add 0.3 margin, take half

		// pick a random path from paths
//		int numofPaths = mySteerWorldPtr->paths.size();
//		pathIndex = floor(frandom2(0, numofPaths));
		pathIndex = config.pathIndex;
//		pathIndex = 0;

		// set the path for this Pedestrian to follow
//		path = getTestPath ();
		path = mySteerWorldPtr->paths[pathIndex].pathPtr;

		// set initial position
		// (random point on path + random horizontal offset)
		const float d = path->length() * frandom01();
		const float r = path->radius();
		const Vec3 randomOffset = randomVectorOnUnitRadiusXZDisk() * r;
		setPosition(path->mapPathDistanceToPoint(d) + randomOffset);

		// randomize 2D heading
		randomizeHeadingOnXZPlane();

		// pick a random direction for path following (upstream or downstream)
		pathDirection = (frandom01() > 0.5) ? -1 : +1;

		// trail parameters: 3 seconds with 60 points along the trail
		setTrailParameters(3, 60);

		// notify proximity database that our position has changed
		proximityToken->updateForNewPosition(position());
	}

	// per frame simulation update
	void pedestrian::update(const float currentTime, const float elapsedTime)
	{
		// apply steering force to our momentum
		applySteeringForce(determineCombinedSteering(elapsedTime), elapsedTime);

		// reverse direction when we reach an endpoint
		if (gUseDirectedPathFollowing)
		{
			const Color darkRed(0.7f, 0, 0);
			float const pathRadius = path->radius();

			gEndpoint0 = mySteerWorldPtr->paths[pathIndex].endPoint0;
			gEndpoint1 = mySteerWorldPtr->paths[pathIndex].endPoint1;

			if (Vec3::distance(position(), gEndpoint0) < pathRadius)
			{
				pathDirection = +1;
				annotationXZCircle(pathRadius, gEndpoint0, darkRed, 20);
			}
			if (Vec3::distance(position(), gEndpoint1) < pathRadius)
			{
				pathDirection = -1;
				annotationXZCircle(pathRadius, gEndpoint1, darkRed, 20);
			}
		}

		// annotation
		annotationVelocityAcceleration(5, 0);
		recordTrailVertex(currentTime, position());

		// notify proximity database that our position has changed
		proximityToken->updateForNewPosition(position());
	}

	// compute combined steering force: move forward, avoid obstacles
	// or neighbors if needed, otherwise follow the path and wander
	Vec3 pedestrian::determineCombinedSteering(const float elapsedTime)
	{
		// move forward
		Vec3 steeringForce = forward();

		// probability that a lower priority behavior will be given a
		// chance to "drive" even if a higher priority behavior might
		// otherwise be triggered.
		const float leakThrough = 0.1f;

		// determine if obstacle avoidance is required
		Vec3 obstacleAvoidance;
		if (leakThrough < frandom01())
		{
			const float oTime = 6; // minTimeToCollision = 6 seconds
			// ------------------------------------ xxxcwr11-1-04 fixing steerToAvoid
			// just for testing
//			obstacleAvoidance = steerToAvoidObstacles (oTime, gObstacles);
			obstacleAvoidance = steerToAvoidObstacles(oTime, mySteerWorldPtr->obstacles);
			// ------------------------------------ xxxcwr11-1-04 fixing steerToAvoid
		}

		// if obstacle avoidance is needed, do it
		if (obstacleAvoidance != Vec3::zero)
		{
			steeringForce += obstacleAvoidance;
		}
		else
		{
			// otherwise consider avoiding collisions with others
			Vec3 collisionAvoidance;
			const float caLeadTime = 3;

			// find all neighbors within maxRadius using proximity database
			// (radius is largest distance between vehicles traveling head-on
			// where a collision is possible within caLeadTime seconds.)
			const float maxRadius = caLeadTime * maxSpeed() * 2;
			neighbors.clear();
			proximityToken->findNeighbors(position(), maxRadius, neighbors);

			if (leakThrough < frandom01())
				collisionAvoidance = steerToAvoidNeighbors(caLeadTime, neighbors) * 10;

			// if collision avoidance is needed, do it
			if (collisionAvoidance != Vec3::zero)
			{
				steeringForce += collisionAvoidance;
			}
			else
			{
				// add in wander component (according to user switch)
				if (gWanderSwitch)
					steeringForce += steerForWander(elapsedTime) * wander;

				// do (interactively) selected type of path following
				const float pfLeadTime = 3;
				const Vec3 pathFollow = (
						gUseDirectedPathFollowing ?
								steerToFollowPath(pathDirection, pfLeadTime, *path) :
								steerToStayOnPath(pfLeadTime, *path));

				// add in to steeringForce
				steeringForce += pathFollow * 0.5;
			}
		}

		// return steering constrained to global XZ "ground" plane
		return steeringForce.setYtoZero();
	}

	// switch to new proximity database -- just for demo purposes
	void pedestrian::newPD(ProximityDatabase& pd)
	{
		// delete this boid's token in the old proximity database
		delete proximityToken;

		// allocate a token for this boid in the proximity database
		proximityToken = pd.allocateToken(this);
	}

	AVGroup pedestrian::neighbors;

	PolylineSegmentedPathwaySingleRadius* getTestPath(void)
	{
		if (gTestPath == NULL)
		{
			const float pathRadius = 2;

			const PolylineSegmentedPathwaySingleRadius::size_type pathPointCount = 7;
			const float size = 30;
			const float top = 2 * size;
			const float gap = 1.2f * size;
			const float out = 2 * size;
			const float h = 0.5;
			const Vec3 pathPoints[pathPointCount] =
			{ Vec3(h + gap - out, 0, h + top - out),  // 0 a
				Vec3(h + gap, 0, h + top),      // 1 b
				Vec3(h + gap + (top / 2), 0, h + top / 2),    // 2 c
				Vec3(h + gap, 0, h),          // 3 d
				Vec3(h, 0, h),          // 4 e
				Vec3(h, 0, h + top),      // 5 f
				Vec3(h + gap, 0, h + top / 2) };   // 6 g

			gObstacle1.center = interpolate(0.2f, pathPoints[0], pathPoints[1]);
			gObstacle2.center = interpolate(0.5f, pathPoints[2], pathPoints[3]);
			gObstacle1.radius = 3;
			gObstacle2.radius = 5;
//            gObstacles.push_back (&gObstacle1);
//            gObstacles.push_back (&gObstacle2);
			// ------------------------------------ xxxcwr11-1-04 fixing steerToAvoid

//            gObstacles.push_back (&gObstacle3);

			//         // rotated to be perpendicular with path
			//         gObstacle3.setForward (1, 0, 0);
			//         gObstacle3.setSide (0, 0, 1);
			//         gObstacle3.setPosition (20, 0, h);

			//         // moved up to test off-center
			//         gObstacle3.setForward (1, 0, 0);
			//         gObstacle3.setSide (0, 0, 1);
			//         gObstacle3.setPosition (20, 3, h);

			//         // rotated 90 degrees around path to test other local axis
			//         gObstacle3.setForward (1, 0, 0);
			//         gObstacle3.setSide (0, -1, 0);
			//         gObstacle3.setUp (0, 0, -1);
			//         gObstacle3.setPosition (20, 0, h);

			// tilted 45 degrees
			gObstacle3.setForward(Vec3(1, 1, 0).normalize());
			gObstacle3.setSide(0, 0, 1);
			gObstacle3.setUp(Vec3(-1, 1, 0).normalize());
			gObstacle3.setPosition(20, 0, h);

			//         gObstacle3.setSeenFrom (Obstacle::outside);
			//         gObstacle3.setSeenFrom (Obstacle::inside);
			gObstacle3.setSeenFrom(Obstacle::both);

			// ------------------------------------ xxxcwr11-1-04 fixing steerToAvoid

			gEndpoint0 = pathPoints[0];
			gEndpoint1 = pathPoints[pathPointCount - 1];

			gTestPath = new PolylineSegmentedPathwaySingleRadius(pathPointCount, pathPoints, pathRadius,
																	false);
		}
		return gTestPath;
	}
}
