/*
 * steer_world.h
 *
 *  Created on: 26 Jan, 2013
 *      Author: jaideep
 */

#ifndef STEER_WORLD_H_
#define STEER_WORLD_H_

#include <vector>
#include "OpenSteer/Proximity.h"
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/Vec3.h"
#include "OpenSteer/PolylineSegmentedPathwaySingleRadius.h"

namespace ops
{

	struct PedestrianConfig
	{
		OpenSteer::Vec3 pos;
		OpenSteer::Vec3 forward;
		float radius;
		int direction;
		bool scatterOnPath;
		float wander;
		int pathIndex;
	};

	struct Path
	{
		OpenSteer::PolylineSegmentedPathwaySingleRadius* pathPtr;
		OpenSteer::Vec3 endPoint0;
		OpenSteer::Vec3 endPoint1;
	};

	class pedestrian;
	typedef std::vector<pedestrian*> groupType;
	typedef OpenSteer::AbstractProximityDatabase<OpenSteer::AbstractVehicle*> ProximityDatabase;
// an STL vector of AbstractObstacle pointers and an iterator for it:
	typedef std::vector<OpenSteer::AbstractObstacle*> ObstacleGroup;
	typedef ObstacleGroup::const_iterator ObstacleIterator;

	class steer_world
	{
	public:
		steer_world();
		virtual ~steer_world();
		void addPedestrian(PedestrianConfig config);
		void removePedestrian(void);
		void addSphereObstacle(OpenSteer::Vec3 center, float radius);
		void addPath(int pathPointCount, OpenSteer::Vec3 const pathPoints[], float radius, bool closeCycle);
		void update(const float currentTime, const float elapsedTime);
		OpenSteer::Vec3 getPedestrianPosition(int curPedestrian);
		OpenSteer::Vec3 getPedestrianNormal(int curPedestrian);

		ObstacleGroup obstacles;
		std::vector<Path> paths;
//    OpenSteer::PolylineSegmentedPathwaySingleRadius* pathPtr;
//    OpenSteer::Vec3 gEndpoint0;
//    OpenSteer::Vec3 gEndpoint1;

	private:

		// type for a group of pedestrians

		groupType crowd;
		typedef groupType::const_iterator iterator;

		// pointer to database used to accelerate proximity queries
		ProximityDatabase* pd;

		// keep track of current flock size
		int population;

	};

}

#endif /* STEER_WORLD_H_ */
