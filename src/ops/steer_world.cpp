/*
 * steer_world.cpp
 *
 *  Created on: 26 Jan, 2013
 *      Author: jaideep
 */

#include "steer_world.h"
#include "pedestrian.h"

bool OpenSteer::enableAnnotation = false;
bool OpenSteer::drawPhaseActive = false;

namespace ops
{

	steer_world::steer_world()
	{
		pd = new BruteForceProximityDatabase<AbstractVehicle*>();
	}

	steer_world::~steer_world()
	{
		delete pd;
	}

	void steer_world::addPedestrian(PedestrianConfig config)
	{
		population++;
		pedestrian* curPed = new pedestrian(*pd, this,config);
		crowd.push_back(curPed);
		return;
	}

	void steer_world::removePedestrian(void)
	{
		if (population > 0)
		{
			// save pointer to last pedestrian, then remove it from the crowd
			const pedestrian* ped = crowd.back();
			crowd.pop_back();
			population--;

			// delete the Pedestrian
			delete ped;
		}
	}

	void steer_world::addSphereObstacle(OpenSteer::Vec3 center, float radius)
	{
		SphereObstacle* curObstacle = new SphereObstacle;
		curObstacle->center = center;
		curObstacle->radius = radius;
		obstacles.push_back(curObstacle);
		return;
	}

	void steer_world::addPath(int pathPointCount, OpenSteer::Vec3 const pathPoints[], float radius,
			bool closeCycle)
	{
		OpenSteer::Vec3 endPoint0 = pathPoints[0];
		OpenSteer::Vec3 endPoint1 = pathPoints[pathPointCount - 1];
		OpenSteer::PolylineSegmentedPathwaySingleRadius* pathPtr =
				new OpenSteer::PolylineSegmentedPathwaySingleRadius(pathPointCount, pathPoints, radius,
																	closeCycle);
		Path newPath =
		{ pathPtr, endPoint0, endPoint1 };
		paths.push_back(newPath);
		return;
	}

	void steer_world::update(const float currentTime, const float elapsedTime)
	{
		// update each Pedestrian
		for (iterator i = crowd.begin(); i != crowd.end(); i++)
		{
			(**i).update(currentTime, elapsedTime);
		}
	}

	OpenSteer::Vec3 steer_world::getPedestrianPosition(int pedestrianIndex)
	{
		iterator i = crowd.begin() + pedestrianIndex;
		return (**i).position();
	}

	OpenSteer::Vec3 steer_world::getPedestrianNormal(int pedestrianIndex)
	{
		iterator i = crowd.begin() + pedestrianIndex;
		return (**i).forward();
	}

}
