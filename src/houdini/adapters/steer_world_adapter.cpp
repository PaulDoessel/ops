/*
 * steer_world_adapter.cpp
 *
 *  Created on: 26 Jan, 2013
 *      Author: jaideep
 */

#include "steer_world_adapter.h"
#include "../utils/attrib_utility.h"
#include "../../ops/steer_world.h"

#include "GU/GU_Detail.h"
#include "GEO/GEO_Point.h"
#include "GEO/GEO_Vertex.h"
#include "GEO/GEO_Primitive.h"

#include "OpenSteer/Vec3.h"

namespace houdini_ops
{

	steer_world_adapter::steer_world_adapter()
	{
		mySteerWorldPtr = new ops::steer_world;
	}

	steer_world_adapter::~steer_world_adapter()
	{
		delete mySteerWorldPtr;
	}
	void steer_world_adapter::addPedestrian(const GU_Detail* gdp, const GEO_Point* ppt)
	{
		UT_Vector3 pos1 = ppt->getPos3();
		OpenSteer::Vec3 pos2(pos1.x(), pos1.y(), pos1.z());

		GA_ROAttributeRef attRef_type = gdp->findNumericTuple(GA_ATTRIB_POINT, "N", 3, 3);
		UT_Vector3 normal1;
		if (attRef_type.isValid())
			ppt->get(attRef_type, normal1.data(), 3);
		OpenSteer::Vec3 normal2(normal1.x(), normal1.y(), normal1.z());

		GA_ROAttributeRef attRef_scatter = gdp->findAttribute(GA_ATTRIB_POINT, "scatterOnPath");
		int scatterOnPath = 1;
		if (attRef_scatter.isValid())
			ppt->get(attRef_scatter, scatterOnPath, 0);

		GA_ROAttributeRef attRef_radius = gdp->findAttribute(GA_ATTRIB_POINT, "radius");
		float radius = 0.5;
		if (attRef_radius.isValid())
			ppt->get(attRef_radius, radius, 0);

		GA_ROAttributeRef attRef_direction = gdp->findAttribute(GA_ATTRIB_POINT, "direction");
		int direction = 1;
		if (attRef_direction.isValid())
			ppt->get(attRef_direction, direction, 0);

		GA_ROAttributeRef attRef_wander = gdp->findAttribute(GA_ATTRIB_POINT, "wander");
		float wander = 1;
		if (attRef_wander.isValid())
			ppt->get(attRef_wander, wander, 0);

		GA_ROAttributeRef attRef_pathIndex = gdp->findAttribute(GA_ATTRIB_POINT, "pathIndex");
		int pathIndex = 0;
		if (attRef_pathIndex.isValid())
			ppt->get(attRef_pathIndex, pathIndex, 0);

		ops::PedestrianConfig config =
		{ pos2, normal2, radius, direction, scatterOnPath,wander,pathIndex };
		mySteerWorldPtr->addPedestrian(config);

		return;
	}

	void steer_world_adapter::addObstacle(const GU_Detail* obstacleGdp, int curPtNum)
	{
		const GEO_Point* ppt = obstacleGdp->points()[curPtNum];
		const UT_Vector4T<float> pos = ppt->getPos();
		OpenSteer::Vec3 pos_vec3(pos.x(), pos.y(), pos.z());

		GA_ROAttributeRef sizeRef = obstacleGdp->findNumericTuple(GA_ATTRIB_POINT, "size", 1, 1);
		const GA_AIFTuple* sizeRefTuple = sizeRef.getAIFTuple();
		float val = 5;
		GA_Offset offset = ppt->getMapOffset();
		if (sizeRef.isValid())
			sizeRefTuple->get(sizeRef.getAttribute(), offset, &val, 1);

		mySteerWorldPtr->addSphereObstacle(pos_vec3, val);
		return;
	}

	void steer_world_adapter::addPath(const GU_Detail* pathGdp, int curPrimNum)
	{
		const GEO_Primitive* prim = pathGdp->primitives()[curPrimNum];
		int pathPointCount = prim->getVertexCount();
		OpenSteer::Vec3* pathPoints;
		pathPoints = new OpenSteer::Vec3[pathPointCount];

		for (int i = 0; i < pathPointCount; i++)
		{
			const GEO_Vertex vtx = prim->getVertex(i);
			UT_Vector4T<float> pos = vtx.getPos();
			OpenSteer::Vec3 pos_vec3(pos.x(), pos.y(), pos.z());
			pathPoints[i] = pos_vec3;
		}

		GA_ROAttributeRef sizeRef = pathGdp->findNumericTuple(GA_ATTRIB_PRIMITIVE, "size", 1, 1);
		float val = 2;
		if (sizeRef.isValid())
			prim->get(sizeRef, val, 0);

		GA_ROAttributeRef closedRef = pathGdp->findNumericTuple(GA_ATTRIB_PRIMITIVE, "closed", 1, 1);
		float closed = 0;
		if (closedRef.isValid())
			prim->get(closedRef, closed, 0);

		mySteerWorldPtr->addPath(pathPointCount, pathPoints, val, closed);
		return;
	}

	void steer_world_adapter::updateSteerWorld(const float currentTime, const float elapsedTime)
	{
		mySteerWorldPtr->update(currentTime, elapsedTime);
		return;
	}

	void steer_world_adapter::updatePoint(GU_Detail* gdp, int curPtNum)
	{
		OpenSteer::Vec3 posVec3 = mySteerWorldPtr->getPedestrianPosition(curPtNum);
		GEO_Point* ppt;
		ppt = gdp->points()[curPtNum];
		ppt->setPos(posVec3.x, posVec3.y, posVec3.z, 1);

		//--------------------START--------------------Update Normals
		OpenSteer::Vec3 normalVec3 = mySteerWorldPtr->getPedestrianNormal(curPtNum);
		attrib_utility attrib_utility_obj;
		attrib_utility_obj.setNormal(gdp, curPtNum, UT_Vector3(normalVec3.x, normalVec3.y, normalVec3.z));

		//----------------------END----------------------Update Normals

		return;
	}

}
