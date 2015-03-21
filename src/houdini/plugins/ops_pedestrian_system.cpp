#include <UT/UT_DSOVersion.h>
#include <UT/UT_Math.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Vector3.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <CH/CH_LocalVariable.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include "ops_pedestrian_system.h"

#include "../adapters/steer_world_adapter.h"
#include <boost/shared_ptr.hpp>

#include <map>
#include <vector>
#include <limits.h>

OP_Node *
myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
	return new ops_pedestrian_system(net, name, op);
}

PRM_Template myTemplateList[] =
{ PRM_Template() };

// Here's how we define local variables for the SOP.
enum
{
	VAR_PT,		// Point number of the star
	VAR_NPT		// Number of points in the star
};

CH_LocalVariable myVariables[] =
{
{ "PT", VAR_PT, 0 },		// The table provides a mapping
	{ "NPT", VAR_NPT, 0 },		// from text string to integer token
	{ 0, 0, 0 }, };

void newSopOperator(OP_OperatorTable *table)
{
	OP_Operator* myNewOpDef = new OP_Operator("ops_pedestrian_system",			// Internal name
			"ops_pedestrian_system",			// UI name
			myConstructor,	// How to build the SOP
			myTemplateList,	// My parameters
			0,				// Min # of sources
			3,				// Max # of sources
			myVariables,	// Local variables
			OP_FLAG_GENERATOR);		// Flag it as generator

	table->addOperator(myNewOpDef);
}

float ops_pedestrian_system::getVariableValue(int index, int thread)
{
	// Note that "gdp" may be null here, so we do the safe thing
	// and cache values we are interested in.
	if (myCurrPoint < 0)
		return 0;
	switch (index)
	{
	case VAR_PT:
		return myCurrPoint;
	case VAR_NPT:
		return myTotalPoints;
	}
	// Not one of our variables, must delegate to the base class.
	return SOP_Node::getVariableValue(index, thread);
}

ops_pedestrian_system::ops_pedestrian_system(OP_Network *net, const char *name, OP_Operator *op) :
		SOP_Node(net, name, op)
{
	myCurrPoint = -1;	// To prevent garbage values from being returned
}

ops_pedestrian_system::~ops_pedestrian_system()
{
}

unsigned ops_pedestrian_system::disableParms()
{
	return 0;
}

void ops_pedestrian_system::initializeOpenSteer(const GU_Detail* gdp, const GU_Detail* obstacleGdp,
		const GU_Detail* pathGdp)
{
	mySteerWorldAdapterPtr = new houdini_ops::steer_world_adapter;

	//--------------------START--------------------Path Setup
	if (NULL != pathGdp)
	{
		const GEO_Primitive* prim;
		int curPrimNum = 0;

//		FOR_ALL_PRIMITIVES(pathGdp, prim)
		GA_FOR_ALL_PRIMITIVES(pathGdp, prim)
		{
			mySteerWorldAdapterPtr->addPath(pathGdp, curPrimNum);
			curPrimNum++;
		}
	}
	//----------------------END----------------------Path Setup

	//--------------------START--------------------Obstacle Setup
	if (NULL != obstacleGdp)
	{
		const GEO_Point* ppt;
		int curPtNum = 0;
//		FOR_ALL_GPOINTS(obstacleGdp, ppt)
		GA_FOR_ALL_GPOINTS(obstacleGdp, ppt)
		{
			mySteerWorldAdapterPtr->addObstacle(obstacleGdp, curPtNum);
			curPtNum++;
		}
	}
	//----------------------END----------------------Obstacle Setup

	//--------------------START--------------------Pedestrian Setup
	{
		const GEO_Point* ppt;
		int curPtNum = 0;
//		FOR_ALL_GPOINTS(gdp, ppt)
		GA_FOR_ALL_GPOINTS(gdp, ppt)
		{
			mySteerWorldAdapterPtr->addPedestrian(gdp, ppt);
			curPtNum++;
		}
	}
	//----------------------END----------------------Pedestrian Setup

	return;
}

void ops_pedestrian_system::updateOpenSteer(const OP_Context &context)
{
	mySteerWorldAdapterPtr->updateSteerWorld(context.getTime(), 1.0 / 24.0);
	return;
}

void ops_pedestrian_system::updateGeometry(GU_Detail* gdp)
{
	GEO_Point* ppt;
	int curPtNum = 0;
//	FOR_ALL_GPOINTS(gdp, ppt)
	GA_FOR_ALL_GPOINTS(gdp, ppt)
	{
		mySteerWorldAdapterPtr->updatePoint(gdp, curPtNum);
		curPtNum++;
	}

	return;
}

OP_ERROR ops_pedestrian_system::cookMySop(OP_Context &context)
{
	OP_Node::flags().timeDep = 1;
	if (lockInputs(context) >= UT_ERROR_ABORT)
		return error();

	//--------------------START--------------------Input Geo Setup
	enum InputIndex
	{
		pedestrianIndex, obstacleIndex, pathIndex
	};
	duplicateSource(pedestrianIndex, context);
	const GU_Detail* obstacleGdp = inputGeo(obstacleIndex);
	const GU_Detail* pathGdp = inputGeo(pathIndex);
	//----------------------END----------------------Input Geo Setup

	if (1 == context.getFrame())
	{
		initializeOpenSteer(gdp, obstacleGdp, pathGdp);
	}
	else
	{
		updateOpenSteer(context);
	}

	updateGeometry(gdp);

	unlockInputs();
	return error();
}

const char *
ops_pedestrian_system::inputLabel(unsigned index) const
{
	switch (index)
	{
	case (0):
		return "pedestrians";
		break;
	case (1):
		return "obstacles";
		break;
	case (2):
		return "path";
		break;
	default:
		return "unknown input index";
	}
}

