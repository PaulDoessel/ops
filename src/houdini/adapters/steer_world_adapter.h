/*
 * steer_world_adapter.h
 *
 *  Created on: 26 Jan, 2013
 *      Author: jaideep
 */

#ifndef STEER_WORLD_ADAPTER_H_
#define STEER_WORLD_ADAPTER_H_

#include "../../ops/steer_world.h"

class GU_Detail;
class GEO_Point;
namespace houdini_ops
{
	class steer_world_adapter
	{
	public:
		steer_world_adapter();
		virtual ~steer_world_adapter();
		void addPedestrian(const GU_Detail* gdp, const GEO_Point* ppt);
		void addObstacle(const GU_Detail* obstacleGdp, int curPtNum);
		void addPath(const GU_Detail* pathGdp, int curPrimNum);
		void updateSteerWorld(const float currentTime, const float elapsedTime);
		void updatePoint(GU_Detail* gdp, int curPtNum);
	private:
		ops::steer_world* mySteerWorldPtr;
	};

}

#endif /* STEER_WORLD_ADAPTER_H_ */
