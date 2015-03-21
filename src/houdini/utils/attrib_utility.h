/*
 * attrib_utility.h
 *
 *  Created on: 31 Jan, 2013
 *      Author: jaideep
 */

#ifndef ATTRIB_UTILITY_H_
#define ATTRIB_UTILITY_H_

#include "UT/UT_Vector3.h"
#include "GU/GU_Detail.h"

namespace houdini_ops {

class attrib_utility {
public:
	attrib_utility();
	virtual ~attrib_utility();

	void setNormal(GU_Detail* gdp, int curPtNum,UT_Vector3 val);
};

}

#endif /* ATTRIB_UTILITY_H_ */
