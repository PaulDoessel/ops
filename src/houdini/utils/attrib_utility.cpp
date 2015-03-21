/*
 * attrib_utility.cpp
 *
 *  Created on: 31 Jan, 2013
 *      Author: jaideep
 */

#include "attrib_utility.h"

#include "UT/UT_Vector3.h"
#include "GEO/GEO_Point.h"
#include "GU/GU_Detail.h"
#include "GEO/GEO_AttributeHandle.h"

namespace houdini_ops
{

	attrib_utility::attrib_utility()
	{
		// TODO Auto-generated constructor stub

	}

	attrib_utility::~attrib_utility()
	{
		// TODO Auto-generated destructor stub
	}

	void attrib_utility::setNormal(GU_Detail* gdp, int curPtNum, UT_Vector3 val)
	{
		GEO_Point* ppt = gdp->points()[curPtNum];
		GEO_AttributeHandle n_gah;
		n_gah = gdp->getPointAttribute("N");
//		if (!n_gah.isAttributeValid())
//		{
//			float def[3] =
//			{ 0, 0, 0 };
//			// Difference of points is vector, thus our choice here.
//			gdp->addPointAttrib((const char *) "N", 3 * sizeof(float), GB_ATTRIB_VECTOR, GB_ATTRIB_INFO_NONE,
//								def);
//			n_gah = gdp->getPointAttribute("N");
//		}
		n_gah.setElement(ppt);
		n_gah.setV3(val, 0);
	}

}
