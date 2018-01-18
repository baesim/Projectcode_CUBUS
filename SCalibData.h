/*
 * SCalibData.h
 *
 *  Created on: Dec 22, 2017
 *      Author: vmuser
 */

#ifndef SCALIBDATA_H_
#define SCALIBDATA_H_
#include "Global.h"

struct SCalibData
{
public:
	float mx1_dd_factor;
	float mx1_dd_offset;
	float mphi1_d_factor;
	float mphi1_d_offset;
	float my1_dd_factor;
	float my1_dd_offset;
	float mx2_dd_factor;
	float mx2_dd_offset;
	float mphi2_d_factor;
	float mphi2_d_offset;
	float my2_dd_factor;
	float my2_dd_offset;
	float mpsi_d_factor;
	float mpsi_d_offset;
	int mADC_signedzero;

};





#endif /* SCALIBDATA_H_ */
