/*
 * CControlComp.h
 *
 *  Created on: Dec 9, 2017
 *      Author: Baehr Simon, Kempf Maurizio
 */

#ifndef CCONTROLCOMP_H_
#define CCONTROLCOMP_H_

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include "IRunnable.h"
#include "CContainer.h"
#include "CBBBHardware.h"
#include "SCalibData.h"
#include "SReglerParam.h"


class CControlComp: public IRunnable
{
public:
	void run();
	void init();

public:
	CControlComp(CContainer*);

private:
	void fetch_values();
	void calib();
	void calc();
	void filter();
	void regler();
	void jump();
private:
	CContainer* mContainer;
	CBBBHardware mHardware;

	UInt16 mADCValue;
	SMPU6050Data mSensor1;
	SMPU6050Data mSensor2;

	float mx1_dd;
	float mx2_dd;
	float my1_dd;
	float my2_dd;
	float phi1_d;
	float phi2_d;

	SStateVectorData mStateVector_calc;
	SStateVectorData mStateVector_filt;

	SCalibData mCalibData;
	SReglerParam mReglerParam;

	timeval timeend;
	timeval timebegin;

	int seconds;
	int useconds;
	float mT_a;
	float malpha;
	float mr_S1;
	float mr_S2;
	float mpsi_val[8];
	float mpsi_sumval;
	float mTorgueMotor;

	float mWinkelOffset;


protected:
};
#endif /* CCONTROLCOMP_H_ */
