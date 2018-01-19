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

private:
	CContainer* mContainer;
	CBBBHardware mHardware;

	UInt16 mADCValue;
	SMPU6050Data mSensor1;
	SMPU6050Data mSensor2;

	SStateVectorData mStateVector_calc;
	SStateVectorData mStateVector_filt;

	SCalibData mCalibData;
	SReglerParam mReglerParam;

	timeval timeend;
	timeval timebegin;

	const float mT_a;			// Zeitkonstante Komplementärfilter (tau = 1/100Hz [s])
	const float mAlpha;			// 1-mAlpha = Tiefpass-Verstärkung, mAlpha = Hochpass-Verstärkung
	const float mR_S1;			// Radius Sensor S1 zu Bezugspunkt A
	const float mR_S2;			// Radius Sensor S2 zu Bezugspunkt A
	const float mWinkelOffset;  // zur Nachkalibration des durch den Versatz der Sensoren entsehenden Offsets

	int seconds;
	int useconds;

	float mx1_dd;
	float mx2_dd;
	float my1_dd;
	float my2_dd;
	float phi1_d;
	float phi2_d;
	float mpsi_val[8];
	float mpsi_sumval;
	float mTorgueMotor;

};
#endif /* CCONTROLCOMP_H_ */
