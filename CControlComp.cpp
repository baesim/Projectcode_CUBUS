/*
 * CControlComp.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: Baehr Simon, Kempf Maurizio
 */
#include "CControlComp.h"

CControlComp::CControlComp(CContainer* myContainer): mContainer(myContainer)
{
	//Init-Values
	mADCValue = 0;
	seconds = 0;
	useconds = 0;
	//Filter const
	mpsi_sumval = 0.0;
	mT_a = 0.01;
	malpha =0.98;
	//calc Const
	mr_S1 = 0.14;
	mr_S2 = 0.061;
	//regler const
	mTorgueMotor = 0.0;
	//Calib const
	mCalibData.mx1_dd_factor   = -0.0005941;
	mCalibData.mx1_dd_offset   = -0.2553;
	mCalibData.my1_dd_factor   = -0.0006183;
	mCalibData.my1_dd_offset   =  0.2341;
	mCalibData.mphi1_d_factor  =  69.81317 / 32768; //4.3633 / 32768;		//DAS //Gyro-MaxWert(250°/s in rad/s)/ 16bit/2
	mCalibData.mphi1_d_offset  =  0.021;

	mCalibData.mx2_dd_factor   = -0.0006251;
	mCalibData.mx2_dd_offset   =  0.2083;
	mCalibData.my2_dd_factor   = -0.0005894;
	mCalibData.my2_dd_offset   = -0.0621;
	mCalibData.mphi2_d_factor  =  69.81317 / 32768; 	//DAS //Gyro-MaxWert(250°/s in rad/s)/ 16bit/2
	mCalibData.mphi2_d_offset  =  0.0447411;

	mCalibData.mpsi_d_factor   = -0.7666;	//DAS //Psi_d-MaxWert*2/ 12bit
	mCalibData.mpsi_d_offset   = -12.0577;
	mCalibData.mADC_signedzero =  4096 / 2;
	//Regler Param
	mReglerParam.mP_psi_d 	   =  -0.0002;
	mReglerParam.mP_phi_d	   =  -0.0971;
	mReglerParam.mP_phi 	   =  -1.3813;

	mWinkelOffset = 0.1486;

	mx1_dd = 0;
	mx2_dd = 0;
	my1_dd = 0;
	my2_dd = 0;
	phi1_d = 0;
	phi2_d = 0;

	mStateVector_filt.mPhi_C = 0;

}
void CControlComp::init()
{

}
void CControlComp::run()
{
	mHardware.enableMotor();
	//jump();
	//std::cout << "ICH STARTE" << std::endl;
	while(1)
	{
		gettimeofday(&timebegin,(struct timezone*)0);

		fetch_values();
		calib();
		calc();
		filter();
		regler();

		//mContainer->writeStateData(mStateVector_filt);
		//mContainer->signalReader();

		gettimeofday(&timeend,(struct timezone*)0);
		seconds = timeend.tv_sec - timebegin.tv_sec;
		useconds = timeend.tv_usec - timebegin.tv_usec;
		if(useconds < 0)
		{
			useconds += 1000000;
			seconds--;
		}
		usleep(20000-useconds);

	}
}
void CControlComp::fetch_values()
{
	mHardware.fetchValues(mADCValue, mSensor1, mSensor2);
}
void CControlComp::calib()
{
	mx1_dd = mCalibData.mx1_dd_factor  * mSensor1.mX__dd + mCalibData.mx1_dd_offset;
	my1_dd = mCalibData.my1_dd_factor  * mSensor1.mY__dd + mCalibData.my1_dd_offset;
	phi1_d = mCalibData.mphi1_d_factor * mSensor1.mPhi__d + mCalibData.mphi1_d_offset;
	mx2_dd = mCalibData.mx2_dd_factor  * mSensor2.mX__dd + mCalibData.mx2_dd_offset;
	my2_dd = mCalibData.my2_dd_factor  * mSensor2.mY__dd + mCalibData.my2_dd_offset;
	phi2_d = mCalibData.mphi2_d_factor * mSensor2.mPhi__d + mCalibData.mphi2_d_offset;
}
void CControlComp::calc()
{
	mStateVector_calc.mPsi__d = mCalibData.mpsi_d_factor * (mADCValue - mCalibData.mADC_signedzero) + mCalibData.mpsi_d_offset;
	mStateVector_calc.mPhi__d = (phi1_d + phi2_d)/2.0;
	mStateVector_calc.mPhi_A = mWinkelOffset + atan(((mr_S1/mr_S2)*mx2_dd - mx1_dd)/((mr_S1/mr_S2)*my2_dd  - my1_dd ));//0.154
}
void CControlComp::filter()
{
	//moving average
	for(int i=0;i<7;i++)
	{
		mpsi_val[i] = mpsi_val[i+1];
	}
	mpsi_val[7] = mStateVector_calc.mPsi__d;

	for(int i=0;i<8;i++)
	{
		mpsi_sumval += mpsi_val[i];
	}
	mStateVector_filt.mPsi__d = 0.125 * mpsi_sumval;
	mpsi_sumval = 0;
	//complimentary filter
	mStateVector_filt.mPhi_C = malpha*(mStateVector_filt.mPhi_C +(mT_a*mStateVector_calc.mPhi__d)) + (1-malpha)*mStateVector_calc.mPhi_A;
	//no filtering
	mStateVector_filt.mPhi__d = mStateVector_calc.mPhi__d;
}
void CControlComp::regler()
{
	mTorgueMotor = mReglerParam.mP_psi_d * mStateVector_filt.mPsi__d + mReglerParam.mP_phi_d * mStateVector_filt.mPhi__d + mReglerParam.mP_phi * mStateVector_filt.mPhi_C;
	if(mStateVector_filt.mPhi_C < 0.1 && mStateVector_filt.mPhi_C > -0.1)
	{
		mHardware.setTorque(-mTorgueMotor);
	}
	else
	{
		mHardware.setTorque(0);
	}
}

void CControlComp::jump()
{
	mHardware.openBrake();
	mHardware.setTorque(-0.08);
	usleep(2000000);
	mHardware.setTorque(0);
	mHardware.closeBrake();
	//usleep(120000);
	//mHardware.openBrake();
}
