/*
 * main.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: Baehr Simon, Kempf Maurizio
 */

#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include "CThread.h"
#include "CContainer.h"
#include "IRunnable.h"
#include "CControlComp.h"
#include "CCommComp.h"

CContainer myContainer;

int main()
{
	CCommComp myCommunication(&myContainer);
	CControlComp myControl(&myContainer);

	CThread commThread(&myCommunication, CThread::PRIORITY_NORM);
	CThread contThread(&myControl, CThread::PRIORITY_NORM);

	commThread.start();
	contThread.start();

	commThread.join();
	contThread.join();

	return 0;
}



