/*
 * CCommComp.h
 *
 *  Created on: Dec 9, 2017
 *      Author: Baehr Simon, Kempf Maurizio
 */

#ifndef CCOMMCOMP_H_
#define CCOMMCOMP_H_

#include "SContent.h"
#include "IRunnable.h"
#include "Config.h"
#include "CServer.h"
#include "CContainer.h"
#include <iostream>

class CCommComp: public IRunnable
{
public:
	void run();
	void init();

public:
	CCommComp(CContainer*);

private:
	CContainer* mContainer;
	CServer mServer;
	SContent mContent;

protected:
};

#endif /* CCOMMCOMP_H_ */
