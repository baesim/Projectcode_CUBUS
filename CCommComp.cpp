/*
 * CCommComp.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: Baehr Simon, Kempf Maurizio
 */
#include "CCommComp.h"

CCommComp::CCommComp(CContainer* myContainer): mContainer(myContainer)
{

}

void CCommComp::init()
{
	mServer.init(); //init server
}
void CCommComp::run()
{

}
