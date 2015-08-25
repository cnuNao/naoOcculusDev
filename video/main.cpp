/*
 * Module for capturing video from the robot's front camera.
 *
 * @author juanvallejo
 */

#include <iostream>

#include <signal.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>
#include "videocapture.h"

#define ALCALL

ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> broker) {

	// init broker with instance from the parent executable
	AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
	AL::ALBrokerManager::getInstance()->addBroker(broker);
	AL::ALModule::createModule<VideoCapture>(broker, "VideoCapture");

	return 0;

}

ALCALL int _closeModule() {
	return 0;
}

int main(int argc, char *argv[]) {

	// pointer to createModule
	TMainType sig;
	sig = &_createModule;

	// call main
	ALTools::mainFunction("VideoCapture", argc, argv, sig);

  return 0;
}
