/*
 * Handle executable run. Creates a remote ALModule, passes its pointer to
 * ALTools mainFunction handler, and initiates created module with a broker passed
 * by ALTools::mainFunction.
 *
 * @author juanvallejo
 */

#include <iostream>

#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "onfacedetection.h"

#define ALCALL

extern "C" {

	ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> broker) {

		// init broker with main instance from parent executable
		AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
		AL::ALBrokerManager::getInstance()->addBroker(broker);

		AL::ALModule::createModule<OnFaceDetection>(broker, "OnFaceDetection");

		return 0;

	}

	ALCALL int _closeModule() {
		return 0;
	}

}

int main(int argc, char *argv[]) {

	// pointer to _createModule
	TMainType sig;
	sig = &_createModule;

	// call main
	ALTools::mainFunction("onfacedetection", argc, argv, sig);

	return 0;

}
