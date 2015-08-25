/*
 * Module for extracting data from the robot
 *
 * @author juanvallejo
 */

#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alcommon/alproxy.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

int main(int argc, char** argv) {

	curlpp::Cleanup cleanup;
	curlpp::Easy 	request;

	const int remotePort		= 9559;

	std::string remoteAddress	= "192.168.10.106";
	std::string phraseToSay 	= "Hey Jake... Don't be leettle beech.";

	if(argc > 1) {
		phraseToSay = argv[1];
	}

	// create a proxy module and invoke its 'say' method
	AL::ALProxy textToSpeechProxy("ALTextToSpeech", remoteAddress, remotePort);
	textToSpeechProxy.callVoid("say", phraseToSay);

	// move the head of the robot
	const AL::ALValue jointName = "HeadYaw";

	try {

		/**
		 * We are forced to import and use the ALMotionProxy module, as it is not available through
		 * the generic ALProxy module
		 */

		// create an almotion proxy
		AL::ALMotionProxy motionProxy(remoteAddress, remotePort);
		
		// set initial stiffness of head in order to be able to move it.
		// If head is not stiff, it cannot be moved. Target time is the second to execute stiffness command.
		AL::ALValue headStiffness 		= 1.0f;
		AL::ALValue targetTime 			= 1.0f;

		// call stiffness interpolation method
		motionProxy.stiffnessInterpolation(jointName, headStiffness, targetTime);

		// set angles for head, in radians
		AL::ALValue targetAngles 	= AL::ALValue::array(-1.5f, 1.5f, 0.0f);

		// set target times, at which angles wiill be reached
		AL::ALValue targetTimes 	= AL::ALValue::array(3.0f, 6.0f, 9.0f);

		// define if angles are absolute
		bool anglesAreAbsolute 		= true;

		// call the angle interpolation method.
		// The joint will reach the desired angle at the specified time.
		motionProxy.angleInterpolation(jointName, targetAngles, targetTimes, anglesAreAbsolute);

		// remove the stiffness on the head. We no longer need to move it.
		headStiffness 	= 0.0f;
		targetTime 		= 1.0f;

		motionProxy.stiffnessInterpolation(jointName, headStiffness, targetTime);		

	} catch(const AL::ALError& error) {
		std::cerr << "Caught exception: " << error.what() << std::endl;
		exit(1);
	}

	return 0;

}
