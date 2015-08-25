/**
 * Face detection class
 *
 * @author juanvallejo
 */

#include "onfacedetection.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

// declare constructor
OnFaceDetection::OnFaceDetection(boost::shared_ptr<AL::ALBroker> broker, const std::string& name):
	AL::ALModule(broker, name),
	fMemoryProxy(getParentBroker()),
	fFaces(AL::ALValue()),
	fFaceCount(0),
	fCallbackMutex(AL::ALMutex::createALMutex()) {

		setModuleDescription("Module that detects faces and acts accordingly.");

		functionName("callback", getName(), "");
		BIND_METHOD(OnFaceDetection::callback);

	}

// define destructor
OnFaceDetection::~OnFaceDetection() {}

// define init method
void OnFaceDetection::init() {

	try {

		// check to see if there are faces present at module initialization
		fFaces = fMemoryProxy.getData("FaceDetected");

		if(fFaces.getSize() < 2) {
			qiLogInfo("module.face") << "No faces detected" << std::endl;
			fTextToSpeechProxy.say("I... can't... see you.");
		}

		// subscribe to "FaceDetected" event, with the appropriate callback function
		fMemoryProxy.subscribeToEvent("FaceDetected", "OnFaceDetection", "callback");

	} catch(const AL::ALError& error) {
		qiLogError("module.face") << error.what() << std::endl;
	}

}

// define our callback function
void OnFaceDetection::callback() {

	// use a mutex to make this function thread safe
	AL::ALCriticalSection section(fCallbackMutex);

	try {

		// retrieves FaceDetected event data
		fFaces = fMemoryProxy.getData("FaceDetected");

		// check to see that there have been faces detected
		if(fFaces.getSize() < 2) {
			// if a face has been previously detected, by any thread, and now there are zero
			// faces detected, then advertise that the robot cannot see the person / people anymore
			if(fFaceCount != 0) {
				qiLogInfo("module.face") << "No faces detected" << std::endl;
				// fTextToSpeechProxy.say("Where. are. you? \\vct=5\\\\rspd=10\\\\pau=200\\I\\pau=50\\ am\\pau=50\\ coming\\pau=20\\ for\\pau=20\\ you.\\pau=1500\\ you\\pau=30\\ can\\pau=30\\ run\\pau=30\\ but\\pau=30\\ you\\pau=20\\ can't\\pau=30\\\\vct=1\\ hide");
				fTextToSpeechProxy.say("Hey! Where'd you \\vct=120\\go!?");
				fFaceCount = 0;
			}

			return;

		}

		// if we've gotten this far, then faces have been detected. Check info on them.
		// if the number / info on the face being detected has not changed, do nothing.
		if(fFaces[1].getSize() - 1 != fFaceCount) {
			
			qiLogInfo("module.face") << (fFaces[1].getSize() - 1) << " face(s) detected." << std::endl;
			
			if(fFaces[1].getSize() > 2) {
				char buffer[50];
				sprintf(buffer, "I see %d faces!", fFaces[1].getSize() - 1);
				fTextToSpeechProxy.say(std::string(buffer));
			}

			fTextToSpeechProxy.say("There you are!");

			// update the number of detected faces
			fFaceCount = fFaces[1].getSize() - 1;

		}

	} catch(const AL::ALError& error) {
		qiLogError("module.face") << error.what() << std::endl;
	}

}