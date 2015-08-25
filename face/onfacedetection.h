/**
 * Face detection class declaration
 *
 * @author juanvallejo
 */

#ifndef ONFACEDETECTION_ONFACEDETECTION_H
#define ONFACEDETECTION_ONFACEDETECTION_H

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <string>

#include <alproxies/almemoryproxy.h>
#include <alproxies/altexttospeechproxy.h>

#include <althread/almutex.h>

namespace AL {
	class ALBroker;
}

class OnFaceDetection : public AL::ALModule {

public:

	OnFaceDetection(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);

	virtual ~OnFaceDetection();
	virtual void init();

	/**
	 * This method is called every time a face is detected.
	 * It makes NAO say the number of detected faces.
	 * Accessed by multiple threads.
	 */
	void callback();

private:

	// memory proxy. Used to subscribe events and access data.
	AL::ALMemoryProxy fMemoryProxy;
	AL::ALTextToSpeechProxy fTextToSpeechProxy;

	// stores number of detected faces
	AL::ALValue fFaces;

	// stores number of detected faces
	unsigned int fFaceCount;

	// mutex to make the callback function thread safe.
	boost::shared_ptr<AL::ALMutex> fCallbackMutex;

};

#endif