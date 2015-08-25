/**
 * Video capturing module definition
 * @author juanvallejo
 */

#ifndef VIDEOCAPTURE_VIDEOCAPTUREREMOTE_H
#define VIDEOCAPTURE_VIDEOCAPTUREREMOTE_H

#include <string>

#include <alcommon/almodule.h>
#include <alproxies/alvideodeviceproxy.h>
#include <pthread.h>

// include all ALBroker dependencies
namespace AL {
	class ALBroker;
}

class VideoCapture : public AL::ALModule {

public:

	// define constructor and destructor
	VideoCapture(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
	virtual ~VideoCapture();

	void init();

	bool isCapturing();

	/**
	 * Start capturing video
	 */
	void startCapture();

	// stop capturing video
	void stopCapture();

private:

	bool fCapturingVideo;

	// start capturing video
	void xStartVideo();

	// stop capturing video
	void xStopVideo();

	// proxy to the video device
	AL::ALVideoDeviceProxy fALVideoDevice;

	// video subscriber id
	std::string fVideoSubscriberId;

	// video framerate
	int fFramerate;

	// vide thread object
	pthread_t fVideoThread;

	// our video capturing loop
	static void* xVideoThread(void *pArg);

	// get time in US
	static long long xGetTime();

};

#endif 
// VIDEOCAPTURE_VIDEOCAPTUREREMOTE_H