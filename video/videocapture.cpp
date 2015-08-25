/**
 * Video capture module implementation
 *
 * @author juanvallejo
 */

#include <iostream>
#include "videocapture.h"

#include <qi/os.hpp>
#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alvision/alvisiondefinitions.h>

/**
 * Constructor for the VideoCapture object
 *
 * @param broker The parent broker
 * @param name The name of the module
 */
VideoCapture::VideoCapture(boost::shared_ptr<AL::ALBroker> broker, const std::string& name):
	AL::ALModule(broker, name),
	fCapturingVideo(false),
	fALVideoDevice(broker),
	fVideoSubscriberId(),
	fFramerate(5),
	fVideoThread() {

		setModuleDescription("Captures video.");

		functionName("isCapturing", "VideoCapture", "Says if the capture was started");
		setReturn("capturing", "Whether capture was started.");
		BIND_METHOD(VideoCapture::isCapturing);

		functionName("startCapture", "VideoCapture", "Starts visual capture.");
		BIND_METHOD(VideoCapture::startCapture);

		functionName("stopCapture", "VideoCapture", "Stops video capturing");
		BIND_METHOD(VideoCapture::stopCapture);

	}

VideoCapture::~VideoCapture() {
	stopCapture();
}

void VideoCapture::init() {
	startCapture();
}

bool VideoCapture::isCapturing() {
	return fCapturingVideo;
}

void VideoCapture::startCapture() {
	
	if(isCapturing()) {
		throw std::runtime_error("I am already capturing video.");
	}

	xStartVideo();

}

void VideoCapture::stopCapture() {
	xStopVideo();
}

void VideoCapture::xStartVideo() {

	try {

		fVideoSubscriberId = fALVideoDevice.subscribe(getName(), AL::kQQVGA, AL::kRGBColorSpace, fFramerate);
		fCapturingVideo = true;
		pthread_create(&fVideoThread, 0, xVideoThread, this);

	} catch(const std::exception &error) {
		std::cerr << "Cannot subscribe to video: " << error.what() << std::endl;
		fCapturingVideo = false;
	}

}

void VideoCapture::xStopVideo() {

	void* lReturn;
	fCapturingVideo = false;
	pthread_join(fVideoThread, &lReturn);

	try {
		fALVideoDevice.unsubscribe(fVideoSubscriberId);
	} catch(const std::exception &error) {
		std::cerr << "Unable to unsubscribe from video: " << error.what() << std::endl;
	}

}

void* VideoCapture::xVideoThread(void *pArg) {

	VideoCapture* lThis = (VideoCapture*)pArg;
	AL::ALValue lImage;
	lImage.arraySetSize(7);

	// time measurement
	long long lLastBufferTime = xGetTime();

	// Buffer duration depends on framerate
	long long lDuration = 1000000000 / lThis->fFramerate; // in nanoseconds

	// extract data
	try {

		// our actual video loop
		while(lThis->fCapturingVideo) {

			if((xGetTime() - lLastBufferTime) < lDuration / 1000) {
				qi::os::msleep(10);
				continue;
			}

			// get an image and its metadata
			lImage = lThis->fALVideoDevice.getImageRemote(lThis->fVideoSubscriberId);

			// parse video data
			std::cout << lImage.toString() << std::endl;
			// end parsing video data

			lThis->fALVideoDevice.releaseImage(lThis->fVideoSubscriberId);

		}

	} catch(const std::exception &error) {
		
		std::cerr << "Video loop stopped -> " << error.what() << std::endl;

		if(lThis->fCapturingVideo) {
			lThis->xStopVideo();
		}

	}

	return 0;

}

long long VideoCapture::xGetTime() {
	
	qi::os::timeval lTimeStruct;
	qi::os::gettimeofday(&lTimeStruct);

	long long lTime = (long long)lTimeStruct.tv_sec * 1000000; // seconds

	lTime += (long long)lTimeStruct.tv_usec;

	return lTime;

}