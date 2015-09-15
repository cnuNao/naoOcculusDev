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

// import udp headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdio.h>

int get_udp() {

	int sockfd;
	int n;

	struct sockaddr_in servaddr;
	struct sockaddr_in cliaddr;

	socklen_t len;

	// make room for message received
	char mesg[1000];

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	// set first n bytes of the area starting at first
	// address location of our servaddr to zero. ('\0')
	memset(&servaddr, sizeof(servaddr), 0);
	servaddr.sin_family = AF_INET;

	// convert '0.0.0.0' to socket byte
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	// convert integer 32000 (representing the port to bind the socket to)
	// to socket form
	servaddr.sin_port = htons(14552);

	// bind the socket. Cast mem area of servaddr to type
	// of struct sockaddr memory (socket friendly stuff)
	bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

	// loop forever listening for messages
	while(1) {

	  len = sizeof(cliaddr);

	  // receive message from 0.0.0.0 on port 32000, cast the client address to
	  // socket friendly memory space
	  n = recvfrom(sockfd, mesg, 1000, 0, (struct sockaddr *)&cliaddr, &len);
	  sendto(sockfd, mesg, n, 0, (struct sockaddr *)&cliaddr, len);

	  mesg[n] = 0;
	  printf("Message received: %s \n", mesg);

	}

}

int main(int argc, char** argv) {

	curlpp::Cleanup cleanup;
	curlpp::Easy 	request;

	const int remotePort		= 9559;

	std::string remoteAddress	= "192.168.10.103";
	std::string phraseToSay 	= "I will listen for udp messages after my head is done moving.";

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
		AL::ALValue targetTimes 	= AL::ALValue::array(1.0f, 1.0f, 1.0f);

		// define if angles are absolute
		bool anglesAreAbsolute 		= true;

		// call the angle interpolation method.
		// The joint will reach the desired angle at the specified time.
		motionProxy.angleInterpolation(jointName, targetAngles, targetTimes, anglesAreAbsolute);

		// remove the stiffness on the head. We no longer need to move it.
		headStiffness 	= 0.0f;
		targetTime 		= 1.0f;

		motionProxy.stiffnessInterpolation(jointName, headStiffness, targetTime);

		std::cout << "Listening for udp input" << std::endl;
		get_udp();	

	} catch(const AL::ALError& error) {
		std::cerr << "Caught exception: " << error.what() << std::endl;
		exit(1);
	}

	return 0;

}
