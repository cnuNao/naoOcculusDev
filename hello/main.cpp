/*
 * Module for extracting data from the robot
 *
 * @author juanvallejo
 */

#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alcommon/alproxy.h>

#include <sstream>

// import udp headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdio.h>

#include <time.h>

const int remotePort		= 9559;
std::string remoteAddress	= "192.168.10.103";

bool HEAD_MOVED = false;

bool nao_moveHead(AL::ALMotionProxy& proxy, std::vector<float> radians) {

	bool anglesAreAbsolute 			= true;

	float yaw 						= radians[0];
	float pitch 					= radians[1];

	// action guards
	if(radians[0] > -0.1f && radians[0] < 0.1f && radians[1] > -0.1 && radians[1] < 0.1) {
		std::cout << "Movement delta too low. Ignoring..." << std::endl;
		HEAD_MOVED = false;
		return false;

	}

	// yaw guards
	if(radians[0] > 1.4f || radians[0] < -1.4f) {
		std::cout << "ERR Yaw value limit exceeded" << std::endl;
		if(radians[0] < 0) {
			yaw = -1.4f;
		} else {
			yaw = 1.4f;
		}
		
	}

	// pitch guards
	if(radians[1] > 0.6f || radians[1] < -0.6f) {
		std::cout << "ERR Pitch value limit exceeded" << std::endl;
		if(radians[1] < 0) {
			pitch = -0.6f;
		} else {
			pitch = 0.6f;
		}
		
	}

	// adjust pitch
	pitch *= -1.0f;

	try {

		// set angles for head, in radians
		AL::ALValue jointNames 	= AL::ALValue::array("HeadYaw", "HeadPitch");

		// set target times, at which angles wiill be reached
		AL::ALValue targetTimes;
		AL::ALValue targetAngles;

		targetTimes.arraySetSize(2);
		targetAngles.arraySetSize(2);

		targetTimes[0] = AL::ALValue::array(0.3f);
		targetTimes[1] = AL::ALValue::array(0.3f);

		targetAngles[0] = AL::ALValue::array(yaw);
		targetAngles[1] = AL::ALValue::array(pitch);

		// call the angle interpolation method.
		// The joint will reach the desired angle at the specified time.
		proxy.angleInterpolation(jointNames, targetAngles, targetTimes, anglesAreAbsolute);

		HEAD_MOVED = true;

	} catch(const AL::ALError& error) {
		std::cerr << "Caught exception: " << error.what() << std::endl;
		HEAD_MOVED = false;
	}

	return true;

}

bool nao_sayPhrase(const char *phrase) {

	std::string phraseToSay = phrase;
	if(phraseToSay == "") {
		return false;
	}

	try {

		AL::ALProxy textToSpeechProxy("ALTextToSpeech", remoteAddress, remotePort);
		textToSpeechProxy.callVoid("say", phraseToSay);
	
	} catch(const AL::ALError& error) {
		std::cerr << "Caught exception: " << error.what() << std::endl;
		exit(1);
	}

	return true;

}

/**
 * Split an std::string into a vector of floats
 * containing yaw, pitch, and roll from a udp
 * stream.
 *
 * @return std::vector<float> {yaw, pitch, roll}
 */
std::vector<float> split_string(std::string string) {


	std::stringstream stream(string);
	std::string word;

	std::vector<float> data;

	for(;std::getline(stream, word, ',');) {
		data.push_back(std::atof(word.c_str()));
	}

	return data;

}

int get_accel_stream(AL::ALMotionProxy& motionProxy) {

	int sockfd;
	int n;

	struct sockaddr_in servaddr;
	struct sockaddr_in cliaddr;

	// yaw 		= accel_data[0]
	// pitch 	= accel_data[1]
	std::vector<float> accel_data;

	socklen_t len;

	// make room for message received
	char mesg[1000];

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	// set first 0 bytes of the area starting at first
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
		std::string message(mesg);
		accel_data = split_string(message);

		std::cout << "Accelerometer data received: " << accel_data[0] << "," << accel_data[1] << std::endl;

		nao_moveHead(motionProxy, accel_data);
		if (!HEAD_MOVED)
		{
			usleep(250000);
		}
	}

	return 0;

}

int main(int argc, char** argv) {

	// if an extra text argument is passed, say it
	if(argc > 1) {
		nao_sayPhrase(argv[1]);
	}

	const AL::ALValue headYaw 		= "HeadYaw";
	const AL::ALValue headPitch 	= "HeadPitch";

	// set initial stiffness of head in order to be able to move it.
	// If head is not stiff, it cannot be moved. Target time is the second to execute stiffness command.
	AL::ALValue headStiffness 		= 1.0f;
	AL::ALValue targetTime 			= 1.0f;

	// create an almotion proxy
	AL::ALMotionProxy motionProxy(remoteAddress, remotePort);

	// call stiffness interpolation method, obtain contol of head
	motionProxy.stiffnessInterpolation(headYaw, headStiffness, targetTime);
	motionProxy.stiffnessInterpolation(headPitch, headStiffness, targetTime);
	
	std::cout << "Listening for udp input" << std::endl;
	get_accel_stream(motionProxy);

	return 0;

}
