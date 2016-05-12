import com.aldebaran.qi.Application;
import com.aldebaran.qi.helper.proxies.ALTextToSpeech;
import com.aldebaran.qi.CallError;
import com.aldebaran.qi.Session;
import com.aldebaran.qi.helper.EventCallback;
import com.aldebaran.qi.helper.proxies.ALMemory;
import com.aldebaran.qi.helper.proxies.ALMotion;

import java.util.ArrayList;
import java.util.List;
import java.net.*;

public class HeadMovement {

	static boolean HEAD_MOVED = false;
	static int dataCount = 0;

	static float dataYawAverage = 0.0f;
	static float dataPitchAverage = 0.0f;

	static float dataYawAverageLastValue = 0.0f;
	static float dataPitchAverageLastValue = 0.0f;
	
	final static String headYaw = "HeadYaw";
	final static String headPitch = "HeadPitch";

	final static float headStiffness = 1.0f;
	final static float targetTime = 1.0f;

	public static int getAccelStream(ALMotion motion) {



		try {

         // Construct the socket
         DatagramSocket socket = new DatagramSocket(14552) ;

         for( ;; ) {

            DatagramPacket packet = new DatagramPacket(new byte[1000], 1000);
            socket.receive(packet);


            // Print the packet
            String accel_data[] = new String(packet.getData()).split(",");
            		// yaw 		= accel_data[0]
					// pitch 	= accel_data[1]
            moveHead(motion, accel_data);
            if(!HeadMovement.HEAD_MOVED)
            {
            	Thread.sleep(250);
            }

        }  
     }
     catch(Exception e) {
     	e.printStackTrace();
     }

		return 0;

	}

	public static boolean moveHead(ALMotion motion, String[] radians) {
		
		boolean anglesAreAbsolute 			= true;

		float yaw 						= Float.parseFloat(radians[0]);
		float pitch 					= Float.parseFloat(radians[1]);

		// yaw guards
		if(Float.parseFloat(radians[0]) > 1.4f || Float.parseFloat(radians[0]) < -1.4f) {
			System.out.println("ERR Yaw value limit exceeded, was " + radians[0] + ", limit is 1.4f");
			if(Float.parseFloat(radians[0]) < 0) {
				yaw = -1.4f;
			} else {
				yaw = 1.4f;
			}
			
		}

		// pitch guards
		if(Float.parseFloat(radians[1]) > 0.6f || Float.parseFloat(radians[1]) < -0.6f) {
			System.out.println("ERR Pitch value limit exceeded");
			if(Float.parseFloat(radians[1]) < 0) {
				pitch = -0.6f;
			} else {
				pitch = 0.6f;
			}
			
		}

		// adjust pitch
		pitch *= -1.0f;

		if(HeadMovement.dataCount >= 2) {

			// reset dataCount
			HeadMovement.dataCount = 0;

			HeadMovement.dataYawAverage /= 2.0;
			HeadMovement.dataPitchAverage /= 2.0;

			// std::cout << "Moving head yaw with value " << dataYawAverage << " (diff = " << std::abs((std::abs(dataYawAverageLastValue) - std::abs(dataYawAverage))) << ")" << std::endl;
			// std::cout << "Moving head pitch with value " << dataPitchAverage << std::endl;

			// if(std::abs(std::abs(dataYawAverageLastValue) - std::abs(dataYawAverage)) < 0.5 || std::abs(std::abs(dataPitchAverageLastValue) - std::abs(dataPitchAverage)) < 0.5) {
				// std::cout << "Movement  delta too low... (" << "..." << ") Ignoring..." << std::endl;
			// } else {

				try {

					// set angles for head, in radians
					List<String> jointNames = new ArrayList<String>();
					jointNames.add("HeadYaw");
					jointNames.add("HeadPitch");

					// set target times, at which angles wiill be reached
					List<Float> targetTimes = new ArrayList<Float>();
					targetTimes.add(0.35f);
					targetTimes.add(0.35f);

					List<Float> targetAngles = new ArrayList<Float>();
					targetAngles.add(dataYawAverage);
					targetAngles.add(dataPitchAverage);

					System.out.println("MOVING HEAD: Movement information gathered... " + "(" + dataYawAverage + ", " + dataPitchAverage + ") [" + Math.abs((Math.abs(dataYawAverageLastValue) - Math.abs(dataYawAverage))) + "]");

					// call the angle interpolation method.
					// The joint will reach the desired angle at the specified time.
					motion.angleInterpolation(jointNames, targetAngles, targetTimes, anglesAreAbsolute);

					HeadMovement.HEAD_MOVED = true;

				} catch(Exception e) {
					e.printStackTrace();						
					HeadMovement.HEAD_MOVED = false;
				}

			HeadMovement.dataPitchAverage = 0.0f;
			HeadMovement.dataYawAverage = 0.0f;

			HeadMovement.dataYawAverageLastValue = HeadMovement.dataYawAverage;
			HeadMovement.dataPitchAverageLastValue = HeadMovement.dataPitchAverage;

		} else {

			HeadMovement.dataCount++;
			HeadMovement.dataYawAverage += yaw;
			HeadMovement.dataPitchAverage += pitch;

		}

		return true;
	}

	public static void main(String args[]) throws Exception {

		String ipAddress = "nao.local";

		// treat first arg as ip address of the robot
		if(args.length > 0) {
			ipAddress = args[0];
		}

		String robotAddress = "tcp://" + ipAddress + ":9559";

		Application app = new Application(args, robotAddress);

		try {

			app.start();

			ALMotion motion = new ALMotion(app.session());
			motion.stiffnessInterpolation(HeadMovement.headYaw, HeadMovement.headStiffness, HeadMovement.targetTime);
			motion.stiffnessInterpolation(HeadMovement.headPitch, HeadMovement.headStiffness, HeadMovement.targetTime);

			System.out.println("Listening for udp input...");
			HeadMovement.getAccelStream(motion);

		} catch(Exception e) {
			e.printStackTrace();
		}

	}

}