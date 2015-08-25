Communicating with robot
========================

This guide assumes you have built your project, and have an executable in the `bin/` folder on a UNIX environment.

####Run executable

To have the executable's Broker libraries communicate with the robot, you must supply the robot's IP address at runtime.
Simply run the executable like this

```
./nameofexecutable --pip IP_OF_ROBOT
```

Without the `--pip` flag, the broker attempts to connect to the default IP of `127.0.0.1`, which would ideally be correct if running the program on the robot itself, or running a local testing environment. Such an environment is started by locating the `libnaoqi` executable in your sdk, and running it on a desired port by passing a `--pport` flag with it.

###Next steps

Now that you're retreiving data from the robot, you can cout it to the console to see a raw string of bytes (in this case if it's image data being requested). To access this data from elsewhere, you can also stream it directly to a file with the command below:

```
./avcapture --pip 192.168.10.130 >> ~/NAME_OF_DESTINATION_FILE
```
