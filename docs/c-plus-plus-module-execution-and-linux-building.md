Basic C++ Module Information and Build Process Using Linux
==========================================================

Until now, it has been assumed that modules could be completely written in C++ and executed solely by calling the generated executable from a built project. It was briefly known that in order to execute a program in C++ on the robot, it had to communicate via proxy with a robot connected to the same network. What has been found clarifies and corrects this misconception.

The SDK is intended to be used both in a Python and C++ environment, at least when focusing solely on the C++ SDK. To clarify, at least on a Linux machine, one would build a project with CMake, generating executables, and load these executables, now referred to as "modules" to the robot. This would be done by copying the module to the robot's filesystem, adding the path to such "module" to the robot's `init.d` file, and then having a python script on the Linux machine execute the newly loaded module by specifying the robot's IP address, and module name in such script.

###Source and guide information

A detailed explanation of this is found starting on page 155 of the book "Using NAO: Introduction To Interactive Humanoid Robots".

###Specification and examples

A technical specification of this entry will be added. For now, the above discussion is shared as a means to share and document newly found information.
