Setting up / using the SDK for C++
==================================

Before you can begin to develop for the NaoQi platform, you must understand how the SDK works, and how it is meant to be used. The naoqi SDK does not copy its libraries to your path, like you would expect. Instead, it insists you keep it in a convenient location in your computer, and link to it using naoqi's build system (qibuild).

**This guide assumes you are working in a Unix environment**

####Installing the Build System

- Before anything can begin to be done with the SDK, you must install `qibuild` naoqi's custom build system based on CMake. If you have python and/or the python package manager `pip`, simply execute the line below on a terminal.

```sh
pip install qibuild
```

Once you have the qi build system installed, set up a new project under it by typing

```sh
qibuild init
```

Next, configure the build system to assign an existing build system to its configuration, and to tell it which IDE you are going to be using.

```sh
qibuild config --wizard
```

A full guide to this build system is found [here](http://doc.aldebaran.com/qibuild/beginner/qibuild/aldebaran.html#qibuild-using-aldebaran-packages).

####Downloading and working with the C++ SDK

- To download the C++ SDK from the Aldebaran website, you must go to one of the robot's IP address on your browser and link your Aldebaran account there. Then, go to the Aldebaran community website, click on products, then software, and you should now be able to see all of the development tools offered for development on the robots.

[https://community.aldebaran.com/en/resources/software/212-4-c-naoqi-sdk](https://community.aldebaran.com/en/resources/software/212-4-c-naoqi-sdk)

- As mentioned in the introduction to this article, the Aldebaran C++ SDK does not have libraries accessible from a PATH. The SDK guide suggests that projects be placed within the SDK folder, or somewhere relative to it, and then linked to a `toolchain.xml` file in the root of that folder.
- To link a project using this toolchain, once the project has been initialized using the `qibuild` system as described in the previous steps, follow the commands below:

```sh
qitoolchain create naoqi-sdk path/to/toolchain.xml
```

- the `path/to/toolchain.xml` part of the command simply refers to the relative path to the `toolchain.xml` file found at the very root of the downloaded C++ SDK folder.

It is recommended that you develop using an SDK for conveniency when building your project; however it is possible to tell the `qibuild` configuration wizard that you wish to develop and compile your code by hand.
