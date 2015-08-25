##Setting up the project to build from an IDE
http://doc.aldebaran.com/1-14/qibuild/guide/overview/building_a_project.html#using-an-ide

##Setting up QI Build
http://doc.aldebaran.com/qibuild/beginner/getting_started.html

##Setting up the worktree, toolchains and making sure projects run with QIBuild
http://doc.aldebaran.com/1-14/dev/cpp/tutos/using_qibuild.html#cpp-tutos-using-qibuild

##General installation guide
http://doc.aldebaran.com/1-14/dev/cpp/install_guide.html

##Creating a new project and the CMAKE.txt
http://doc.aldebaran.com/1-14/dev/cpp/tutos/create_module.html

##Working With the C++ SDK
http://doc.aldebaran.com/1-14/dev/cpp/index.html

##Creating a new QIBUILD Project
cd into the directory you want the project
$ qisrc create <name>
$ qibuild configure <name>
$ qibuild make <name>

On Windows, a .sln should have been generated for you in the build directory.

In any case, you can open the correct IDE by using:
$ qibuild open <name> //where name is the name of the project