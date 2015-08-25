naoqi-nao-development
=====================

Development journal for nao robots (with Dr. Flores). This project aims to learn and implement as much of the NAOqi SDK as possible.

###Downloads

- [Download the SDK](https://community.aldebaran.com/en/resources/software)

###Resources

- [NAOqi SDK](http://doc.aldebaran.com/2-1/ref/index.html)
- [NAOqi Python SDK](http://doc.aldebaran.com/2-1/dev/python/index.html)
- [NAOqi Java SDK](http://doc.aldebaran.com/2-1/dev/java/index_java.html)
- [Hello World (C++)](http://doc.aldebaran.com/2-1/dev/cpp/examples/core/helloworld/example.html)
- [Capturing audio and video](http://doc.aldebaran.com/2-1/dev/cpp/examples/audiovideocapture/audiovideocapture.html#cpp-examples-audiovideocapture)

### Modules list

The following, are `C++` modules that have been completed by the team so far.

- `hello`
- `face`
- `video` (work in progress)

### Setting Up Workspace

This repo comes with a few `C++` modules, `hello` and `face` are two of them. To get started, make sure you have `naoqi-sdk-2.1.2.17` installed, and your `qitoolchain` is configured to use it.

- Make sure you have `qibuild`, `qitoolchain`, and `qisrc` installed. Clone this repository, and `cd` into it.

Link your project to the `C++ SDK` by typing

```
qitoolchain create <give_toolchain_a_name_here> <path_to_c++_sdk_location>/toolchain.xml
```

Then, configure your workspace to use this new toolchain:

```
qibuild config --wizard
```

Follow the instructions that appear, and when prompted to, select your newly made toolchain.

- Then, link all dependencies to the modules by typing `qibuild configure <module_name>`
- Finally, build the module by typing `qibuild make <module_name>`
- If you wish to create a module of your own, type `qisrc create <name_of_new_module>` 

### Adding Modules

You must add these modules to your workspace before you can build and run them.

- To add modules (that should already be present in your current folder) to the `worktree`, type:
 
```
qisrc add <module_name>
```

### Building Modules

To build a specific module, make sure it's been added to your worktree. Then link its dependencies:

```
qibuild configure <module_name>
```

And run `make` on it:

```
qibuild make <module_name>
```

### Running Modules

To run a module, simply type:
```
qibuild run <module_name>
```

Please note that if the module is a remote module (pretty much all of them are), you will need to pass arguments to it when running it, specifying the **IP address** of the robot. To do so, run it by typing:

```
qibuild run <module_name> -- --pip <IP_ADDRESS>
``` 

**Make note of the double dashes** before the first argument. These *escape* the rest of the dashes before each flag.

###Project members

- Juan Vallejo
- Jake Hayhurst
- Amanda Lee
