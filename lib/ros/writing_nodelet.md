# Writing ros nodelet libraries   
This document will have step by step directions for writing a complete library for ros nodelets.

## What is a nodelet  
A nodelet is a class that can act as a rosnode. The main difference between the two implimentations is node is created through a main function and a nodelet calls an oninit() function.  

Because the nodelet is a class, it inherits some qualitites from the base nodelet() class. This makes it easier to manage threads for larger projects (like ours)  

## Library directories   
The best practice for creating a ros package with nodelet is the following architecture:   
```  
my_package_name <dir> 
- CMakeLists.txt  
- package.xml 
- include <dir>
    - my_package_name <dir> 
- launch <dir>
- plugins <dir>
- src <dir>
```
The directories above should be the following: 
### include/my_package_name  
This should have your .h header files  
### launch  
holds all .launch launch files for your package. See the roslaunch command for more info on how these works  
### plugins  
contains xml items that export your nodelets in the package.xml file. This makes it visible, so you can run `roslaunch my_package_name my_nodelet.launch`
### src  
source code. We use C++ becuase it's the best, but you could put python here as well 

## Writing the nodelet  
To write your nodelet, you need to have the bareminimum installed for ros, go online to see how to install it. Once you do that, do the following:  
### my_nodelet.h  
The header file only needs a few things to be able to run.  
1. You should always wrap your header files with macroguards. The best practice would be to use the name of the file in all caps followed by `_H`. You should also put it in a namespace for your package. So for a file called "my_nodelet.h" in the package "my_package" I would wrap the code with the following:  
```cpp 
#ifndef MY_NODELET_H 
#define MY_NODELET_H  
// includes here 
namespace my_package {
// all code here
}// my_package
#endif // MY_NODELET_H
```
This prevents from compiling the same class mulitple times.  

2. Next, you need the following includes:  
```cpp 
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
```  

3. next you define your class. You can call it whatever you would like, but it must inherit from nodelet, and you must override the onInit() function, like so:  
```cpp  
// must declare nodelet::Nodelet as public so this class is visible later
class my_nodelet : public nodelet::Nodelet {
public:
    void onInit() override;
};
```  
You can add any other functions or variables you want in your class, this is just the bare minimum.  

### my_nodelet.cpp  
There are a few more nodelet specific things needed in the .cpp files to run a nodelet.  
1. include the header file and setup the namespace:  
```cpp 
#include <my_package/my_nodelet.h>
namespace my_package {
    // all code here
}// namespace my_package
```  
2. write the override function for onInit in this file:  
```cpp 
void my_nodelet::onInit() {
    // code here
}
```

3. Setup the export macros. This is what the `#include <pluginlib/class_list_macros.h>` line is for, it makes this nodelet visible to ros externally to this project. To export it, add the following line in your cpp file:  
```cpp 
PLUGINLIB_EXPORT_CLASS(my_package::my_nodelet, nodelet::Nodelet);
``` 
This function uses the format `PLUGINLIB_EXPORT_CLASS(namespace::classname, baseclass). **NOTE**: The base class is Nodelet. 

After completing these three steps, your nodelet is complete wit hthe bareminimum needed to run.  

## Compilation  
To compile, there are a few things needed: A CmakeLists.txt and a package.xml. 
### CMakeLists.txt  
This is used to create the binaries and define the library. There are a lot of things needed to run this, so just checkout the file used in the behaviors directory.  
### package.xml  
This is a ros specific item which acts a manifest of the package. It will be here that you define all dependencies and nodelets to export. During compilation, we will only worry about dependencies. Again there is an exmple of this in behaviors. But the bareminimum needed in a package.xml file is the following: 
```xml 
<?xml version="1.0"?>
<package format="2">
    <!-- required tags -->
    <name>statemachine</name>
    <version>1.0.0</version>
    <description>
        This package holds various statemachines 
    </description>
    <maintainer email="ntrinite@students.kennesaw.edu">Noah Trinite</maintainer>
    <license>This was just required for compilation</license>

    <!-- dependencies -->
    <buildtool_depend>catkin</buildtool_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>roslaunch</exec_depend>

    <build_depend>message_generation</build_depend>
    <build_depend>nodelet</build_depend>

    <exec_depend>message_runtime</exec_depend>
    <exec_depend>rospy</exec_depend>
    <exec_depend>nodelet</exec_depend>

    <!-- export nodelets -->
    <export>

    </export>
</package>
```
**NOTE**: add all dependencies needed to run the package in this file. The following are the tags used:  
`<exec_depend>`  
This is used to define dependencie used during execution. This will include libraries included in cpp files.  
`<build_depend>`  
This is used to define dependencies used during compile time. This includes anything used to create messages and the like.  

### compiling  
after creating the CMakeLists.txt and package.xml files, you are ready to compile your code. You can do so by doing the following:  
```BASH 
cd /path/to/my_package  
mkdir build && cd build 
cmake .. -GNinja # if you don't have ninja installed, you can remove the flag  
ninja  # if you don't have ninja, run 'make' instead  
```  
This will compile your nodelet  

## setting up for runtime  
There are two files used during runtime: my_nodelet.launch and my_nodelet_plugin.xml.  
### my_nodelet_plugin.xml  
This file should live in the plugins directory. The bareminimum needed in this file is:  
```xml 
<library path="lib/libmy_package">
  <class name="my_pacakge/my_nodelet" type="my_package::my_nodelet" base_class_type="nodelet::Nodelet">
  <description>
  This is my nodelet.
  </description>
  </class>
</library>
```
**NOTE:** The `type` definition should match the defined export type in the function `PLUGINLIB_EXPORT_CLASS()` in the CPP file.  

Next, you must add this file to the list of exports in the manifest. There was already the `<export>` flag in the package.xml minimum required above. So add the following:  
```xml  
<export>
    <nodelet plugin="${prefix}/plugins/my_nodelet_plugin.xml"/>
</export>
```  

### my_nodelet.launch  
This is the file used during launch time, and should live in the launch directory. Though the suffix of the file is .launch, these are still xml files. The following is an example of a launch file for this nodelet:  
```xml
<launch>
<node pkg="nodelet" 
    type="nodelet" name="myNodelet"
    args="standalone my_package/my_nodelet"
    respawn="false" output="screen"/>
</launch>
```
**NOTE:** The args definition must match the `name` definition from the `my_nodelet_plugin.xml` file.  

### Running  
Now you have everything needed to run the nodelet. Follow the following steps to run:  
1. recompile:  
```BASH  
cd /path/to/my_package/build  
ninja # run make if that's what you did before  
```  

2. Source the setup file.  
At this point your code is ready, but not visible. You can check this by running `rospack list`. You will see all available ros pacakges, and you will notice yours is not one of them. To add it run the following:  
```BASH  
source /path/to/my_package/build/devel/setup.sh  
rospack list
```  
Now you should see your pacakge on the list. This setup.sh file is generated by catkin during development.  

3. run roslaunch. To do this, use this command:  
```BASH  
roslaunch my_package my_nodelet.launch
```  
You will see roscore start, and the list of all nodelets that are going to be used. 

## Happy Dance  
You've ran a nodelet know. Go celebrate. Or don't IDK follow your heart.