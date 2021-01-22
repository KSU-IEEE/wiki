# Arduino Libraries  
This document is meant to serve as a basic howto for adding custom libraries to be able to be used/included into arduino projects. This should be used for developing something you wish to unit test, or for embodying a class that will be used in your future projects. 

### Installation
--------------------------------------------------------------------------------

**NOTE**:There isn't any actual installating here, it's mostly moving files around and ensuring your folders are formated correctly.  

The first step is to move your library into the correct location: Arduino/lib/targets/libraries   
If you are grabbing a repository from github, you can clone it directly to this directory, other wise you will have to move it from their.  


### Building
--------------------------------------------------------------------------------

After this library is "installed", you just have to start the Arduino application.
You may see a few warning messages as it's built. This one method to debug the code, as any errors within your code will be shown here.

To use this library in a sketch, go to the Sketch | Import Library menu and
select Test.  This will add a corresponding line to the top of your sketch:
`#include <my_library.h>`
To stop using this library, delete that line from your sketch. 

You can also just type the include, if you aren't below that.

Geeky information:
After a successful build of this library, a new file named "mecanum_drive.o" will appear
in "Arduino/lib/targets/libraries/mecanum_drive". This file is the built/compiled library
code.