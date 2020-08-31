# dodo-whole-body-balance
This project is for developping a control strategy in order to balance a robot against external disturbances. The robot is simulated in Mujoco.

## Setup

Install MuJoCo dependencies:
```shell
Install the g++ compiler:
sudo apt-get install build-essential

OpenGL utilities:
sudo apt-get install libgl1-mesa-dev
sudo apt-get install libglew-dev

Later, we will also use the Eigen library, for that install

Blas library:
sudo apt-get install libblas-dev

GUI for CMake:
sudo apt-get install cmake-curses-gui

Eigen library (version 3.3.7):
http://eigen.tuxfamily.org/index.php?title=Main_Page
unzip into a folder (ex. called "Dodo", where you place also the dodo-whole-body-balance and the osqp &osqp-eigen later ), then in the folder
mkdir build
cd build
ccmake ../

In the GUI hit configure (-c) repeatedly, then generate (-g)
Install the library, by:
make
sudo make install
```

Install MuJoCo:
```shell
Unzip the folder in "Dodo", in a terminal navigate to the folder, inside do:

cd build

Configure the project using ccmake:

ccmake ../

Hit -c -g in the GUI:

make 

Place your mjkey.txt inside the build folder. You can test using e.g.:

./simulate ../model/creature.xml
```

Download and install OSQP-Eigen: 

https://osqp.org/docs/get_started/sources.html#build-from-sources
```shell
In the "Dodo" folder do:

git clone --recursive https://github.com/oxfordcontrol/osqp

cd osqp

mkdir build

cd build

cmake -G "Unix Makefiles" ..

cmake --build .

cmake --build . --target install
```

Download and install OSQP-Eigen: 

https://github.com/robotology/osqp-eigen

https://robotology.github.io/osqp-eigen/doxygen/doc/html/index.html
```shell
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ../
make
[sudo] make install
```

If your CMakeLists.txt file does not contain:
```shell
find_package(OsqpEigen)
target_link_libraries(balancerWrench OsqpEigen::OsqpEigen)

Add it to the CMakeLists.txt file
```

Building your code again now and everytime you changed your code: 
```shell
cd dodo-whole-body-balance
cd build
make
```

Open MuJoCo in your IDE (i.e. qtcreator):
```shell
Open MuJoCo in your IDE of choice. In the following, we will use qtcreator. If you did not already obtain qcreator, install it using:

sudo apt-get install qtcreator 

You now can open the project by opening a terminal and typing:

qtcreator CMakeLists.txt 
```

Get to know MuJoCo
```shell
Please familiarize yourself with the XML and API reference: http://www.mujoco.org/book/index.html. You can, for example, recreate our double pendulum from the first tutorial. To understand MuJoCo's modeling language, please also take a look at the examples inside the model folder. Please also take a look at the main function in e.g. basic.cpp, where the simulation and rendering loop is located. You do not have to worry too much about the parts that are concerned with rendering using openGL. There will be an additional tutorial session for everybody that uses MuJoCo.
```
