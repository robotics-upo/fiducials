#! /bin/bash
#####################################################################################################################################################################################
read -p "Installing g2o. Press a ENTER to contine. (CTRL+C) to cancel"

echo "Installing dependencies"
sudo apt-get install cmake libeigen3-dev
sudo apt-get install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev libcholmod3.0.6 liblapack-dev libblas-dev

echo "Downloading and installing g2o in temp"

cd /tmp/
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake .. -DG2O_BUILD_EXAMPLES=yes -DBUILD_WITH_MARCH_NATIVE=ON ..
make -j4
sudo make install

