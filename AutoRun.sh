echo 'Installing dependencies via apt. Please enter your password'
sudo apt-get update -y
sudo apt-get -qy install git build-essential cmake libboost-dev libboost-filesystem-dev libboost-serialization-dev libeigen3-dev libglew-dev libglfw3-dev
sudo apt-get install -y libpng-dev
cmake .
make
./lyra ./conf/conf.json