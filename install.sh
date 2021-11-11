echo Installing ...

cd $HOME && git clone https://github.com/iamarkaj/AwesomeSLAM.git
ln -s $HOME/AwesomeSLAM/external/* $HOME/catkin_ws/src/
ln -s $HOME/AwesomeSLAM/awesome_slam $HOME/catkin_ws/src/
roscd && rosdep install --from-paths src --ignore-src -r -y
catkin_make

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/AwesomeSLAM/awesome_slam/models" >> ~/.bashrc
source ~/.bashrc

echo Complete.