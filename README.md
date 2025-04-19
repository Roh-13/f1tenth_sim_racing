Robot_localization

## Demonstration

```bash
xhost local:root
docker run --name autodrive_f1tenth_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:2024-cdc-practice
./AutoDRIVE\ Simulator.x86_64
```

```bash
xhost local:root
docker run --name autodrive_f1tenth_api --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_api:2024-cdc-practice
docker start autodrive_f1tenth_api
docker exec -it autodrive_f1tenth_api bash
#run in terminal 1 and then connect (ONE OF THEM)
#gui mode
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
# OR headless mode
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py4
colcon build && source install/setup.bash && source /opt/ros/humble/setup.bash
ros2 run f1tenth_stanley_controller stanley_controller
#OR
ros2 launch f1tenth_stanley_controller controller_launch.py 

```
