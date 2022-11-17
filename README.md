# Begineer Tutorials - Publisher-Subscriber

# Dependencies
<ul>
  <li>Ubuntu 20.04+</li>
  <li>ROS2 Foxy</li>
</ul>

# Compile and run instructions
```
    cd "your_colcon_workspace"/src
    git clone https://github.com/sj0897/beginner_tutorials.git
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
```

After you've compiled and sourced your workspace

## Running publisher
```
    ros2 run beginner_tutorials talker
```

## Running Subscriber
```
    ros2 run beginner_tutorials listener
```

# Service - Logging - Launch Files

## Compile and run instructions
```
  git checkout Week10_HW
```
Follow same compile and build instructions as above.

## Using the launch file with argument
```
  ros2 launch beginner_tutorials week_10.yaml count:=20
```

## Results

### RQT_Console
![image](https://github.com/sj0897/beginner_tutorials/blob/Week10_HW/Results/rqt.png)

