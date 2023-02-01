# HD-CCSOM
HD-CCSOM: Hierarchical and Dense Collaborative Continuous Semantic Occupancy Mapping through Label Diffusion

<img src="https://github.com/BIT-DYN/HD-CCSOM/blob/master/image/robot1.png" width="250"><img src="https://github.com/BIT-DYN/HD-CCSOM/blob/master/image/robot2.png" width="250"><img src="https://github.com/BIT-DYN/HD-CCSOM/blob/master/image/robot12.png" width="250">

This is a novel collaborative continuous semantic occupancy Mapping  algorithm, opening up a new field of mapping.

## Getting Started

### Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/BIT-DYN/HD-CCSOM
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Building using Intel C++ compiler (optional for better speed performance)
```bash
catkin_ws$ source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64
catkin_ws$ catkin_make -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Downloading the Dataset

Download the demo dataset and place them in directory  "catkin_ws/src/HD-CCSOM/data/*.bag".

[Baidu Cloud Disk](https://pan.baidu.com/s/1MNne-sYKJ7nwjxB0PjZiTg?pwd=a6bx )

[Google Drive](https://drive.google.com/drive/folders/1QqWbfXb1OFcQoGQFMKcS6NNP_UApdNt5?usp=sharing) 

### Running the Demo
First, each robot builds a local semantic map.

```bash
catkin_ws$ roslaunch hd_ccsom multi_gazebo.launch
```

Then, you can run the command through the new terminal to complete the map sharing and fusion.


```bash
rosservice call /robot2/manual_triger
```

Each robot stores a local map and a global map.

<img src="https://github.com/BIT-DYN/HD-CCSOM/blob/master/image/dyn7_1.png" width="250"><img src="https://github.com/BIT-DYN/HD-CCSOM/blob/master/image/dyn7_2.png" width="250"><img src="https://github.com/BIT-DYN/HD-CCSOM/blob/master/image/dyn7_12.png" width="250">

## Citation

```
@inproceedings{deng2022hd,
  title={Hd-ccsom: Hierarchical and dense collaborative continuous semantic occupancy mapping through label diffusion},
  author={Deng, Yinan and Wang, Meiling and Yang, Yi and Yue, Yufeng},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={2417--2422},
  year={2022},
  organization={IEEE}
}
```
