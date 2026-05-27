## 依赖
- livox_ros_driver
## 编译
```
source livox_ros_driver所在工作空间/devel/setup.bash
```

```
cd ./ros_app/build
```

```
cmake ..
```

```
make -j8
```
## 运行
```
source ./devel/setup.bash
```

```
roslaunch frlio frlio_ros_online.launch
```# my_frlio
