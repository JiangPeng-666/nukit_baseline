# nukit_baseline
The modification for nukit based on [EPSILON](https://github.com/HKUST-Aerial-Robotics/EPSILON)

可以编译生成动态链接库供Python3.9调用

## 环境依赖：

由于之后生成动态链接库时，boost对环境的要求较严格，建议使用conda新建环境，并指定Python版本。

``` 
conda create -n nuplan python=3.9.10
```



1. 安装相关package

```
sudo apt-get install libgoogle-glog-dev libdw-dev libopenblas-dev gfortran
pip install empy pygame
```

2. 安装OOQP
   1. 使用OOQP求解二次优化问题，请参考[这里](https://github.com/emgertz/OOQP/blob/master/INSTALL)完成安装，过程中需要链接一个名为`libma27.a`的头文件，可在群里下载`ma27-1.0.0.zip`文件解压缩，运行`install-sh`文件可以得到需要的头文件。
3. 安装Protobuf
   1. 使用protobuf进行参数设置，请参考[这里](https://github.com/protocolbuffers/protobuf/blob/main/src/README.md)完成安装。
4. 安装boost

## 安装注意

## 安装注意：


## 安装注意：
1. boost库需要

   ```
   conda install boost
   ```

2. 生成的动态链接库建议放到.py文件所在的文件夹下，或者也可以修改`~/.bashrc`文件中的PYTHONPATH，修改动态链接库路径

3. 代码中有获取当前工作路径的部分，使用时请注意

4. 在根目录的CMakeList中，需要修改`include_directories`中的路径，具体如下：

   ```
   include_directories(
       inc
       ${PROJECT_SOURCE_DIR}/include
       /usr/include
       /usr/include/opencv4
       ${EIGEN3_INCLUDE_DIR}
       /home/lain/anaconda3/envs/nuplan_modified/include/python3.9
       /home/lain/anaconda3/envs/nuplan_modified/include
   )
   ```

   其中，最后两行要修改为自己的conda环境中的路径。

5. 在solver.cc的第39行和/core/eudm_planner/src/eudm_planner/eudm_manager.cc的第18行中因为VSCode执行的时候与终端直接执行时的工作路径不同，所以这里全都修改成了硬编码的地址形式，注意修改。

6. 如果想让glog能正常输出，注意根据eudm_manager.cc中的路径对应新建文件夹，如果之前不存在路径所述的文件夹的话，glog不能自己创建文件夹，也就不能创建文件，会一直报错，不过在命令行是有输出的

   

​	
