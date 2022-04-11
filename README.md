# nukit_baseline
The modification for nukit based on EPSILON


## 安装注意：
1. include路径中需要改成自己的conda环境的路径，且注意Python版本为3.9
2. boost库需要conda install boost
3. 需要修改动态链接库路径
4. 生成的动态链接库可供Python调用，但是注意修改Python的sys.path变量，可在bashrc中修改PYTHONPATH实现
5. 代码中有获取当前工作路径的部分，使用时请注意


## Tips: 
如果想让glog能正常输出，注意修改eudm_manager.cc中的路径设置，个人亲测，似乎如果之前不存在路径所述的文件夹的话，glog不能自己创建文件夹，也就不能创建文件，会一直报错。

	
