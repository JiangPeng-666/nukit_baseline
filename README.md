# nukit_baseline
The modification for nukit based on EPSILON

Tips: 如果想让glog能正常输出，注意修改eudm_manager.cc中的路径设置，个人亲测，似乎如果之前不存在路径所述的文件夹的话，glog不能自己创建文件夹，也就不能创建文件，会一直报错。


3.16版本
	目前可以实现behavior planner的正常输出，但是motion planner的输出还不能做到，现有问题是behavior planner的set_ego_hahavior()执行后，ego_behavior_变量值不能保存下来供后续使用，还在解决这个问题中。
