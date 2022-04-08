# RosStudy
1. Ros上传Git代码时只需要上传src目录的
2. Ros代码下拉时，需要在对应的工作目录下初始化工作空间，没有工作空间的先创建工作空间  
   <font color=red>
   mkdir -p 自定义空间名称/src   
   cd 自定义空间名称  
   catkin_make
   </font>
3. 运行时先在进工作空间，再启动devel目录下setup.bash(用source命令)


