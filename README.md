# vins-mynteye-m100   
ros工作空间在wrappers/ros下面  

本项目利用dji-m100的机型，挂在了mynteye相机，机载电脑用了nvidia tx2    

对dji_sdk进行了顶层开发，实现了光流定点悬停和对给定的目标点和当前点进行位置控制。    

配好依赖，编译通过后可以使用ros下的startup.sh来运行程序。
