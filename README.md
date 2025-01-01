# Embodied-Intelligence-SCARA
# 并联Scara机械臂的智能应用（硬件与底层控制部分）
#### [github:https://github.com/Lengxiaom0/SCARA](https://github.com/Lengxiaom0/SCARA/tree/master)

#### B站连接：【智能机械臂（脉塔电机应用）】https://www.bilibili.com/video/BV18ACKY7EwT?vd_source=c03debe08019b673d647dc4e8aaa8287

##### solidworks机械臂图纸：通过网盘分享的文件：Scara
链接: https://pan.baidu.com/s/1tgeA8zEO1uHGxUXIj4xtng?pwd=wtmc 提取码: wtmc

##### 电机底层控制代码：通过网盘分享的文件：1210.zip
链接: https://pan.baidu.com/s/1JB90Ae1iG4P93yj973Xq-A?pwd=wtmc 提取码: wtmc 



## 概述 
- ### 具身智能是指依靠物理实体通过与环境交互来实现智能增长的智能系统。目前，其形式包括智能机器人、自动驾驶等。2024年5月，ICRA 2024的最佳论文奖授予Open X-Embodiment，展示了机器人学习数据集和基于此训练的模型RT-X。
- ### 我们本次是基于Scara并联机械臂的智能应用，并联机械臂在平面内相较于其他机械臂更具备稳定性，并且移动更加平稳，适合于二维平面的移动，加之图像理解等多模态大模型即可具备智能交互等操作。


## Demo展示：
![alt text](sw.png)



## 控制板块

### 控制硬件介绍
- 底层控制硬件采用stm32f427igh6芯片，使用freertos系统将电机控制任务，信号接收任务，以及外部控制任务进行多任务编程，通过串口与电脑通信，实现底层控制。后期打算直接整合到ROS上，可以更方便大家使用。
- 所有单片机底层配置已用Cubemax配置完成，使用cubemax可以快速生成代码，并可以直接下载到stm32f427igh6芯片上。所有的代码都已经带有注释，方便大家学习，抛砖引玉，共同进步，欢迎参考与相互学习。
- 为了省事开发，大家可以使用Dji的A型开发板进行底层开发。

![alt text](cubemax.jpg)

上述文件，放到了控制底层配置与电机控制库与控制算法的文件夹当中，大家可以根据自己的需求进行修改。


### 控制系统设计

- **描述**:我们采用的是江苏脉塔智能科技有限公司的RMD-X系列高精度电机，电机型号为RMD-X6-P6-7-C-N，每一圈分为36000个刻度，可以达到高精度的控制。整体采用can驱动，UART通信，步进电机+丝杠构成末端装置
- 因为采用的是五连杆结构，因此我们要对五连杆结构进行运动学逆解 

- 公式拆分以及代码实现：

**如图所示** 
![alt text](五连杆解算.png)


- 其中A，E为机器人腿部控制的两个电机，θ1,θ4可以通过电机的编码器测得。五连杆控制任务主要关注机构末端C点位置，其位置用直角坐标表示为(Cx,Cy)，极坐标系用(L0,θ0)表示。


![alt text](image.png)

对公式(1)移项，并在等式两边进行平方有：

![alt text](image-0.png) 

将平方展开有：

![alt text](image-1.png) 

对公式(3)内部两个等式相加并移项有： 

![alt text](image-2.png) 

使用二倍角法对公式(4)进一步化简，已知： 

![alt text](image-3.png) 

推导得到极坐标为：

![alt text](image-4.png) 

- 示例代码（C语言）
```
void Kinematics(float postion_x,float postion_y)
{
	
	float alpha1,alpha2,beta1,beta2;
	uint16_t servoLeftfront,servoLeftRear,servoRightfront,servoRightRear;
	
	float a=2*postion_x*L1;
	float b=2*postion_y*L1;
	float c=postion_x*postion_x + postion_y*postion_y + L1*L1 -L2*L2;
	float d=2 * L4 * (postion_x - L5);
	float e=2 * L4 *  postion_y;
	float f=((postion_x-L5)*(postion_x-L5)+postion_y*postion_y+L4*L4-L3*L3);
	
	alpha1=2*atan((b+sqrt(a*a+b*b-c*c))/(a+c));
	alpha2=2*atan((b-sqrt(a*a+b*b-c*c))/(a+c));
	beta1=2*atan((e+sqrt(d*d+e*e-f*f))/(d+f));
	beta2=2*atan((e-sqrt(d*d+e*e-f*f))/(d+f));

	alpha1=(alpha1>=0)?alpha1:(alpha1+2*PI);
	alpha2=(alpha2>=0)?alpha2:(alpha2+2*PI);

	beta1=(beta1>=0)?beta1:(beta1+2*PI);
	beta2=(beta2>=0)?beta2:(beta2+2*PI);

if (postion_y>0)
{
	temp_left_a=alpha1;
	temp_right_b=beta2;
}

else
{
	temp_left_a=alpha2;
	temp_right_b=beta1;
}
}
```

## 智能交互板块设计 
- **描述**：我们采用多个模型进行串联架构，综合图像理解，文本理解，语音识别等综合起来进而得到的结果，与下位机进行交互。

- ### 具体实现流程： 
- 1. 在得到下位机传送的运行指令后，首先打开摄像头进行外界信息的读取，然后进行多模态模型（本模型采用华为闭源模型）的推理，并保存。
- 2. 然后进行语音识别，与机器进行对话下达指令，识别到的文本将会放入到LLM进行推理，并进行指令拆分，得到的指令经过微调，system以及prompt的固化从而输出坐标指令。

#### 本项目借鉴于  《Do AsICan, Not As I Say:Grounding Language in Robotic Affordances》


## 机械板块

### 机械结构设计
（下附solidworks机械结构设计图链接）

- **描述**：并联机械臂是一种多连杆机械臂系统，由多个机械臂自由度联结而成，其关节可以分别控制，可实现复杂的运动轨迹，具有高度的灵活性和精度。
- **图片**：![alt text](0baf9801b3491bcec1eda4f21d49824.png)

  *图：机械结构示意图*

---


