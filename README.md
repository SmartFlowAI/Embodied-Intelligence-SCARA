# Embodied-Intelligence-SCARA
# ����Scara��е�۵�����Ӧ�ã�Ӳ����ײ���Ʋ��֣�
#### [github:https://github.com/Lengxiaom0/SCARA](https://github.com/Lengxiaom0/SCARA/tree/master)

#### Bվ���ӣ������ܻ�е�ۣ��������Ӧ�ã���https://www.bilibili.com/video/BV18ACKY7EwT?vd_source=c03debe08019b673d647dc4e8aaa8287

##### solidworks��е��ͼֽ��ͨ�����̷�����ļ���Scara
����: https://pan.baidu.com/s/1tgeA8zEO1uHGxUXIj4xtng?pwd=wtmc ��ȡ��: wtmc

##### ����ײ���ƴ��룺ͨ�����̷�����ļ���1210.zip
����: https://pan.baidu.com/s/1JB90Ae1iG4P93yj973Xq-A?pwd=wtmc ��ȡ��: wtmc 



## ���� 
- ### ����������ָ��������ʵ��ͨ���뻷��������ʵ����������������ϵͳ��Ŀǰ������ʽ�������ܻ����ˡ��Զ���ʻ�ȡ�2024��5�£�ICRA 2024��������Ľ�����Open X-Embodiment��չʾ�˻�����ѧϰ���ݼ��ͻ��ڴ�ѵ����ģ��RT-X��
- ### ���Ǳ����ǻ���Scara������е�۵�����Ӧ�ã�������е����ƽ���������������е�۸��߱��ȶ��ԣ������ƶ�����ƽ�ȣ��ʺ��ڶ�άƽ����ƶ�����֮ͼ�����ȶ�ģ̬��ģ�ͼ��ɾ߱����ܽ����Ȳ�����


## Demoչʾ��
![alt text](sw.png)



## ���ư��

### ����Ӳ������
- �ײ����Ӳ������stm32f427igh6оƬ��ʹ��freertosϵͳ��������������źŽ��������Լ��ⲿ����������ж������̣�ͨ�����������ͨ�ţ�ʵ�ֵײ���ơ����ڴ���ֱ�����ϵ�ROS�ϣ����Ը�������ʹ�á�
- ���е�Ƭ���ײ���������Cubemax������ɣ�ʹ��cubemax���Կ������ɴ��룬������ֱ�����ص�stm32f427igh6оƬ�ϡ����еĴ��붼�Ѿ�����ע�ͣ�������ѧϰ����ש���񣬹�ͬ��������ӭ�ο����໥ѧϰ��
- Ϊ��ʡ�¿�������ҿ���ʹ��Dji��A�Ϳ�������еײ㿪����

![alt text](cubemax.jpg)

�����ļ����ŵ��˿��Ƶײ������������ƿ�������㷨���ļ��е��У���ҿ��Ը����Լ�����������޸ġ�


### ����ϵͳ���

- **����**:���ǲ��õ��ǽ����������ܿƼ����޹�˾��RMD-Xϵ�и߾��ȵ��������ͺ�ΪRMD-X6-P6-7-C-N��ÿһȦ��Ϊ36000���̶ȣ����Դﵽ�߾��ȵĿ��ơ��������can������UARTͨ�ţ��������+˿�ܹ���ĩ��װ��
- ��Ϊ���õ��������˽ṹ���������Ҫ�������˽ṹ�����˶�ѧ��� 

- ��ʽ����Լ�����ʵ�֣�

**��ͼ��ʾ** 
![alt text](�����˽���.png)


- ����A��EΪ�������Ȳ����Ƶ������������1,��4����ͨ������ı�������á������˿���������Ҫ��ע����ĩ��C��λ�ã���λ����ֱ�������ʾΪ(Cx,Cy)��������ϵ��(L0,��0)��ʾ��


![alt text](image.png)

�Թ�ʽ(1)������ڵ�ʽ���߽���ƽ���У�

![alt text](image-0.png) 

��ƽ��չ���У�

![alt text](image-1.png) 

�Թ�ʽ(3)�ڲ�������ʽ��Ӳ������У� 

![alt text](image-2.png) 

ʹ�ö����Ƿ��Թ�ʽ(4)��һ��������֪�� 

![alt text](image-3.png) 

�Ƶ��õ�������Ϊ��

![alt text](image-4.png) 

- ʾ�����루C���ԣ�
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

## ���ܽ��������� 
- **����**�����ǲ��ö��ģ�ͽ��д����ܹ����ۺ�ͼ����⣬�ı���⣬����ʶ����ۺ����������õ��Ľ��������λ�����н�����

- ### ����ʵ�����̣� 
- 1. �ڵõ���λ�����͵�����ָ������ȴ�����ͷ���������Ϣ�Ķ�ȡ��Ȼ����ж�ģ̬ģ�ͣ���ģ�Ͳ��û�Ϊ��Դģ�ͣ������������档
- 2. Ȼ���������ʶ����������жԻ��´�ָ�ʶ�𵽵��ı�������뵽LLM��������������ָ���֣��õ���ָ���΢����system�Լ�prompt�Ĺ̻��Ӷ��������ָ�

#### ����Ŀ�����  ��Do AsICan, Not As I Say:Grounding Language in Robotic Affordances��


## ��е���

### ��е�ṹ���
���¸�solidworks��е�ṹ���ͼ���ӣ�

- **����**��������е����һ�ֶ����˻�е��ϵͳ���ɶ����е�����ɶ�������ɣ���ؽڿ��Էֱ���ƣ���ʵ�ָ��ӵ��˶��켣�����и߶ȵ�����Ժ;��ȡ�
- **ͼƬ**��![alt text](0baf9801b3491bcec1eda4f21d49824.png)

  *ͼ����е�ṹʾ��ͼ*

---


