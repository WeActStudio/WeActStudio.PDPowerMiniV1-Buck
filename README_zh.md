* [English version](./README.md)
# WeActStudio.PDPowerMiniV1-Buck
![display](Images/zh/1.png)

|规格||
|:-:|:-:|
|输入|DC 24V/PD 20V/QC 12V,3A|
|输出|1-20V/0.05-3A + 5V/0.3A|
|输出设置精度|±0.01V/±2mA|
|输出显示精度|±0.02V/±3mA|
|主输出电压纹波|<±25mV|
|输出保护|过流保护,短路保护|
> 注意: 主输出2A电流可长时间运行,3A电流需要加强散热

## 使用说明
![display](Images/zh/2.png)
### 设置项说明
1. `Brightness` 亮度： 0-100%设置，0-10%为超低亮度。
2. `Reverse Dis` 翻转显示：0为正常，1为翻转180°，重新上电生效。
3. `OCP Enable` 过流保护：0为不使能，1为使能，电流超过设置值200ms左右关断输出。
4. `OC Discharge` 过流放电：0为不使能，1为使能。
5. `AUTO OUT` 自动输出：0为不使能，1为使能，上电后自动使能输出。
6. `Filter.Win` 输出滤波窗口设置：1-200，默认50。
7. `Factory Reset` 恢复出厂设置：设置为1后退出设置界面恢复出厂设置(不会清除校准值)。
8. `Offset Enable` 校准使能：0为不使能，1为使能，出厂为1，已校准。
### 测试数据
![display](Images/zh/3.png)
### 上位机说明
![display](Images/zh/4.png)

```
/*---------------------------------------
- WeAct Studio Official Link
- taobao: weactstudio.taobao.com
- aliexpress: weactstudio.aliexpress.com
- github: github.com/WeActStudio
- gitee: gitee.com/WeAct-TC
- blog: www.weact-tc.cn
---------------------------------------*/
```