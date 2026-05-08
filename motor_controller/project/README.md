电机启动方案
---

# 硬件平台  
- MCU: Artery AT32M416KBU7-4  
- 预驱+MOS: MP6540GU-Z
- 磁编: KTH7111
- CAN-FD: TCAN334GDCNR
- DC-DC: MP2456GJ-Z  


# 系统设计  
- TIM1  
    - Channel 1/2/3 PWM 输出，接预驱的 PWMA/PWMB/PWMC
    - Channel 4 作为 ADC 采样(相电流采样、温度采样)的触发源
- ADC2: 负责采集相电流、温度
    - SOA(PA7) -> ADC2 IN7
    - SOB(PB0) -> ADC2 IN8
    - SOC(PB2) -> ADC2 IN10
    - 温度(PA5) -> ADC2 IN5
- SPI2: 负责编码器数据采集
- ENA(PA12)/ENB(PA11)/ENC(PB10): 接预驱
- 预驱 nFAULT 接 MCU 的 PB3（后续考虑切换为 PA6, 可作为定时器刹车）
- 预驱 nSLEEP 接 MCU 的 PA15
