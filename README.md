## 检测器数据采集系统-嵌入式软件
![image](https://github.com/sunduoze/detector_das/assets/10105111/74917ee7-99c0-4ca6-aa97-fd72159780ac)

该嵌入式软件是在VSCODE下采用Platform IO对ESP32进行开发，由于其可以选择Arduino模式编程，并且有丰富的库可以方便调用，可大幅提高开发效率。实现了Wi-Fi的Client端收发数据、通过螺旋编码按键来控制OLED菜单切换，采集数据的校准、数值及曲线显示等功能。
整体嵌入式软件基于ESP32平台下，运行Free RTOS系统来方便开发。主要包含了3层，应用层主要包含用于实现Wi-Fi client端数据的收发模块和OLED的交互模块；中间层包含了U8g2图形库、Wi-Fi Client库等来方便应用层与底层（LL、HAL、驱动层）交互；底层包含了AD7606C、AD5272、FUSB302等驱动库，I2C、SPI、GPIO等HAL库。
