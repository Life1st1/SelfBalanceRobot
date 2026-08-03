# SelfBalanceRobot
1. Init workspace
Please refer to the Zephyr documents: 
https://docs.zephyrproject.org/latest/develop/getting_started/index.html

2. Build command for esp32c3
cd zephyrproject/
mkdir app
cp <path of SelfBalanceRobot> ./
cd SelfBalanceRobot/
west build -p always -b esp32c3_supermini --extra-dtc-overlay board/esp32_c3_app.overlay ./
