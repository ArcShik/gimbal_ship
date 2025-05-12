使用的是cubemax6.2.1生成的Freertos和LWIP

若修改cubemax更新代码需要在代码中做如下修改：

```c
#define LAN8742A_PHY_ADDRESS           0U     //原本默认为1
#define configTOTAL_HEAP_SIZE                    ((size_t)32020)   //防止heap不够用
```

#因代码部分文件存在本地路径依赖，下载后需删除CMakeLists文件和cmake-build-debug后进入CLion待编译器重新生成，再在Clion生成的CMakeLists文件基础上添加部分内容，添加内容如下：

```python
#将22行到24行取消注释并在后续添加下面的代码

include_directories(
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
)
link_directories(src/algorithm/arm_math)
link_directories(Middlewares/Third_Party/FreeRTOS/Source/tasks.c)
link_directories(Middlewares/Third_Party/FreeRTOS/Source/include/task.h)

#  include_directories()括号内修改为如下代码
        Core/Inc
        src
        src/algorithm
        src/algorithm/arm_math
        src/algorithm/kalman_filter
        src/algorithm/pid
        src/algorithm/ramp
        src/algorithm/QuaternionEKF
        src/algorithm/user_lib
        src/modules
        src/modules/BMI088
        src/modules/can
        src/modules/dwt
        src/modules/ipc
        src/modules/leg
        src/modules/motor
        src/modules/motor/DJI_motor
        src/modules/motor/LK_motor
        src/modules/motor/HT_motor
        src/modules/pwm
        src/modules/rc/sbus
        src/task
        src/task/ins
        src/task/motor
        src/task/cmd
        src/task/chassis
        src/task/trans
        src/task/gimbal
        src/task/shoot
        LWIP/App
        LWIP/Target 
        Middlewares/Third_Party/LwIP/src/include 
        Middlewares/Third_Party/LwIP/system 
        Drivers/STM32F4xx_HAL_Driver/Inc 
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy 
        Middlewares/Third_Party/FreeRTOS/Source/include 
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS 
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F 
        Middlewares/Third_Party/LwIP/src/include/netif/ppp Drivers/CMSIS/Device/ST/STM32F4xx/Include 
        Middlewares/Third_Party/LwIP/src/include/lwip 
        Middlewares/Third_Party/LwIP/src/include/lwip/apps 
        Middlewares/Third_Party/LwIP/src/include/lwip/priv 
        Middlewares/Third_Party/LwIP/src/include/lwip/prot 
        Middlewares/Third_Party/LwIP/src/include/netif 
        Middlewares/Third_Party/LwIP/src/include/compat/posix 
        Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa 
        Middlewares/Third_Party/LwIP/src/include/compat/posix/net 
        Middlewares/Third_Party/LwIP/src/include/compat/posix/sys 
        Middlewares/Third_Party/LwIP/src/include/compat/stdc 
        Middlewares/Third_Party/LwIP/system/arch Drivers/CMSIS/Include
 #将file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "LWIP/*.*" "Drivers/*.*" )修改为file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "LWIP/*.*" "Drivers/*.*" "src/*.*")

#在最后一行添加如下代码
target_link_libraries(${PROJECT_NAME}.elf libarm_cortexM4lf_math.a)

```

