### Troubleshooting

If IDE don't start after crash or not correct finish.


```bash
ps -elf | grep stm
0 S user1      35455    4973 17  80   0 - 2278192 futex_ 19:17 pts/1  00:01:00 /opt/st/stm32cubeide_1.19.0_2//plugins/com.st.stm32cube.ide.jre.linux64_3.4.0.202409160955/jre/bin/java -Dosgi.requiredJavaVersion=1.8 -Dosgi.instance.area.default=@user.home/STM32CubeIDE/workspace_1.19.0 -Declipse.buildId=Version 1.19.0 -DproductMaturityGrade=mm -Djdk.http.auth.tunneling.disabledSchemes= -XX:+UseG1GC -XX:+UseStringDeduplication -Xms256m -Xmx1024m -jar /opt/st/stm32cubeide_1.19.0_2//plugins/org.eclipse.equinox.launcher_1.6.900.v20240613-2009.jar -os linux -ws gtk -arch x86_64 -showsplash -launcher /opt/st/stm32cubeide_1.19.0_2/stm32cubeide -name Stm32cubeide --launcher.library /opt/st/stm32cubeide_1.19.0_2//plugins/org.eclipse.equinox.launcher.gtk.linux.x86_64_1.2.1100.v20240722-2106/eclipse_11904.so -startup /opt/st/stm32cubeide_1.19.0_2//plugins/org.eclipse.equinox.launcher_1.6.900.v20240613-2009.jar --launcher.overrideVmargs -exitdata 1803e -clean -vm /opt/st/stm32cubeide_1.19.0_2//plugins/com.st.stm32cube.ide.jre.linux64_3.4.0.202409160955/jre/bin/java -vmargs -Dosgi.requiredJavaVersion=1.8 -Dosgi.instance.area.default=@user.home/STM32CubeIDE/workspace_1.19.0 -Declipse.buildId=Version 1.19.0 -DproductMaturityGrade=mm -Djdk.http.auth.tunneling.disabledSchemes= -XX:+UseG1GC -XX:+UseStringDeduplication -Xms256m -Xmx1024m -jar /opt/st/stm32cubeide_1.19.0_2//plugins/org.eclipse.equinox.launcher_1.6.900.v20240613-2009.jar
0 S user1      38532   18297  0  80   0 -  2771 pipe_w 19:23 pts/1    00:00:00 grep --color=auto stm
kill -9 35455
rm -rf ~/.stmcufinder
rm -rf ~/.stm32cubeide
rm STM32CubeIDE/workspace_1.19.0/.metadata/.lock
mv ~/STM32CubeIDE/workspace_1.19.0/.metadata ~/STM32CubeIDE/workspace_1.19.0/.metadata.bak
``` 

Then start IDE

Then import projects from workspace:

File->Import->Projects From Folder Or Archive -> Import source (choose directory)

===============================================================================================
### Debug with OpenOCD

Download OpenOCD archive
https://release-assets.githubusercontent.com/github-production-release-asset/189087873/f3036860-3fe2-4d97-a9b0-128002d0f08c?sp=r&sv=2018-11-09&sr=b&spr=https&se=2025-10-09T15%3A50%3A22Z&rscd=attachment%3B+filename%3Dxpack-openocd-0.12.0-7-linux-x64.tar.gz&rsct=application%2Foctet-stream&skoid=96c2d410-5711-43a1-aedd-ab1947aa7ab0&sktid=398a6654-997b-47e9-b12b-9515b896b4de&skt=2025-10-09T14%3A49%3A23Z&ske=2025-10-09T15%3A50%3A22Z&sks=b&skv=2018-11-09&sig=hDYZbG76aECbWqFj0apDXqP4pEQFW7rpxfi4z5rxMLA%3D&jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmVsZWFzZS1hc3NldHMuZ2l0aHVidXNlcmNvbnRlbnQuY29tIiwia2V5Ijoia2V5MSIsImV4cCI6MTc2MDAyMTgwOSwibmJmIjoxNzYwMDIxNTA5LCJwYXRoIjoicmVsZWFzZWFzc2V0cHJvZHVjdGlvbi5ibG9iLmNvcmUud2luZG93cy5uZXQifQ.3zY6tfm4PaBMYF51MLMjQztauNHBvmckab8lZ0HhCAg&response-content-disposition=attachment%3B%20filename%3Dxpack-openocd-0.12.0-7-linux-x64.tar.gz&response-content-type=application%2Foctet-stream


Unpack to home folder

~/STM32Toolchain

cd ~/STM32Toolchain/openocd/scripts


#### Запуск сервера
```bash
../../bin/openocd -f interface/stlink.cfg -f board/stm32f103c8_blue_pill.cfg -c "transport select swd"

xPack Open On-Chip Debugger 0.12.0+dev-02228-ge5888bda3-dirty (2025-10-04-22:42)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Warn : DEPRECATED: auto-selecting transport "swd (dapdirect)". Use 'transport select swd' to suppress this message.
Warn : Transport "swd" was already selected
Info : STLINK V2J46S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.224463
Info : Unable to match requested speed 1000 kHz, using 950 kHz
Info : Unable to match requested speed 1000 kHz, using 950 kHz
Info : clock speed 950 kHz
Info : SWD DPIDR 0x1ba01477
Info : [stm32f1x.cpu] Cortex-M3 r1p1 processor detected
Info : [stm32f1x.cpu] target has 6 breakpoints, 4 watchpoints
Info : [stm32f1x.cpu] Examination succeed
Info : [stm32f1x.cpu] starting gdb server on 3333
Info : Listening on port 3333 for gdb connections
semihosting is enabled
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections

```

#### Запуск отлаживаемого приложения

```bash
cd ~/STM32CubeIDE/workspace_1.19.0/test1/Debug
arm-none-eabi-gdb test1.elf

GNU gdb (GNU Arm Embedded Toolchain 10.3-2021.10) 10.2.90.20210621-git
Reading symbols from test1.elf...
(gdb)

# Подключись к OpenOCD
target remote localhost:3333

# Загрузить прошивку в микроконтроллер
load

# Управлять программой
    monitor reset halt — сбросить и остановить МК

    b main — поставить брейкпоинт в main()

    c — продолжить выполнение

    s — шагнуть внутрь функции

    n — следующий шаг

    info reg — показать регистры

    p myVar — вывести значение переменной myVar



    q — выйти из GDB

    Ctrl+C в окне с OpenOCD — завершить отладчик

``` 

#### Поддержка семихостинга

```bash
 ../../bin/openocd -f interface/stlink.cfg -f board/stm32f103c8_blue_pill.cfg -c "transport select swd" -c "init" -c "arm semihosting enable"
 
```

тут описано как настроить Debug в STM32CubeIDE используя внешний Open OCD сервер
https://www.galliumio.com/1531/debugging-with-stm32cubeide/

Another way to launch a debug session is to start OpenOCD as an external tool. In order to do that, we need to first setup OpenOCD by clicking the down arrow next to the External Tools icon to pull down a menu. Click the “External Tools Configurations…” menu item.
Create a new tool configuration under Program on the left side panel. Name this configuration OpenOCD STM32L4 Discovery. The key configurations are the path to the openocd.exe, path to the scripts folder and the specific board cfg file for our platform. It is assumed that you have OpenOCD installed previously. In the example below, OpenOCD has been installed under C:\Users\Public\Documents\Projects\openocd (There will be another post to show you how to install openocd and custom GNU toolchain).


Click the Run button at the bottom-right corner to start OpenOCD. Alternatively you may run it via the favorite link named OpenOCD STM32L4 Discovery at the top of the pull-down menu. If it is started successfully, the console shows these log messages:
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 500 kHz
Info : STLINK V2J39M27 (API v2) VID:PID 0483:374B
Info : Target voltage: 3.246592
Info : stm32l4x.cpu: Cortex-M4 r0p1 processor detected
Info : stm32l4x.cpu: target has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32l4x.cpu on 3333
Info : Listening on port 3333 for gdb connections
Now we go back to set up our debug configuration. Click the down arrow next to the Debugger icon to pull down a menu. Click the “Debug Configurations…” menu item.
Click GDB Hardware Debugging >platform-stm32l475-disco (Ext Tool) on the left side panel. Review the default settings which are similar to those in the Built-in OpenOCD section above, except for the Debugger tab shown below.

Click the Debug button at the bottom-right corner to start debugging. Alternatively you may launch it via the favorite link named platform-stm32l475-disco (Ext Tool) at the top of the pull-down menu. When the program is downloaded and flashed successfully, you should see logs ending like these:
xPSR: 0x41000000 pc: 0x08015a9c msp: 0x20017f78
Info : Unable to match requested speed 500 kHz, using 480 kHz
Info : Unable to match requested speed 500 kHz, using 480 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08066e28 msp: 0x20018000
2000
Info : Unable to match requested speed 500 kHz, using 480 kHz
Info : Unable to match requested speed 500 kHz, using 480 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08066e28 msp: 0x20018000
Info : Padding image section 0 at 0x08000194 with 12 bytes
Info : Unable to match requested speed 500 kHz, using 480 kHz
Info : Unable to match requested speed 500 kHz, using 480 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08066e28 msp: 0x20018000
This method is less convenient as you would need to start OpenOCD manually prior to launching a debug session. However it may be faster since OpenOCD remains running in the background across multiple debug sessions.
In some cases you may need to stop OpenOCD manually. For example, when you unplug the target board while Eclipse is still running, OpenOCD will lose its connection to the target and show an error message repeatedly. To stop OpenOCD, click the Red Stop button and the Double Cross button at the top-right corner of the console window.



Настройка отладки в STM32CubeIDE используя внешний Open OCD сервер
--------------------------------------------------------------------------

Run-> Debug Configuration -> GDB Hardware Debugging. Создать или редактировать конфигурацию (например test1 Debug(ocd))

Main:
   Project: test1
   C/C++ Application:   /home/user1/STM32CubeIDE/workspace_1.19.0/test1/Debug/test1.elf
   
Debugger:
   GDB Command:  arm-none-eabi-gdb
   Debug Server: Open OCD(via socket)
   Protocol: remote
   Connection:  localhost:3333
   
Startup: 
  Reset and Delay
  Halt
  
  Load Image and Symbols: Use file ${workspace_loc:/test1/Debug/test1.elf}
  
  Set breakpoint at: main (если нужно) 
  
  
Настраиваем запуск сервера Open OCD (внешнего) из IDE
Run -> External Tools -> External Tools Configuration -> Под Program создаем новый конфиг, например External-Open-OCD

Main:
   Location: /home/user1/STM32Toolchain/bin/openocd
   Working Directory: /home/user1/STM32Toolchain/openocd/scripts
   Arguments: -f interface/stlink.cfg -f board/stm32f103c8_blue_pill.cfg -c "transport select swd"  -c "init" -c "arm semihosting enable"
   
Refresh:
   Specified resource: test1
   Recurcive include subfolders
   
Build:
    Build before launch
    The project conteined the selected resource
    
Отладка
--------
Run -> External Tools -> External-Open-OCD

Запустится Open-OCD сервер, в консоли должны увидеть:

xPack Open On-Chip Debugger 0.12.0+dev-02228-ge5888bda3-dirty (2025-10-04-22:42)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Warn : DEPRECATED: auto-selecting transport "swd (dapdirect)". Use 'transport select swd' to suppress this message.
Warn : Transport "swd" was already selected
Info : STLINK V2J46S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.224463
Info : Unable to match requested speed 1000 kHz, using 950 kHz
Info : Unable to match requested speed 1000 kHz, using 950 kHz
Info : clock speed 950 kHz
Info : SWD DPIDR 0x1ba01477
Info : [stm32f1x.cpu] Cortex-M3 r1p1 processor detected
Info : [stm32f1x.cpu] target has 6 breakpoints, 4 watchpoints
Info : [stm32f1x.cpu] Examination succeed
Info : [stm32f1x.cpu] starting gdb server on 3333
Info : Listening on port 3333 for gdb connections
semihosting is enabled
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections


Жмем стрелку вниз на значке дебага (зеленый жучок) и выбираем наш созданный выше конфиг 
test1 Debug(ocd)
В консоли должны увидеть 
Info : accepting 'gdb' connection on tcp/3333
Info : device id = 0x20036410
Info : ignoring flash probed value, using configured bank size
Info : flash size = 128 KiB
undefined debug reason 8 (UNDEFINED) - target needs reset
Warn : Prefer GDB command "target extended-remote :3333" instead of "target remote :3333"
[stm32f1x.cpu] halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08000608 msp: 0x20005000, semihosting
[stm32f1x.cpu] halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08000608 msp: 0x20005000, semihosting
[stm32f1x.cpu] halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08000608 msp: 0x20005000, semihosting

И появятся кнопки для управления отладкой и вкладка Debug рядом с Project Explorer


Подключение библиотеки Ливиу Ионеску (https://codeload.github.com/eclipse-embed-cdt/eclipse-plugins/zip/refs/tags/v4.7.2-202001271244)
для использования вывода сообщений в отладочную консоль CubeIDE.
----------------------------------------------------------------------------------------------------------------------------------------
Добавляем папку diag в папку Src проекта
В этой папке файлы semihosting.h, Trace.h, Trace.c. trace_impl.c, взятые из скачанной по ссылке выше библиотеки.
В trace_impl.c закомменчена строка //#include "cmsis_device.h" за ненадобностью в CubeIDE
В trace_impl.c исправлен путь до semihosting.h :  #include "arm/semihosting.h" -> #include "semihosting.h"

В main.c подключаем 
#include "diag/Trace.h"
Добавляем также 
extern void initialise_monitor_handles(void); <-- добавляем

int main(void)
{

  /* USER CODE BEGIN 1 */
	initialise_monitor_handles(); <-- добавляем
	...

 trace_printf("Semihosting is working!\n\r");  <-- можем использовать, \n обязательно!!!
 
}

Нужно задать макросы в настройках компилятора (чтобы применились ко всем исходникам).
    ПКМ на проект → Properties
    C/C++ Build → Settings → Tool Settings → MCU GCC Compiler → Preprocessor
    В поле "Defined symbols (-D)" добавляем:

TRACE
OS_USE_TRACE_SEMIHOSTING_DEBUG

Жмем Apply and Close, пересобираем проект.

Теперь при отладке через Open OCD сервер, в консоли будут отображаться сообщения, выводимые через trace_printf()
Функция принимает строку форматирования и параметры аналогично printf()

!!!Важно. При отладке не через Open OCD программа будет зависать на trace_printf(), для того чтобы вернуть
возможность запускать без сеанса отладки с Open OCD нужно в настройках проекта убрать определение TRACE

Полухостинг – отличная функция, но он также имеет несколько недостатков. Прежде
всего, он работает только во время сеанса отладки, и полностью «подвешивает» микро-
программу, если она не работает под управлением GDB. Например, загрузите один из
предыдущих примеров на вашу плату Nucleo и завершите сеанс отладки. Если вы пере-
загрузите плату, нажав кнопку RESET, светодиод LD2 не будет мигать. Это происходит
потому, что микропрограмма зависла в процедуре trace_printf() (подробнее о том, по-
чему это происходит в следующем параграфе). Это достаточно распространенная про-
блема, с которой сталкивается каждый новичок всякий раз, когда начинает работать с
платформой STM32.
Еще один важный аспект, который следует иметь в виду, заключается в том, что полухо-
стинг оказывает большое влияние на производительность программного обеспечения.
Каждый вызов полухостинга затрачивает несколько тактовых циклов ЦПУ, и он влияет
на общую производительность. Более того, эти затраты непредсказуемы, поскольку они
включают в себя действия, которые происходят вне потоков выполнения микроконтроллера.

   


