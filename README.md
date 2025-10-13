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
 ../../bin/openocd -f interface/stlink.cfg -f board/stm32f103c8_blue_pill.cfg -c "transport select swd" -c "arm semihosting enable"
 
```




