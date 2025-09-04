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

