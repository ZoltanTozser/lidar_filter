# Lidar Filter

## Fejlesztői környezet:

Visual Studio Code, ROS Noetic, Python 3.8.10, Terminator 1.91

Dell laptop:
- Intel Core i7-12700H CPU
- 32GB memória
- Nvidia Geforce RTX 3070 graphical card
- 1000GB SSD
- Ubuntu 20.04 

## Teszteléshez felhasznált rosbag: 
- leaf-2021-04-23-campus-bag

## Program indítása: 
- rosbag play -l leaf-2021-04-23-campus-bag
- roslaunch lidar_filter_pkg test1.launch


## 2022.09.25 első commit tartalma:
- ROS package létrehozása
- topicra felíratkozás
- publisher létrehozása
- pontfelhő beolvasása, szelektált pontok listába helyezése, majd numpy tömbbé alakítása
- 2D nupmy tömb kialakítása, a tömb értékeinek kiíratása
- szelektált pontok publikálása
- launch fájl elkészítése
- rviz config fájl elkészítése
- képek feltöltése
