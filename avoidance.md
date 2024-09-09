# Система обнаружения препятствий и предотвращения столкновений


PX4-совместимые алгоритмы компьютерного зрения, упакованные в узлы ROS для слияния данных с датчиков глубины и избежания препятствий. В этом репозитории содержатся три различных реализации:

-  **local_planner** - локальный планировщик на основе VFH+\*, который планирует с учетом некоторых данных в векторном поле гистограммы.

-  **global_planner** - глобальный планировщик, основанный на графе, который планирует в традиционной окта-карте сетки занятости.

-  **safe_landing_planner** - локальный планировщик для поиска безопасного места для посадки.

Три алгоритма работают автономно и не предназначены для совместного использования.

Локальный планировщик требует меньше вычислительных ресурсов, но не вычисляет оптимальные пути к цели, так как не хранит информацию об уже исследованной среде. Подробнее о его работе можно узнать в этой диссертации. С другой стороны, глобальный планировщик более требователен к ресурсам, так как создает карту окружающей среды. Для точной навигации требуется точное глобальное положение и ориентация. Подробнее о его работе можно узнать в этой диссертации. Safe_landing_planner классифицирует местность под аппаратом на основе среднего значения и стандартного отклонения координаты z точек облака. Облако точек с датчика, направленного вниз, разделяется на 2D-сетку на основе координат xy точек. Для каждого элемента сетки вычисляются среднее значение и стандартное отклонение координаты z, и эти данные используются для нахождения плоских участков, где можно безопасно приземлиться.

>  **Примечание:** Наиболее доработанный и надёжный планировщик - local_planner. Рекомендуется начинать именно с него.

Документация содержит информацию о том, как настроить и запустить две системы планирования в симуляторе Gazebo и на компьютере с ОС Ubuntu 20.04 (рекомендуется), для задач избегания препятствий и предотвращения столкновений.

>  **Примечание:** Настройка на стороне PX4 описана в Руководстве пользователя PX4:
   -  [Обнаружение и облёт препятствий](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html)
   -  [Предотвращение столкновений](https://docs.px4.io/en/computer_vision/collision_prevention.html)

[![Видео о PX4 Avoidance](http://img.youtube.com/vi/VqZkAWSl_U0/0.jpg)](https://www.youtube.com/watch?v=VqZkAWSl_U0)

# Содержание:
-  [Начало работы](#начало-работы)
   -  [Установка](#установка)
      -  [Установка для Ubuntu](#установка-для-ubuntu)
   -  [Запуск симуляции Avoidance в Gazebo](#запуск-симуляции-avoidance-в-gazebo)
      -  [Локальный планировщик](#локальный-планировщик-по-умолчанию-прошёл-интенсивное-тестирование-в-полёте)
      -  [Глобальный планировщик](#глобальный-планировщик-продвинутый-не-тестировался-в-полёте)
      -  [Планировщик безопасной посадки](#планировщик-безопасной-посадки-1)
   -  [Запуск на аппаратном обеспечении](#запуск-на-аппаратном-обеспечении)
      -  [Предварительные требования](#предварительные-требования)
      -  [Локальный планировщик](#локальный-планировщик)
      -  [Глобальный планировщик](#глобальный-планировщик)
-  [Поиск и устранение неисправностей](#поиск-и-устранение-неисправностей)
-  [Расширенные функции](#расширенные-возможности)
   -  [Потоки сообщений](#потоки-сообщений)
      -  [PX4 и локальный планировщик](#взаимодействие-px4-и-локального-планировщика)
      -  [PX4 и глобальный планировщик](#взаимодействие-px4-и-глобального-планировщика)
      -  [PX4 и планировщик безопасной посадки](#взаимодействие-px4-и-планировщика-безопасной-посадки)


# Начало работы

## Установка

### Установка для Ubuntu

Это пошаговое руководство по установке и сборке всех необходимых компонентов для работы модуля avoidance на Ubuntu 20.04 с ROS Noetic (включая Gazebo 11). Возможно, вам придется пропустить некоторые шаги, если ваша система уже частично установлена.

>  **Примечание:** Эти инструкции предполагают, что ваше рабочее пространство catkin, в котором мы будем собирать модуль avoidance, находится в `~/catkin_ws`, а каталог прошивки PX4 -- в `~/Firmware`. При необходимости адаптируйте это под свою систему.


**Добавьте ROS в файл sources.list:**

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main« > /etc/apt/sources.list.d/ros-latest.list'
```

**Установите curl и ROS-ключ:**

```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

**Установите полный набор ROS с Gazebo:**

```bash
sudo apt install ros-noetic-desktop-full
```

**Источники ROS:**

```bash
echo »source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **Примечание:** Мы рекомендуем использовать версию Gazebo, которая поставляется с установкой ROS. Если вам нужно использовать другую версию Gazebo, не забудьте установить связанные пакеты ros-gazebo:

-  Для Gazebo 8:

```bash
sudo apt install ros-noetic-gazebo8-*
```

-  Для Gazebo 9:

```bash
sudo apt install ros-noetic-gazebo9-*
```

### Зависимости для сборки пакетов:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

Установите и инициализируйте rosdep:

```bash
rosdep init

rosdep update
```

Установите catkin и создайте директорию для вашего рабочего пространства catkin:

```bash
sudo apt install python3-catkin-tools

mkdir -p ~/catkin_ws/src
```

### Установка MAVROS (версия 0.29.0 и выше)

> **Примечание:** Инструкции по установке MAVROS из исходных кодов можно найти [здесь](https://github.com/mavlink/mavros/blob/master/mavros/README.md)

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

Установите набор данных для geographiclib:

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod +x install_geographiclib_datasets.sh

sudo ./install_geographiclib_datasets.sh
```

### Установка зависимостей модуля avoidance (библиотека pointcloud и octomap):

```bash
sudo apt install libpcl1 ros-noetic-octomap-*
```

### Клонируйте этот репозиторий в рабочее пространство catkin для сборки узла avoidance:

```bash
cd ~/catkin_ws/src

git clone https://github.com/PX4/avoidance.git
```

### Сборка узла avoidance:

```bash
catkin build -w ~/catkin_ws
```

> **Примечание:** Вы можете собрать узел в режиме релиза:

```bash
catkin build -w ~/catkin_ws —cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Источники setup.bash из вашего рабочего пространства catkin:

```bash
echo «source ~/catkin_ws/devel/setup.bash» >> ~/.bashrc

source ~/.bashrc
```

## Запуск симуляции Avoidance в Gazebo

В следующем разделе описано, как установить и запустить симуляцию в Gazebo для локального и глобального планировщика.

### Сборка и запуск симулятора

Клонируйте прошивку PX4 и все её подмодули (это может занять некоторое время):

```bash
cd ~

git clone https://github.com/PX4/Firmware.git —recursive

cd ~/Firmware
```

Установите зависимости PX4:

```bash
./Tools/setup/ubuntu.sh —no-sim-tools —no-nuttx
```

Установите плагины Gstreamer (для камеры Gazebo):

```bash
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev
```

Соберите прошивку для генерации файлов моделей SDF для Gazebo. Этот шаг фактически запустит симуляцию (её можно сразу же закрыть):

```bash
export QT_X11_NO_MITSHM=1

make px4_sitl_default gazebo
```

Закройте симуляцию (***Ctrl+C***).

Настройте дополнительные переменные среды, связанные с Gazebo:

```bash
. ~/Firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
```

Добавьте каталог прошивки в ROS_PACKAGE_PATH, чтобы ROS мог запускать PX4:

```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
```

Наконец, добавьте переменную GAZEBO_MODEL_PATH в ваш bashrc:

```bash
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds" >> ~/.bashrc
```

Эти последние три шага, а также выполнение команды `source ~/catkin_ws/devel/setup.bash`, должны повторяться каждый раз при открытии нового окна терминала. Вы также можете настроить файл `~/.bashrc`, чтобы эти команды выполнялись автоматически при каждом запуске терминала.

### Локальный планировщик (по умолчанию, прошёл интенсивное тестирование в полёте)

Этот раздел показывает, как запустить локальный планировщик (**local_planner**) и использовать его для избегания препятствий в режиме миссии или режима **OFFBOARD**.

Планировщик основан на алгоритме **3DVFH+**.

> **Примечание:** Возможно, вам потребуется установить дополнительные зависимости для запуска следующего кода (если они не установлены):

```bash
sudo apt install ros-noetic-stereo-image-proc ros-noetic-image-view
```

Для запуска локального планировщика можно использовать любой из трёх следующих файлов сценариев запуска:

> **Примечание:** Все три скрипта запускают один и тот же планировщик, но симулируют различные конфигурации датчиков/камер. Все они включают функции избегания препятствий и предотвращения столкновений.

\- **local_planner_stereo**: симулирует аппарат со стереокамерой, использующей алгоритм блочного согласования OpenCV (по умолчанию SGBM) для генерации данных о глубине.

Запуск:

```bash
roslaunch local_planner local_planner_stereo.launch
```

> **Примечание:** Карта диспаритета из **stereo-image-proc** публикуется как сообщение **stereo_msgs/DisparityImage**, которое не поддерживается визуализаторами **rviz** или **rqt**. Для его визуализации откройте новый терминал и установите необходимые переменные среды:

```bash
source devel/setup.bash
```

Затем выполните одно из следующих действий:

-  Запустите:

```bash
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
```

-  Опубликуйте **DisparityImage** как простое сообщение **sensor_msgs/Image**:

```bash
rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image 'm.image'
```

Карта диспаритета затем может быть визуализирована в **rviz** или **rqt** под темой **/stereo/disparity_image**.

-  **local_planner_depth_camera**: симулирует аппарат с одним фронтально направленным датчиком глубины (например, **Kinect**).

Запуск:

```bash
roslaunch local_planner local_planner_depth-camera.launch
```

-  **local_planner_sitl_3cam**: симулирует аппарат с тремя датчиками глубины (\*\*Kinect\*\*) (слева, справа, спереди).

Запуск:

```bash
roslaunch local_planner local_planner_sitl_3cam.launch
```

После этого вы увидите дрон **Iris** в разоружённом состоянии в мире **Gazebo**. Чтобы начать полёт, есть два варианта: режим **OFFBOARD** или режим **MISSION**.

Для **OFFBOARD** выполните:

```bash
#В другом терминале

rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

Дрон сначала изменит свою высоту, чтобы достичь целевой высоты. Целевую высоту можно изменить с помощью графического интерфейса **rqt_reconfigure**.

Затем дрон начнёт двигаться к цели. По умолчанию, координаты цели **x** и **y** можно изменить в **Rviz** с помощью кнопки **2D Nav Goal**, выбрав новое положение цели путём нажатия на серое пространство визуализированной среды. Если цель установлена правильно, на месте вашего нажатия появится жёлтая сфера.

Для **MISSION**, откройте **QGroundControl** и спланируйте миссию, как описано [здесь](https://docs.qgroundcontrol.com/en/PlanView/PlanView.html). Установите параметр **COM_OBS_AVOID** в **true**. Начните миссию, и дрон будет лететь по путевым точкам, динамически пересчитывая путь так, чтобы он был свободен от столкновений.

### Глобальный планировщик (продвинутый, не тестировался в полёте)

Этот раздел описывает запуск **global_planner** и его использование для избежания препятствий в режиме **OFFBOARD**.

Запуск:

```bash
roslaunch global_planner global_planner_stereo.launch
```

После этого вы увидите дрон в невооружённом состоянии на земле в лесной среде, как показано ниже.

Чтобы начать полёт, переведите дрон в режим **OFFBOARD** и вооружите его. Узел избегания препятствий затем возьмёт управление на себя.

```bash
# В другом терминале

rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

Изначально дрон просто будет висеть на высоте 3,5 м.

С командной строки можно также заставить **Gazebo** следовать за дроном, если необходимо:

```bash
gz camera --camera-name=gzclient_camera --follow=iris
```

Вы можете задать новый путь, установив новую цель с помощью кнопки **2D Nav Goal** в **rviz**. Запланированный путь появится в **rviz**, и дрон будет следовать по нему, обновляя его при обнаружении препятствий. Также можно задать цель без использования избежания препятствий (например, дрон полетит прямо к этой цели, возможно столкнувшись с препятствиями). Для этого установите позицию с помощью кнопки **2D Pose Estimate** в **rviz**.

### Планировщик безопасной посадки

Этот раздел описывает запуск **safe_landing_planner** и его использование для безопасной посадки в режиме **MISSION** или **AUTO LAND**.

Запуск:

```bash
roslaunch safe_landing_planner safe_landing_planner.launch
```

Вы увидите невооружённый аппарат на земле. Откройте **QGroundControl**, либо спланируйте миссию с последней точкой типа **Land**, либо управляйте полётом в режиме **Position Control**, выбрав кнопку **Land** с левой стороны, где вы хотите приземлиться. В выбранной позиции дрон начнёт снижаться к земле до высоты **loiter_height** от земли или препятствия. После этого дрон начнёт зависать для оценки местности под собой. Если местность плоская, дрон продолжит посадку. В противном случае он будет оценивать близлежащую территорию по спиральной траектории, пока не найдёт достаточно безопасное место для посадки.

\---

На этом заканчивается основная информация по запуску локального и глобального планировщиков, а также планировщика безопасной посадки.

## Запуск на аппаратном обеспечении

### Предварительные требования

#### Камера

Оба планировщика требуют 3D облака точек типа **sensor_msgs::PointCloud2**. Любая камера, способная предоставлять такие данные, совместима. Официально поддерживаемая камера -- **Intel Realsense D435**. Мы рекомендуем использовать прошивку версии 5.9.13.0. Инструкции по обновлению прошивки камеры можно найти [здесь](https://github.com/IntelRealSense/librealsense/blob/master/doc/firmware.md).

> **Совет:** Будьте осторожны при подключении камеры с помощью кабеля **USB3**. **USB3** может создавать помехи для сигналов GPS и других датчиков. По возможности всегда используйте кабели **USB2**.

Другие протестированные модели камер:

-  **Intel Realsense D415** и **R200**

-  **Occipital Structure Core**

#### Генерация облаков точек из карт глубины

Если уже существует поток облаков точек, этот шаг можно пропустить. Если поток карт глубины уже есть на ROS-топике **<depthmap_topic>**, необходимо создать соответствующий поток облаков точек. Начните с выполнения инструкций по установке модуля **disparity_to_point_cloud**. Затем запустите генерацию облаков точек с параметрами для внутренних характеристик камеры:

```bash
rosrun disparity_to_point_cloud disparity_to_point_cloud_node \
    fx_:=fx fy_:=fy cx_:=cx cy_:=cy base_line_:=base_line disparity:=<depthmap_topic>
```

Поток облаков точек теперь должен публиковаться в топике **/point_cloud**.

### Автопилот PX4

Необходимые параметры, которые нужно задать через **QGroundControl**:

-  **COM_OBS_AVOID** (включение функции избегания препятствий).

-  **MAV_1_CONFIG**, **MAV_1_MODE**, **SER_TEL2_BAUD** для активации MAVLink на последовательном порту.

   Подробнее в [Руководстве по разработке PX4](https://dev.px4.io/en/).

### Компаньон-компьютер

-  **Операционная система**: Ubuntu 20.04 или контейнер с Ubuntu 20.04 (например, на системе Yocto).

-  **ROS Noetic**: см. [инструкцию по установке](http://wiki.ros.org/noetic/Installation).

#### Необходимые компоненты для **Intel Realsense:**

-  **Librealsense** (SDK для Realsense). Инструкции по установке можно найти [здесь](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md).

-  **Обёртки ROS для Librealsense**.

#### Необходимые компоненты для **Occipital Structure Core**:

-  Скачайте **Structure SDK**. Тестируемая версия пакета -- 0.7.1.

-  Создайте каталог сборки и соберите SDK:

```bash
  mkdir build
  cd build
  cmake ..
  make
```

-  Клонируйте обёртку ROS в **catkin_ws**.

-  Скопируйте библиотеки из **Libraries/Structure/Linux/x86_64/libStructure.so** в **/usr/local/lib/**.

-  Скопируйте заголовки из **Libraries/Structure/Headers/** в каталог **include** в ROS-обёртке **struct_core_ros**.

#### Протестированные модели компьютеров

-  **Local planner**: протестирован на **Intel NUC**, **Jetson TX2**, **Intel Atom x7-Z8750** (встроен в дрон **Intel Aero RTF**).

-  **Global planner**: протестирован на **Odroid**.

### Глобальный планировщик

Глобальный планировщик был протестирован на компьютере-компаньоне Odroid командой разработчиков. Подробностей не сообщалось.

### Локальный планировщик

После сборки рабочего пространства **catkin** для запуска планировщика с камерами **Realsense D435** или **Occipital Structure Core** можно сгенерировать файл запуска с помощью скрипта **generate_launchfile.sh**:

```bash
export CAMERA_CONFIGS="camera_namespace, camera_type, serial_n, tf_x, tf_y, tf_z, tf_yaw, tf_pitch, tf_roll"
export DEPTH_CAMERA_FRAME_RATE=frame_rate
export VEHICLE_CONFIG=/path/to/params.yaml
```

Где **camera_type** -- это либо **realsense**, либо **struct_core_ros**, а параметры **tf**\* указывают на смещение между камерой и контроллером полёта. Для более чем одной камеры перечислите их конфигурации, разделяя их точкой с запятой. Если не указано, будет использована стандартная частота кадров камеры.

Пример:

```bash
export CAMERA_CONFIGS="camera_main,realsense,819612070807,0.3,0.32,-0.11,0,0,0"
export DEPTH_CAMERA_FRAME_RATE=30
export VEHICLE_CONFIG=~/catkin_ws/src/avoidance/local_planner/cfg/params_vehicle_1.yaml

./tools/generate_launchfile.sh
roslaunch local_planner avoidance.launch fcu_url:=/dev/ttyACM0:57600
```

Планировщик работает корректно, если частота обработки облаков точек находится в пределах 10-20 Гц. Для проверки частоты выполните:

```bash
rostopic hz /local_pointcloud
```

Чтобы выводить отладочные сообщения на консоль, измените **custom_rosconsole.conf** следующим образом:

```bash
log4j.logger.ros.local_planner=DEBUG
```

\---

Это были основные шаги для запуска локального и глобального планировщика на аппаратных системах.

### Планировщик безопасной посадки

После сборки рабочего пространства **catkin** для запуска планировщика безопасной посадки с камерами **Realsense D435** или **Occipital Structure Core** можно сгенерировать файл запуска с помощью скрипта **generate_launchfile.sh**. Этот скрипт работает аналогично описанному выше для локального планировщика.

Пример:

```bash
export CAMERA_CONFIGS="camera_main,struct_core,819612070807,0.3,0.32,-0.11,0,0,0"
export VEHICLE_CONFIG_SLP=~/catkin_ws/src/avoidance/safe_landing_planner/cfg/slpn_structure_core.yaml
export VEHICLE_CONFIG_WPG=~/catkin_ws/src/avoidance/safe_landing_planner/cfg/wpgn_structure_core.yaml
./safe_landing_planner/tools/generate_launchfile.sh
roslaunch safe_landing_planner safe_landing_planner_launch.launch
```

В каталоге **cfg/** находятся конфигурации для камер, специфичные для узлов алгоритмов. Эти параметры можно загрузить, указав файл в системной переменной **VEHICLE_CONFIG_SLP** для узла планировщика безопасной посадки и **VEHICLE_CONFIG_WPG** для узла генератора путевых точек.

Размер квадрата местности под аппаратом, который оценивается алгоритмом, можно изменить для разных размеров аппаратов с помощью параметра **smoothing_land_cell** в узле **WaypointGeneratorNode**. Поведение алгоритма также зависит от высоты, на которой принимается решение о посадке (параметр **loiter_height** в **WaypointGeneratorNode**), и от размера фильтра окрестности для сглаживания (параметр **smoothing_size** в **LandingSiteDetectionNode**).

Для различных моделей камер также может потребоваться настройка порогов по числу точек в каждом бине, стандартного отклонения и среднего значения.

## Поиск и устранение неисправностей

### Проблема: Я вижу позицию дрона в **rviz** (показанную красной стрелкой), но окружающий мир пуст

-  Проверьте, публикуются ли какие-либо темы камеры (включая **/camera/depth/points**) с помощью команды:

   `rostopic list | grep camera`

-  Если единственная опубликованная тема -- **/camera/depth/points**, возможно, **Gazebo** не публикует данные с симулированной камеры глубины. Проверьте это с помощью команды:

   `rostopic echo /camera/depth/points`

   Если всё работает правильно, команда должна выводить большое количество данных в терминале. Если сообщений нет, это может означать, что **Gazebo** не публикует данные камеры.

-  Проверьте, публикуются ли данные часов **Gazebo**:

   `rostopic echo /clock`

-  Если часы не публикуются, у вас проблема с **Gazebo** (загрузился ли мир? Видны ли здания и дрон в интерфейсе **Gazebo**?).

-  Если часы публикуются, возможно, проблема в плагине камеры глубины. Убедитесь, что пакет **ros-noetic-gazebo-ros-pkgs** установлен. Если нет, установите его и пересоберите прошивку (с помощью команды **make px4_sitl_default gazebo**, как было объяснено ранее).

### Проблема: Я вижу дрон и мир в **rviz**, но дрон не движется при установке новой цели с помощью «2D Nav Goal»

-  Находится ли дрон в режиме OFFBOARD? Находится ли в состоянии полёта?

-  Переведите дрон в режим OFFBOARD и активируйте его:

```bash
   rosrun mavros mavsys mode -c OFFBOARD 
   rosrun mavros mavsafety arm
```

### Проблема: Я вижу дрон и мир в **rviz**, но дрон не следует по пути корректно

-  Возможно, потребуется настройка файла ***“<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris”***

### Проблема: Я вижу дрон и мир в **rviz**, нахожусь в режиме **OFFBOARD**, но планировщик всё равно не работает

-  Некоторые параметры могут быть настроены через **rqt reconfigure**.



## Расширенные возможности

### Потоки сообщений

Больше информации о взаимодействии между системой избежания препятствий и автопилотом можно найти в [Руководстве пользователя PX4](https://docs.px4.io/master/en/).

#### Взаимодействие PX4 и локального планировщика

Вот полное описание потока сообщений от прошивки PX4 к локальному планировщику:

| Тема PX4                            | MAVLink                            | MAVROS Плагин  | Сообщения ROS               | Тема ROS                       |
|-------------------------------------|------------------------------------|----------------|-----------------------------|--------------------------------|
| vehicle_local_position              | LOCAL_POSITION_NED                 | local_position | geometry_msgs::PoseStamped  | mavros/local_position/pose     |
| vehicle_local_position              | LOCAL_POSITION_NED                 | local_position | geometry_msgs::TwistStamped | mavros/local_position/velocity |
| vehicle_local_position              | ALTITUDE                           | altitude       | mavros_msgs::Altitude       | mavros/altitude                |
| home_position                       | ALTITUDE                           | altitude       | mavros_msgs::Altitude       | mavros/altitude                |
| vehicle_air_data                    | ALTITUDE                           | altitude       | mavros_msgs::Altitude       | mavros/altitude                |
| vehicle_status                      | HEARTBEAT                          | sys_status     | mavros_msgs::State          | mavros/state                   |
| vehicle_trajectory_waypoint_desired | TRAJECTORY_REPRESENTATION_WAYPOINT | trajectory     | mavros_msgs::Trajectory     | mavros/trajectory/desired      |
| none                                | MAVLINK_MSG_ID_PARAM_REQUEST_LIST  | param          | mavros_msgs::Param          | /mavros/param/param_value      |
| none                                | MISSION_ITEM                       | waypoint       | mavros_msgs::WaypointList   | /mavros/mission/waypoints      |

Полное описание потока сообщений от локального планировщика к прошивке PX4:

| Тема ROS                                   | Сообщения ROS                       | MAVROS Плагин            | MAVLink                            | Тема PX4                    |
|--------------------------------------------|-------------------------------------|--------------------------|------------------------------------|-----------------------------|
| /mavros/setpoint_position/local (offboard) | geometry_msgs::PoseStamped          | setpoint_position        | SET_POSITION_LOCAL_POSITION_NED    | position_setpoint_triplet   |
| /mavros/trajectory/generated (mission)     | mavros_msgs::Trajectory             | trajectory               | TRAJECTORY_REPRESENTATION_WAYPOINT | vehicle_trajectory_waypoint |
| /mavros/obstacle/send                      | sensor_msgs::LaserScan              | obstacle_distance        | OBSTACLE_DISTANCE                  | obstacle_distance           |
| /mavros/companion_process/status           | mavros_msgs::CompanionProcessStatus | companion_process_status | HEARTBEAT                          | telemetry_status            |

#### Взаимодействие PX4 и глобального планировщика

Поток сообщений от прошивки PX4 к глобальному планировщику:

| Тема PX4                            | MAVLink                            | MAVROS Плагин  | Сообщения ROS               | Тема ROS                       |
|-------------------------------------|------------------------------------|----------------|-----------------------------|--------------------------------|
| vehicle_local_position              | LOCAL_POSITION_NED                 | local_position | geometry_msgs::PoseStamped  | mavros/local_position/pose     |
| vehicle_local_position              | LOCAL_POSITION_NED                 | local_position | geometry_msgs::TwistStamped | mavros/local_position/velocity |
| vehicle_trajectory_waypoint_desired | TRAJECTORY_REPRESENTATION_WAYPOINT | trajectory     | mavros_msgs::Trajectory     | mavros/trajectory/desired      |

Полное описание потока сообщений от глобального планировщика к прошивке PX4:

| Тема ROS                                   | Сообщения ROS              | MAVROS Плагин     | MAVLink                            | Тема PX4                    |
|--------------------------------------------|----------------------------|-------------------|------------------------------------|-----------------------------|
| /mavros/setpoint_position/local (offboard) | geometry_msgs::PoseStamped | setpoint_position | SET_POSITION_LOCAL_POSITION_NED    | position_setpoint_triplet   |
| /mavros/trajectory/generated (mission)     | mavros_msgs::Trajectory    | trajectory        | TRAJECTORY_REPRESENTATION_WAYPOINT | vehicle_trajectory_waypoint |

#### Взаимодействие PX4 и планировщика безопасной посадки

Поток сообщений от прошивки PX4 к планировщику безопасной посадки:

| Тема PX4               | MAVLink            | MAVROS Плагин  | Сообщения ROS              | Тема ROS                   |
|------------------------|--------------------|----------------|----------------------------|----------------------------|
| vehicle_local_position | LOCAL_POSITION_NED | local_position | geometry_msgs::PoseStamped | mavros/local_position/pose |
| vehicle_status         | HEARTBEAT          | sys_status     | mavros_msgs::State         | mavros                     |

Поток сообщений от планировщика безопасной посадки к прошивке PX4:

| Тема ROS                               | Сообщения ROS                       | MAVROS Плагин            | MAVLink                            | Тема PX4                    |
|----------------------------------------|-------------------------------------|--------------------------|------------------------------------|-----------------------------|
| /mavros/trajectory/generated (mission) | mavros_msgs::Trajectory             | trajectory               | TRAJECTORY_REPRESENTATION_WAYPOINT | vehicle_trajectory_waypoint |
| /mavros/companion_process/status       | mavros_msgs::CompanionProcessStatus | companion_process_status | HEARTBEAT                          | telemetry_status            |

На этом завершается документация по потокам сообщений и взаимодействию с автопилотом PX4.
