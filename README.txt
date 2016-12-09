Настройки камеры:

    По хардкору:
     v4l2-ctl -d /dev/video1 --set-ctrl power_line_frequency=2  --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=17

    Доступные команды и возможные значения:
     v4l2-ctl -d /dev/video1 -L

    Граф. интерфейс с неработающими текстбоксами:
     v4l2ucp /dev/video1

    На всякий пожарный:
     sudo apt-get install v4l2ucp


Запуск:

    make -j4; ./JointTracker live calib_Logitech.yml 8.5

    /home/caldor/gazetrack/aruco-1.2.4/utils/aruco_test.cpp

