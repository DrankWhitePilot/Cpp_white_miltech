# Встановлення та запуск на іншому комп'ютері

Перевірене середовище: Windows, WSL Ubuntu 24.04, Docker, ROS 2 Jazzy,
AirSim 1.8.1 і готова сцена `MSBuild2018/WindowsNoEditor`.

## 1. Залежності

| Компонент | Перевірка |
|---|---|
| WSL Ubuntu 24.04 | `wsl -l -v` |
| Docker Desktop | `wsl -d Ubuntu-24.04 -- docker ps` |
| ROS 2 Jazzy у контейнері | існує `/opt/ros/jazzy/setup.bash` |
| Пакети `rclcpp`, `geometry_msgs`, `nav_msgs`, `ament_cmake` | доступні під час `colcon build` |
| AirSim 1.8.1 у контейнері | є `AirLib/include` і зібрані статичні бібліотеки |
| Готова сцена AirSim | існує `MSBuild2018.exe` разом із папками сцени |
| Базовий репозиторій | `homework_10`, `homework_11`, `coursework_ros` лежать поруч |

Сцена, ROS 2, Docker і вихідне дерево AirSim не входять до Git-репозиторію.
Це зовнішні системні залежності, які треба підготувати окремо.

## 2. Збірка AirSim C++

Після отримання AirSim 1.8.1:

```bash
cmake -S /шлях/до/AirSim-1.8.1/cmake \
  -B /шлях/до/AirSim-1.8.1/build_coursework \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /шлях/до/AirSim-1.8.1/build_coursework -j2
```

У `build_coursework/output/lib` потрібні `libAirLib.a`, `librpc.a` і
`libMavLinkCom.a`.

## 3. Збірка ROS 2 пакета

З кореня `Cpp_white_miltech` усередині контейнера:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/coursework build \
  --base-paths coursework_ros \
  --build-base build/coursework \
  --install-base install/coursework \
  --packages-select coursework_guidance_ros \
  --cmake-args -DCOURSEWORK_REQUIRE_AIRSIM=ON \
    -DAIRSIM_ROOT=/шлях/до/AirSim-1.8.1
source install/coursework/setup.bash
ros2 pkg executables coursework_guidance_ros
```

У переліку має бути `airsim_online_node`. Прапорець
`COURSEWORK_REQUIRE_AIRSIM=ON` перериває збірку, якщо залежності AirSim
відсутні.

## 4. Налаштування Windows

1. Скопіювати весь каталог `coursework_ros/windows` у звичайну папку
   Windows. Файли всередині каталогу не розділяти.
2. Зробити резервну копію наявного
   `%USERPROFILE%\Documents\AirSim\settings.json` і застосувати приклад
   `coursework_ros/config/airsim_settings.json`.
3. Скопіювати `coursework.local.example.json` у `coursework.local.json`.
4. Заповнити поля:
   - `Distro` — назва WSL-дистрибутива;
   - `LinuxUser` — користувач WSL із доступом до Docker;
   - `ContainerName` — точна назва підготовленого контейнера або порожній рядок;
   - `LinuxProject` — корінь репозиторію всередині контейнера;
   - `SceneExe` — повний Windows-шлях до `MSBuild2018.exe`;
   - `AirSimHost` — зазвичай порожній рядок; за потреби IPv4 Windows-хоста.

Приклад:

```json
{
  "Distro": "Ubuntu-24.04",
  "LinuxUser": "user",
  "ContainerName": "coursework_web_control",
  "LinuxProject": "/workspace",
  "SceneExe": "C:\\AirSim\\MSBuild2018\\WindowsNoEditor\\MSBuild2018.exe",
  "AirSimHost": ""
}
```

`coursework.local.json` не комітити: він містить шляхи конкретного ПК.

## 5. Робота користувача

1. Двічі клацнути `START_COURSEWORK.cmd`.
2. Дочекатися сторінки `http://127.0.0.1:8080/`.
3. Вибрати траєкторії та набір параметрів.
4. Натиснути **«ЗАПУСТИТИ СИМУЛЯЦІЮ»**.
5. Для завершення використати **«ЗУПИНИТИ»** або **«ЗАКРИТИ»**.

Головне вікно AirSim захоплюється в панель автоматично. Воно може бути за
браузером, але не повинно залишатися мінімізованим. `CAMERA_FEED.ps1`
відновлює мінімізоване вікно без переведення фокусу.

Для перегляду власного `simulation.json` вибрати файл і натиснути
**«ВІДТВОРИТИ»**. Цей режим не запускає AirSim.

Результати зберігаються у `windows/results/coursework_results.csv`.

## 6. Діагностика

| Симптом | Що перевірити |
|---|---|
| Немає `coursework.local.json` | створити файл із прикладу й заповнити всі обов'язкові поля |
| Не знайдено сцену | `SceneExe` має вказувати саме на наявний `.exe` |
| Не знайдено контейнер | Docker Desktop, `ContainerName`, `LinuxProject` і право виконувати `docker ps` |
| Немає `airsim_online_node` | повторити збірку з `COURSEWORK_REQUIRE_AIRSIM=ON` |
| Немає RPC-з'єднання | налаштування AirSim, `RpcEnabled` та `AirSimHost` |
| Є панель, але немає телеметрії | лог `/tmp/coursework_airsim_restart_*.log` у контейнері |
| Є телеметрія, але немає кадру | вікно `MSBuild2018` і файл `CAMERA_FEED.ps1` поруч зі стартовим скриптом |
| Порт 8080 зайнятий | закрити інший локальний сервер або попередній запуск курсової |

Переносимість на новий ПК означає виконання цих кроків, а не запуск одного
інсталятора. Повний приймальний сценарій наведено у
[VERIFICATION.md](VERIFICATION.md).
