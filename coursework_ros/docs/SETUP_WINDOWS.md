# Встановлення та запуск на іншому комп'ютері

Ця інструкція описує **Windows + WSL Ubuntu 24.04 + Docker/dev container**.
Вона не означає, що достатньо лише завантажити репозиторій: ROS 2 та AirSim
є окремими залежностями. Перевірений автором стенд використовує ROS 2 Jazzy,
AirSim 1.8.1 і готову Windows-сцену `MSBuild2018`.

## 1. Що має бути підготовлено

| Компонент | Де працює | Як перевірити |
|---|---|---|
| Windows-сцена AirSim `WindowsNoEditor` | Windows | існує `MSBuild2018.exe` разом із папками `Engine` і `MSBuild2018` |
| WSL Ubuntu 24.04 та Docker Desktop | Windows/WSL | `wsl -l -v`, `wsl -d Ubuntu-24.04 -- docker ps` |
| C++ dev container базового репозиторію | Docker | `docker ps` показує контейнер зі змонтованим `Cpp_white_miltech` |
| ROS 2 Jazzy, `colcon`, C++20, пакети `rclcpp`, `geometry_msgs`, `nav_msgs`, `ament_cmake` | всередині контейнера | існує `/opt/ros/jazzy/setup.bash`; `colcon --version-check` виконується |
| Вихідний AirSim 1.8.1 і статичні бібліотеки | всередині контейнера | `AirLib/include/api/RpcLibClientBase.hpp` та три `.a` у `build_coursework/output/lib` |
| `homework_10`, `homework_11`, `coursework_ros` | один корінь репозиторію | ці каталоги є сусідами |

Готова сцена важить приблизно 1,06 ГБ і **не додається до Git**. Потрібно
отримати та розпакувати її окремо. Цей каталог також не містить самого
AirSim 1.8.1, ROS 2 або образу вже налаштованого контейнера. У наявному
`.devcontainer/Dockerfile` ROS та AirSim автоматично не встановлюються:
на новому ПК їх треба підготувати до збірки курсової.

AirSim C++-частину в поточному середовищі зібрано так (після отримання
вихідного дерева AirSim та його залежностей):

```bash
cmake -S /шлях/до/AirSim-1.8.1/cmake \
  -B /шлях/до/AirSim-1.8.1/build_coursework \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /шлях/до/AirSim-1.8.1/build_coursework -j2
```

У каталозі `build_coursework/output/lib` після цього потрібні `libAirLib.a`,
`librpc.a` і `libMavLinkCom.a`. Збирання на повністю чистому ПК поки не
перевірене; не вважайте сам факт успішної ROS-збірки підтвердженням
наявності AirSim-адаптера.

## 2. Збірка курсової всередині контейнера

З кореня `Cpp_white_miltech` (замініть шлях до AirSim на свій):

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

У списку має бути `airsim_online_node`. Параметр
`COURSEWORK_REQUIRE_AIRSIM=ON` не дозволяє непомітно зібрати лише ROS-частину,
коли бібліотек AirSim немає.

## 3. Windows-частина

1. Скопіюйте весь каталог `coursework_ros/windows` у звичайну папку Windows
   (наприклад, `Documents\CourseworkAirSim`). Скрипти не потрібно класти на
   робочий стіл; усі файли мають залишатися разом.
2. Збережіть резервну копію наявного
   `%USERPROFILE%\Documents\AirSim\settings.json`, якщо він існує. Для цієї
   сцени використайте приклад `coursework_ros/config/airsim_settings.json`.
   Не перезаписуйте особисті налаштування без резервної копії.
3. У Windows-каталозі скопіюйте `coursework.local.example.json` у
   `coursework.local.json` і заповніть:
   - `Distro`: ім'я WSL-дистрибутива (`wsl -l -q`);
   - `LinuxUser`: користувач WSL, який може виконувати `docker`;
   - `LinuxProject`: абсолютний шлях до кореня репозиторію **в контейнері**;
   - `SceneExe`: повний шлях до `MSBuild2018.exe` у Windows;
   - `AirSimHost`: залиште порожнім для автоматичного визначення шлюзу WSL;
     якщо мережа налаштована інакше, впишіть IPv4-адресу Windows-хоста.
4. Двічі клацніть `START_COURSEWORK.cmd`. Після перевірок відкриється локальна
   панель `http://127.0.0.1:8080/`. У ній виберіть сценарій і запустіть
   AirSim. Сцена відкривається окремим вікном.

У каталозі `windows/assets/target_textures` мають залишатися всі шість PNG:
вони додаються до репозиторію й не потребують окремого копіювання у профіль
Windows. Файл `coursework.local.json` — персональне налаштування; його не
комітять. Панель слухає лише локальну адресу; мережевий доступ до неї не
налаштовується.

## 4. Повторний запуск, результати і завершення

- У панелі: `ЗАПУСТИТИ AIRSIM`, `ЗУПИНИТИ`, `ЗАКРИТИ`.
- `ВІДТВОРИТИ` для `simulation.json` — тільки локальний перегляд даних, без
  запуску AirSim.
- `ПОРІВНЯТИ З AIRSIM` — порівняння CSV-журналу телеметрії ДЗ11 із записаним
  шляхом AirSim; треба підтвердити однакові умови, координати й ділянку польоту.
  Запис журналу описано в [HW11_TELEMETRY.md](HW11_TELEMETRY.md).
- Журнал результатів: `windows/results/coursework_results.csv`.
- Якщо активне саме вікно AirSim, `J` повторює поточний сценарій, `Esc`
  закриває демонстрацію. Кнопки панелі не залежать від цих клавіш.

## 5. Якщо не запускається

| Симптом | Перевірка |
|---|---|
| `coursework.local.json` не знайдено | виконати пункт 3; не перейменовувати приклад без заповнення |
| Сцену не знайдено | `SceneExe` має вказувати на існуючий `.exe`, а не на `.uproject` |
| Контейнер не знайдено | запустити dev container; перевірити `LinuxProject` і право `docker ps` у WSL |
| Пакет ROS або `airsim_online_node` не знайдено | повторити збірку з `COURSEWORK_REQUIRE_AIRSIM=ON` та перевірити `install/coursework/setup.bash` |
| RPC-з'єднання з AirSim не встановлено | перевірити, що сцена відкрилася, `RpcEnabled` у settings, а `AirSimHost` відповідає мережі WSL |
| Вебпанель відкрита, але телеметрії немає | перевірити лог останнього запуску `/tmp/coursework_airsim_restart_*.log` у контейнері |
| Порівняння не показує цифр | потрібні щонайменше дві точки AirSim, CSV-журнал ДЗ11 формату `hw11-telemetry-v1` і підтвердження однакових умов; `simulation.json` тут не приймається |

Повний запуск на **іншому** ПК має пройти окрему приймальну перевірку за
[VERIFICATION.md](VERIFICATION.md); наявні перевірки виконано в авторському
середовищі.
