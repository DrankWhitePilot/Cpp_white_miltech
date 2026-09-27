# Перевірка перед здачею

Дата останнього повного локального прогону: **27.09.2026**.

## Підтверджено на авторському ПК

| Перевірка | Фактичний результат |
|---|---|
| Вхідні дані | 30 JSON-файлів у десяти сценаріях успішно прочитано |
| Windows-скрипти | 6 PowerShell-файлів, 0 синтаксичних помилок |
| Вебпанель | 4 статичні тести пройдено; JavaScript компілюється |
| Збірка ROS 2 з AirSim | пакет `coursework_guidance_ros` зібрано з `COURSEWORK_REQUIRE_AIRSIM=ON` |
| Тести ROS 2 | 16 тестів, 0 помилок, 0 провалів, 4 пропущено тестовим набором |
| Повний прогін | сценарій `Базовий тест Кола`, боєприпас `VOG-17`, ціль `T3` |
| Результат прогону | `HIT`, похибка 2.923 м, допустима межа 3.000 м |
| Відео в панелі | захоплення справжнього вікна AirSim, джерело `native-window`, 1440×810 |

Під час повного прогону C++-вузол підключився до AirSim, отримував
телеметрію, виконав сценарій і передав стан вебпанелі. Панель показала
кадри AirSim, поточні параметри та підсумковий результат.

## Команди повторної перевірки в контейнері

Із кореня репозиторію:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/coursework build \
  --base-paths coursework_ros \
  --build-base build/coursework \
  --install-base install/coursework \
  --packages-select coursework_guidance_ros \
  --cmake-args -DCOURSEWORK_REQUIRE_AIRSIM=ON \
    -DAIRSIM_ROOT=/home/user/AirSim-1.8.1

colcon --log-base log/coursework test \
  --base-paths coursework_ros \
  --build-base build/coursework \
  --install-base install/coursework \
  --packages-select coursework_guidance_ros

colcon --log-base log/coursework test-result \
  --test-result-base build/coursework --verbose
```

Статичний тест панелі на Windows:

```powershell
node --test coursework_ros/tests/dashboard_static.test.js
```

## Ручна приймальна перевірка

1. Запустити `coursework_ros/windows/START_COURSEWORK.cmd`.
2. Дочекатися відкриття єдиної вебсторінки.
3. Обрати сценарій руху та тип боєприпасу.
4. Натиснути `ЗАПУСТИТИ СИМУЛЯЦІЮ`.
5. Переконатися, що в лівій частині сторінки видно живі кадри AirSim, а
   праворуч оновлюється телеметрія.
6. Дочекатися результату `HIT` або `MISS` і перевірити показану похибку.
7. Перевірити кнопки `ЗУПИНИТИ` та `ЗАКРИТИ AIRSIM`.
8. Окремо вибрати власний `simulation.json` і запустити його відтворення.

## Межі підтвердженої переносимості

Проєкт перевірено в авторському Windows-середовищі з Docker Desktop,
WSL, готовою сценою AirSim та її C++ залежностями. Встановлення всіх
залежностей з нуля на іншому комп'ютері ще не виконувалося. Тому коректне
формулювання стану: **працює й перевірено на авторському ПК; для іншого ПК
потрібна підготовка залежностей за `SETUP_WINDOWS.md`**.

Готова сцена `WindowsNoEditor` запускається, але її вихідного Unreal-проєкту
в цьому пакеті немає. Створення власної Unreal-сцени є окремим наступним
етапом і не входить до перевіреної версії.
