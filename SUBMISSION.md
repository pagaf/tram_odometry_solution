# Тексты для формы сдачи

Задача: **Резервная одометрия по модели**.

Базовая ссылка на репозиторий (ветка с решением):
`https://github.com/pagaf/tram_odometry_solution/tree/route-map-and-stop-anchoring`

---

## 1. Ссылка на пакет(ы) ROS 2 Humble

```text
https://github.com/pagaf/tram_odometry_solution/tree/route-map-and-stop-anchoring/src
Пакеты: tram_reserve_odometry (Python, нода reserve_odometry) и tram_vehicle_msgs. Сборка: colcon build --base-paths src. Запуск: ros2 launch tram_reserve_odometry run.py. Подписка: /vehicle/front_bogie_velocity, /vehicle/rear_bogie_velocity, /vehicle/driver_position_cmd (GNSS fix — только первые 5 с для выставки). Публикация: /result/velocity (VelocitySensor, м/с), /result/position (nav_msgs/Odometry, frame map = система pathgraph), stamp = время входного сообщения.
```

## 2. Ссылка на инструкцию для жюри

```text
https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/README.md
Разделы: 7 «Быстрый запуск для жюри» (сборка, ros2 launch, ros2 bag play, ожидаемые топики), 9 «Offline evaluation» → «Проверка на тестовом бэге организаторов (check-code)» (запуск hackathon_solution_checker), 15 «Что смотреть жюри» (ros2 topic hz, /result/latency_ms, /result/diagnostics). Чек-лист: https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/JURY_CHECKLIST.md
```

## 3. Описание математической модели

```text
https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/MATH_MODEL.md
Кратко: состояние x=[s, v, a]; нелинейная идентифицированная модель удельной тяги/торможения a_id(u,v) по позиции контроллера u=k/15 (u+, u+², u−, u−², v, v|v|, u·v) — аналог характеристики привода и сопротивления Дэвиса; инерция привода первого порядка τ·ȧ + a = a_eq; EKF-прогноз; колёсные скорости (км/ч → м/с) — измерения v с 4-режимным наблюдателем сцепления (норма / буксование передней / задней / синхронное), NIS-гейтом и Huber-обновлением; консенсус тележек; ограничение неподвижности; положение p = Γ(s) по карте маршрута с привязкой к остановкам на платформах. Переход момент→скорость: T_m = m·a_drive·R_w/(n·η·i) при заданных массе/радиусе/передаточном числе.
```

## 4. Допущения, ограничения и конфигурируемые параметры

```text
Допущения и ограничения: https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/ASSUMPTIONS_AND_LIMITS.md
Параметры (с комментариями): https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/src/tram_reserve_odometry/config/odom_params.yaml и README раздел 10 «Конфигурация». Начальная позиция — по двум GNSS-антеннам в окне gnss_init_seconds или manual_initialization/initial_easting/northing/yaw; масса/радиус колеса/передаточное число/КПД — vehicle_mass_kg, wheel_radius_m, gear_ratio, drive_efficiency; сопротивление и тяга — dynamics_coeffs, grade_gain, curve_accel_per_curvature; пороги проскальзывания — nis_gate, consensus_tol_mps, common_slip_weight, stop_speed_mps; фильтр — tau_accel, wheel_sigma, process_accel_sigma; карта — path_file, grid_offset_*, stop_anchor_*.
```

## 5. Сведения о точности и быстродействии

```text
https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/README.md — раздел 9 «Offline evaluation» (методика, таблицы, графики) и раздел 11 «Производительность».
Тестовый бэг организаторов 30618_88aea4d9 (эталон /localization/kinematic_state, метрики как в check-code): скорость RMSE 0.034 м/с; положение 3D RMSE 6.0 м (средняя 2.2 м; x 4.1, y 4.4, z 0.18 м).
Отложенные прогоны (29 шт., эталон GNSS base_link, пары по stamp ±0.05 с): скорость RMSE 0.083 м/с (сырые колёса 0.295); положение RMSE медиана 3.5 м, финальная ошибка медиана 0.18 % пути.
Быстродействие: обработка сообщения p99 0.18 мс (max ~40 мс при инициализации), 0.24 % одного ядра, память ~64 МБ, публикация ~21 Гц. Воспроизведение: tools/replay_eval.py, tools/checker_replay.py, tools/plot_report.py.
```

## 6. Ограничения решения и план развития

```text
https://github.com/pagaf/tram_odometry_solution/blob/route-map-and-stop-anchoring/README.md — раздел 13 «Допущения и ограничения» и раздел 16 «План развития после хакатона».
Главные ограничения: маршрут и остановки сняты для линии Щукинская — Таллинская; ветка тупика у Таллинской после конца пути ненаблюдаема по скоростям колёс (до ~50 м ошибки на последних десятках метров); позиция публикуется после первой пары GNSS-фиксов. План: онлайн-оценка масштаба колёс по привязкам к остановкам, граф маршрута с ветвями, переидентификация модели тяги с уклоном по карте, таймер публикации при пропусках входов, C++-порт ядра, diagnostic_msgs.
```
