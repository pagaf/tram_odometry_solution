# Чек-лист соответствия критериям

## Критерий 1 — скорость

- Нелинейная `notch x speed` модель с first-order actuator lag.
- Есть явная формула перехода `specific traction force -> motor shaft torque`; абсолютный torque публикуется при заданных физических параметрах привода.
- Davis-подобные `v` и `v|v|` члены.
- EKF-предсказание + робастные wheel updates.
- Отдельный контроль bias на разгонах/торможениях через offline `evaluate_dataset.py`.
- Сравнение минимум с `front`, `rear`, `(front+rear)/2`.

## Критерий 2 — положение

- Состояние интегрирует `s`, а не произвольный 2D yaw.
- Стартовая выставка по двум GNSS-антеннам и известным TF в системе pathgraph (`UTM 37N − (300000, 6100000)`).
- Основной режим: `p=Gamma(s)` по маршруту `config/route.json` = pathgraph + снятые конечные участки.
- Привязка к маршруту с учётом курса (пути ST/TS в 3.5 м друг от друга).
- Map-matching по 23 остановкам на платформах гасит дрейф от масштаба колёс (±1.5% между прогонами).
- До инициализации позиция не публикуется (нет «мусора» в произвольной системе).
- Ориентация кузова использует базу тележек 7.55 m.
- `Odometry`: stamp, frame_id, child_frame_id, pose, twist и covariance заполнены.

## Критерий 3 — slip / anomalies

- 4 режима: normal/front-slip/rear-slip/common-slip.
- NIS gating + Huber influence limit.
- Один одометр может быть отброшен независимо от второго.
- Согласованные тележки (|front−rear| ≤ 0.15 м/с) не отбрасываются гейтом: резкое торможение рельсовым тормозом не принимается за выброс.
- Common-mode slip определяется не только разностью колёс, но и противоречием физической модели.
- Медленная model adaptation разрешена только при high-confidence normal adhesion.
- Zero-velocity constraint снижает drift на остановках.
- `/result/diagnostics`: slip score, wheel weights, model accel, common-slip probability.

## Критерий 4 — real time

- `O(1)` память и вычисления на сообщение.
- Нет нейросетей/оптимизации по скользящему окну.
- Публикация идёт от разрешённых input callbacks и throttled до 50 Hz.
- `/result/latency_ms` измеряет локальное callback->publish processing time.
- Runtime не требует интернета.

## Воспроизведение офлайн

1. `python3 tools/calibrate_from_bags.py task_description/data --out calibration.json` — коэффициенты `dynamics_coeffs` (уже перенесены в YAML).
2. `python3 tools/build_route.py task_description/data --bags <список bag> --out src/tram_reserve_odometry/config/route.json` — маршрут и остановки.
3. `python3 tools/replay_eval.py task_description/data --out replay.csv` — метрики кода ноды против GNSS-эталона.
4. На ROS: `ros2 topic hz /result/velocity`, `ros2 topic hz /result/position`, `/result/latency_ms`.
