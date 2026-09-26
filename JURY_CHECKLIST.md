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
- Стартовая MGRS/UTM выставка по двум GNSS-антеннам и известным TF.
- Основной режим: `p=Gamma(s)` по pathgraph.
- Ориентация кузова использует базу тележек 7.55 m.
- `Odometry`: stamp, frame_id, child_frame_id, pose, twist и covariance заполнены.

## Критерий 3 — slip / anomalies

- 4 режима: normal/front-slip/rear-slip/common-slip.
- NIS gating + Huber influence limit.
- Один одометр может быть отброшен независимо от второго.
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

## Перед сдачей обязательно

1. `python3 tools/calibrate_from_bags.py dataset/data --out calibration.json --val-fraction 0.2`
2. Вставить `dynamics_coeffs` из calibration.json в YAML.
3. `python3 tools/evaluate_dataset.py dataset/data --calibration calibration.json --out metrics.csv`
4. После получения карты заполнить `path_csv`.
5. Идентифицировать/проверить `curve_accel_per_curvature`; без данных оставить 0.
6. Записать `ros2 topic hz /result/velocity`, `ros2 topic hz /result/position`, `/result/latency_ms`.
7. Сохранить таблицу ours vs front/rear/mean-wheel на holdout-bags.
