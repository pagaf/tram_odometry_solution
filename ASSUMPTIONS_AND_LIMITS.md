# Допущения и ограничения

1. Raw front/rear velocity — km/h согласно уточнению организаторов; `/result/velocity` — m/s.
2. Движение в основном рассматривается вперёд (`v>=0`).
3. GNSS после стартового окна не влияет на estimator.
4. MGRS трактуется как метрическая grid-система; при получении официального pathgraph его координаты имеют приоритет.
5. Без карты глобальный yaw на кривой ненаблюдаем по двум скалярным wheel speeds.
6. Синхронный common-mode slip двух тележек не позволяет алгебраически восстановить true speed; в этот момент используется физическая модель и повышенная неопределённость.
7. `mu` wheel–rail не публикуется как физически измеренный коэффициент: без axle load/contact parameters он неидентифицируем. Публикуется slip probability/pseudo-slip.
8. Абсолютный motor torque требует mass, wheel radius, gear ratio, efficiency и driven-axle configuration. До их появления модель работает в удельных силах `F/m`; после заполнения этих параметров код публикует `/result/model_motor_torque_nm`.
9. `curve_accel_per_curvature=0` до получения карты и train-based identification.
10. Коэффициенты dynamics обязательно переоценить на полном train split; bootstrap коэффициенты в YAML построены по одному загруженному bag и не являются финальными.
