# fc_stub — заглушка полётного контроллера PX4 с инъекцией отказов

Имитатор полётного контроллера для отладки и автоматического тестирования бортового ПО БВС **без железа**. Для бортового ПО он выглядит как PX4:
- MAVLink 2 поверх UDP;
- режимы и команды PX4;
- переход в удержание при потере потока заданий дольше 500 мс.

Семь отказов включаются сценарием в YAML. Прогон воспроизводим: в режиме модельного времени одинаковые конфигурация и seed дают побайтно одинаковый поток кадров (проверяется SHA-256).

**Язык — C++17:** выдержка периода с джиттером ≤ 1 мс без сборщика мусора и интерпретатора, один язык с бортовым ПО, официальная C-библиотека MAVLink.

## Для проверяющего: где что по ТЗ

| Пункт ТЗ | Где |
|---|---|
| Часть 1. Архитектура бортового ПО (3–4 стр.) | [`docs/architecture-note.md`](docs/architecture-note.md) — §0–6 основная часть, приложения А–В сверх объёма |
| Часть 2. Заглушка ПК, язык и обоснование | этот README, код в [`src/`](src/) и [`include/fcstub/`](include/fcstub/) |
| §3.1 Телеметрия и частоты | [`src/core/fc_core_tx.cpp`](src/core/fc_core_tx.cpp), расписание — [`src/core/scheduler.cpp`](src/core/scheduler.cpp) |
| §3.1 Задания и команды | [`src/core/setpoint_gate.cpp`](src/core/setpoint_gate.cpp), [`src/core/fc_core_rx.cpp`](src/core/fc_core_rx.cpp) |
| §3.1 Режимы и удержание по таймауту 500 мс | [`src/core/mode_machine.cpp`](src/core/mode_machine.cpp) |
| §3.1 Динамика | [`src/core/dynamics.cpp`](src/core/dynamics.cpp), [`src/core/battery.cpp`](src/core/battery.cpp) |
| §3.1 Воспроизводимость | [`src/runtime/sim_driver.cpp`](src/runtime/sim_driver.cpp), [`src/core/rng.cpp`](src/core/rng.cpp) |
| §3.2 Отказы: что ломается, почему, обнаружимость | [`docs/fault-matrix.md`](docs/fault-matrix.md); сценарии — [`config/scenarios/`](config/scenarios/) |
| §3.3 Конфигурация отдельно от кода, ≤ 200 параметров | [`config/default.yaml`](config/default.yaml), загрузчик — [`src/config/`](src/config/) |
| §3.3 Джиттер ≤ 1 мс | [`docs/measurements/jitter-x86_64.md`](docs/measurements/jitter-x86_64.md), тест — [`tests/integration/timing_test.cpp`](tests/integration/timing_test.cpp) |
| §3.3 Тесты: частоты, удержание, каждый отказ, воспроизводимость | [`tests/`](tests/) — см. «Карта репозитория» ниже |
| §3.3 Сборка на чистой машине | [Быстрый старт](#быстрый-старт), [`scripts/verify_clean_machine.sh`](scripts/verify_clean_machine.sh) |

## Быстрый старт

Нужны Linux (x86_64 или aarch64), CMake ≥ 3.22, компилятор C++17 (GCC ≥ 11 или Clang ≥ 14), git и сеть при первой сборке: зависимости скачиваются с зафиксированными версиями.

```bash
./scripts/build_and_test.sh            # конфигурация, сборка, все тесты (release)
```

Варианты: `./scripts/build_and_test.sh debug|asan|coverage`. Без сети можно собрать на системных пакетах: `cmake --preset release -DFCSTUB_USE_SYSTEM_DEPS=ON` (нужны `libyaml-cpp-dev`, `libgtest-dev`).

| Зависимость | Версия |
|---|---|
| MAVLink `c_library_v2` | коммит `28eae47` (MAVLink 2, диалект `common`) |
| yaml-cpp | 0.8.0 |
| GoogleTest | 1.14.0 |

## Запуск

```bash
# реальное время: слушает UDP 14580, шлёт на 127.0.0.1:14540 — как PX4 SITL для MAVSDK/MAVROS
./build/release/src/fc_stub --config config/default.yaml

# модельное время: прогон сценария за доли секунды, журнал и хэш
./build/release/src/fc_stub --config config/scenarios/f5_gnss_drift.yaml --sim --out out/f5
#   -> out/f5/frames.bin  журнал всех кадров
#   -> out/f5/truth.csv   истинное и сообщённое состояние рядом (err_h_m — ошибка положения)
#   -> sha256 <hex>       то же в out/f5/sha256.txt
```

Ключи: `--seed`, `--duration`, `--out`, `--report FILE` (JSON-отчёт о джиттере), `--validate`, `--print-param-count`. Коды выхода:

| Код | Значение |
|---|---|
| 0 | успех |
| 1 | ошибка выполнения |
| 2 | ошибка конфигурации (с путём ключа, например `config: link.bind_port: out of range [1, 65535]`) |
| 3 | ошибка сети при старте |
| 64 | неверная командная строка |

Чтобы подключить QGroundControl, укажите `link.remote_port: 14550`.

## Что умеет

**Телеметрия:**

| Сообщение MAVLink | Частота |
|---|---|
| `HEARTBEAT` (режим, arm, готовность) | 1 Гц |
| `ATTITUDE` | 50 Гц |
| `GLOBAL_POSITION_INT` | 10 Гц |
| `BATTERY_STATUS` (6S) | 2 Гц |
| `SYS_STATUS` (исправность датчиков, батарея) | 1 Гц |
| `EXTENDED_SYS_STATE` (на земле / в воздухе) | 1 Гц |
| `STATUSTEXT` | по событию |

Ответы: `COMMAND_ACK`, `TIMESYNC`.

**Задания и команды:**
- `SET_POSITION_TARGET_LOCAL_NED`: скорость (маска 3527), положение (3576), положение со скоростью (3520). Невалидные задания отбрасываются и **не продлевают** таймаут. Скорость выше `vehicle.v_max_mps` принимается и выполняется на пределе, как в PX4. Задание положения за геозоной или ниже земли **отбрасывается** — в отличие от PX4, который его примет и сработает failsafe по геозоне.
- `COMMAND_LONG`: `ARM_DISARM`, `DO_SET_MODE` с режимами PX4. На повтор команды отвечает прежний `ACK`. Как в PX4, снятие с охраны в воздухе отклоняется, если не передан `param2 = 21196` (принудительно).

**Режимы** (кодировка PX4 в `custom_mode`):

| Режим | arm | PX4 | `custom_mode` | Признак готовности |
|---|---|---|---|---|
| не готов | нет | POSCTL | 196608 | `MAV_STATE_UNINIT` / `BOOT` |
| готов | нет | POSCTL | 196608 | `MAV_STATE_STANDBY` |
| ручной | да | MANUAL | 65536 | |
| внешнее управление | да | OFFBOARD | 393216 | |
| удержание | да | AUTO.LOITER | 50593792 | |

Во внешнем управлении при отсутствии валидных заданий **больше 500 мс** — переход в удержание и `STATUSTEXT CRITICAL "Offboard lost >500ms: HOLD"`. Это эквивалент `COM_OF_LOSS_T = 0.5`, `COM_OBL_RC_ACT = Hold` в PX4. Из удержания обратно во внешнее управление — только явной командой.

**Динамика:** интегратор первого порядка по скорости (τ = 0,3 с), задание положения через P-регулятор, углы из ускорения, батарея — по кулоновскому счётчику. Как в PX4, контуры замкнуты на **оценку** состояния, а не на истину: при отказе оценки (F1, F5) телеметрия продолжает следовать заданию, а уходит настоящий аппарат.

## Отказы

| # | Отказ | Слой | Видно ли бортовому ПО |
|---|---|---|---|
| F1 | замёрзшая оценка состояния | оценка | при движении — да, за 1,0 с; **при висении — нет** |
| F2 | перезагрузка ПК в полёте | жизненный цикл | да: HEARTBEAT через 1,5 с, откат времени |
| F3 | дрейф и скачок часов ПК | часы | **без TIMESYNC — нет**; с TIMESYNC — да |
| F4 | потери пачками, задержка, переупорядочивание, порча байтов | линия | да: разрывы `seq` через 0,06 с |
| F5 | ГНСС: скачок / медленный дрейф | оценка | скачок — сразу; **дрейф — нет**, ошибка > 10 м |
| F6 | батарея: потеря ёмкости / рост сопротивления | батарея | **по остатку в % — нет**; сверкой с напряжением — да |
| F7 | автопилот сам выходит из OFFBOARD (пилот, failsafe) | режимы | да: HEARTBEAT ≤ 1 с, причина в STATUSTEXT |

Подробности, причины и найденные ловушки — в [`docs/fault-matrix.md`](docs/fault-matrix.md). Сценарии — в `config/scenarios/`.

## Конфигурация

[`config/default.yaml`](config/default.yaml) — все ключи с комментариями и значениями по умолчанию. Схема `schema_version: 1`. Неизвестный или повторённый ключ, неверный тип, выход за диапазон — ошибка с путём ключа. В документе не больше 200 параметров (`--print-param-count`); типичный сценарий — 20–50. Секция `estimator:` задаёт шум оценки (по умолчанию нет); в сценариях отказов он включён на уровне EKF2 с обычным ГНСС.

## Тесты и проверки

| Команда | Что проверяет |
|---|---|
| `./scripts/build_and_test.sh` | 191 тест: модули, ядро, режим модельного времени, сценарии отказов и эталонный детектор, CLI и обмен по UDP (метка `integration`, реальный процесс), джиттер в реальном времени (метка `integration_timing`) |
| `ctest --preset release -L timing` | реальное время 5 с: все 6 потоков на своих частотах, джиттер ≤ 1 мс, пропусков нет (метка `integration_timing`; на нагруженной или эмулируемой машине исключить `-LE timing`) |
| `./scripts/build_and_test.sh asan` | то же под AddressSanitizer + UndefinedBehaviorSanitizer |
| `./scripts/quality.sh` | cppcheck (0 замечаний) и clang-format |
| `./scripts/coverage.sh` | покрытие `src/` (нужен `gcovr`, порог 80 % строк): сейчас **92,6 % строк**, 60,5 % ветвлений |
| `./scripts/verify_clean_machine.sh` | чистые контейнеры из `git archive HEAD`: Ubuntu 22.04 и 24.04 с g++, 22.04 с clang++, покрытие, arm64 (qemu) — все PASS |
| `.venv/bin/python scripts/mavsdk_check.py` | настоящий MAVSDK 2.8 (`pip install "mavsdk>=2.8,<3"` в venv): подключение, arm, OFFBOARD по заданиям скорости, `in_air`, HOLD, отказ disarm в воздухе — все PASS |
| `./scripts/measure_jitter.sh idle 60` | замер джиттера на этой машине |

Ключевые проверки:
- частоты 1/50/10/2/1/1 Гц ровно на сетке, в модельном и в реальном времени;
- граница таймаута 499/501 мс;
- **ноль выделений памяти** в ядре после старта (отдельный исполняемый файл подменяет `operator new`);
- одинаковый SHA-256 двух прогонов и эталонный хэш сценария со всеми отказами — совпадает в `-O0`, `-O2`, под ASan, в clang, на Ubuntu 24.04 (GCC 13) и на aarch64;
- независимость потоков случайных чисел отказов.

## Джиттер

Измерено на x86_64 (i7-10700F, Ubuntu 22.04, ядро без PREEMPT_RT, без `SCHED_FIFO`), 60 с:

| Условия | max |
|---|---|
| простой | **0,31 мс** ✅ |
| простой, `spin_us: 200` | **0,15 мс** ✅ |
| все 16 потоков CPU заняты | 5 мс ❌ |

Под полной нагрузкой нужен `SCHED_FIFO`: `sudo setcap cap_sys_nice+ep build/release/src/fc_stub`. Заглушка включает его сама, если разрешено, и предупреждает, если нет. Подробно: [`docs/measurements/jitter-x86_64.md`](docs/measurements/jitter-x86_64.md).

**Jetson Orin Nano Super:** сборка, тесты и воспроизводимость проверены в Docker arm64 (qemu). Тайминги на самом устройстве **не измерены** — железа нет. Для замера на устройстве: `nvpmodel` MAXN SUPER, `jetson_clocks`, ядро PREEMPT_RT из JetPack 6, `SCHED_FIFO`, затем `scripts/measure_jitter.sh load 60 0`.

## Устройство кода

- **Ядро** ([`FcCore`](include/fcstub/fc_core.hpp)) не читает часы, не спит, не выделяет память и не трогает сеть: время и байты передаются ему аргументами, кадры уходят через `FrameSink`. Поэтому один и тот же код работает и за UDP-сокетом в реальном времени, и в модельном времени для воспроизводимых тестов.
- **Один поток.** Вызовы ядра из обработчиков прерываний и сигналов не допускаются.
- **Отказы вносятся там, где возникают на реальном аппарате:**
  - F1, F5 — в оценке, из которой строится телеметрия: [`estimator_tap.cpp`](src/core/estimator_tap.cpp);
  - F2 — в жизненном цикле процесса: [`lifecycle.cpp`](src/core/lifecycle.cpp), `FcCore::reboot` в [`fc_core.cpp`](src/core/fc_core.cpp);
  - F3 — в часах автопилота (метки времени и TIMESYNC): [`fc_clock.cpp`](src/core/fc_clock.cpp);
  - F4 — в модели линии для каждого направления: [`link_model.cpp`](src/core/link_model.cpp);
  - F6 — в модели батареи (напряжение — от настоящего заряда, остаток — от заданной ёмкости): [`battery.cpp`](src/core/battery.cpp);
  - F7 — в машине режимов (решение автопилота или пилота): `FcCore::handle_overrides` в [`fc_core.cpp`](src/core/fc_core.cpp).

  Окна отказов и потоки случайных чисел — [`fault_schedule.cpp`](src/core/fault_schedule.cpp): у каждого отказа свой поток.
- **Детерминизм между платформами** держится на флаге `-ffp-contract=off` ([`CMakeLists.txt`](CMakeLists.txt)): без него GCC на aarch64 подставляет FMA. `-march=native` и `-ffast-math` запрещены.

## Карта репозитория

**Документы**

| Файл | Что внутри |
|---|---|
| [`docs/architecture-note.md`](docs/architecture-note.md) | Часть 1: архитектура бортового ПО, стыки, бюджет канала, время, ROS, отказы и полномочия |
| [`docs/fault-matrix.md`](docs/fault-matrix.md) | Отказы F1–F7: что ломается, почему вероятно, видно ли бортовому ПО; сводка обнаружимости с цифрами |
| [`docs/measurements/jitter-x86_64.md`](docs/measurements/jitter-x86_64.md) | Методика и результаты замера джиттера, исходные JSON рядом |

**Ядро — [`src/core/`](src/core/), заголовки — [`include/fcstub/`](include/fcstub/)**

| Файл | Что делает |
|---|---|
| [`fc_core.cpp`](src/core/fc_core.cpp), [`fc_core.hpp`](include/fcstub/fc_core.hpp) | `FcCore`: порядок работы на шаге, перезагрузка (F2), смена режима автопилотом (F7) |
| [`fc_core_rx.cpp`](src/core/fc_core_rx.cpp) | Входящие задания, `COMMAND_LONG` (arm/disarm, смена режима), ответ на TIMESYNC |
| [`fc_core_tx.cpp`](src/core/fc_core_tx.cpp) | Исходящая телеметрия, HEARTBEAT, STATUSTEXT |
| [`mode_machine.cpp`](src/core/mode_machine.cpp) | Пять режимов, переходы, удержание по потере заданий > 500 мс |
| [`setpoint_gate.cpp`](src/core/setpoint_gate.cpp) | Проверка заданий: маска, система координат, NaN, геозона |
| [`command_dedup.cpp`](src/core/command_dedup.cpp) | Повтор `COMMAND_LONG` в течение 1 с получает прежний ACK |
| [`dynamics.cpp`](src/core/dynamics.cpp) | Интегратор первого порядка, контуры на оценке, контакт с землёй |
| [`estimator_tap.cpp`](src/core/estimator_tap.cpp) | Сообщённая оценка: шум, замерзание (F1), отказ ГНСС (F5) |
| [`battery.cpp`](src/core/battery.cpp) | Кулоновский счётчик, просадка под током, отказ батареи (F6) |
| [`fc_clock.cpp`](src/core/fc_clock.cpp) | Часы автопилота: `time_boot_ms`, дрейф и скачок (F3) |
| [`lifecycle.cpp`](src/core/lifecycle.cpp) | Окна перезагрузки автопилота (F2) |
| [`link_model.cpp`](src/core/link_model.cpp) | Линия: потери Гилберта–Эллиотта, задержка, переупорядочивание, порча байтов (F4) |
| [`fault_schedule.cpp`](src/core/fault_schedule.cpp) | Окна отказов и отдельный поток случайных чисел на отказ |
| [`scheduler.cpp`](src/core/scheduler.cpp) | Периодические задачи на абсолютной сетке времени, учёт пропусков |
| [`mav_codec.cpp`](src/core/mav_codec.cpp) | Кодирование и разбор MAVLink 2 (единственное место с заголовками MAVLink) |
| [`virtual_client.cpp`](src/core/virtual_client.cpp) | Сценарный «бортовой компьютер» для модельного времени |
| [`rng.cpp`](src/core/rng.cpp), [`sha256.cpp`](src/core/sha256.cpp) | Переносимые генераторы случайных чисел и SHA-256 для воспроизводимости |
| [`geo.hpp`](include/fcstub/geo.hpp), [`time.hpp`](include/fcstub/time.hpp) | Пересчёт NED → WGS-84, единицы времени |

**Конфигурация — [`src/config/`](src/config/), [`config/`](config/)**

| Файл | Что делает |
|---|---|
| [`config_loader.cpp`](src/config/config_loader.cpp) | Схема `schema_version: 1`, диапазоны, бюджет 200 параметров, дубликаты ключей |
| [`config_faults.cpp`](src/config/config_faults.cpp) | Разбор `faults[]` и `client[]` |
| [`yaml_section.cpp`](src/config/yaml_section.cpp) | Типизированное чтение YAML с путём ключа в ошибке |
| [`config/default.yaml`](config/default.yaml) | Все ключи с комментариями и значениями по умолчанию |
| [`config/scenarios/`](config/scenarios/) | Сценарии: `clean`, по одному-два на каждый отказ F1–F7, `all_faults` |

**Драйверы и программа — [`src/runtime/`](src/runtime/), [`src/app/`](src/app/)**

| Файл | Что делает |
|---|---|
| [`realtime_driver.cpp`](src/runtime/realtime_driver.cpp) | Реальное время: `ppoll` до абсолютного дедлайна, `mlockall`, `SCHED_FIFO`, отчёт о джиттере |
| [`udp_link.cpp`](src/runtime/udp_link.cpp) | Неблокирующий UDP-сокет |
| [`sim_driver.cpp`](src/runtime/sim_driver.cpp) | Модельное время: журнал `frames.bin`, `truth.csv`, SHA-256 |
| [`jitter_stats.cpp`](src/runtime/jitter_stats.cpp) | Статистика отклонений от дедлайна |
| [`main.cpp`](src/app/main.cpp) | Командная строка `fc_stub`, коды выхода |

**Тесты — [`tests/`](tests/)**

| Файл | Что проверяет |
|---|---|
| [`unit/fc_core_test.cpp`](tests/unit/fc_core_test.cpp) | Частоты потоков, удержание через 500 мс, команды, перезагрузка, F7, disarm в воздухе |
| [`unit/mode_machine_test.cpp`](tests/unit/mode_machine_test.cpp) | Переходы режимов, граница таймаута 499/501 мс |
| [`unit/scheduler_test.cpp`](tests/unit/scheduler_test.cpp) | Точное число срабатываний за 60 с, пропуски, порядок |
| [`unit/faults_test.cpp`](tests/unit/faults_test.cpp) | F1, F2, F3, F5 и шум оценки по отдельности |
| [`unit/link_model_test.cpp`](tests/unit/link_model_test.cpp) | F4: потери пачками, задержка, порча, переполнение |
| [`unit/dynamics_test.cpp`](tests/unit/dynamics_test.cpp) | Динамика, контуры на оценке, батарея и F6 |
| [`unit/sim_driver_test.cpp`](tests/unit/sim_driver_test.cpp) | Воспроизводимость: одинаковый хэш двух прогонов, эталонный хэш |
| [`unit/config_test.cpp`](tests/unit/config_test.cpp) | Схема конфигурации и сообщения об ошибках |
| [`unit/`](tests/unit/) — остальные | Кодек MAVLink, ворота заданий, повтор команд, ГСЧ, SHA-256 |
| [`scenario/detectability_test.cpp`](tests/scenario/detectability_test.cpp) | Каждый сценарий отказа против эталонного детектора: обнаружен ли и когда |
| [`support/naive_detector.cpp`](tests/support/naive_detector.cpp) | Эталонный детектор: девять проверок, как у бортового ПО |
| [`noalloc/no_alloc_test.cpp`](tests/noalloc/no_alloc_test.cpp) | Ядро не выделяет память после старта, со всеми отказами |
| [`integration/udp_e2e_test.cpp`](tests/integration/udp_e2e_test.cpp) | Настоящий процесс по UDP: arm, OFFBOARD, HOLD после потери заданий |
| [`integration/timing_test.cpp`](tests/integration/timing_test.cpp) | Реальное время: частоты всех потоков и джиттер ≤ 1 мс |
| [`integration/cli_test.cpp`](tests/integration/cli_test.cpp) | Командная строка и коды выхода |

**Сборка и скрипты**

| Файл | Что делает |
|---|---|
| [`CMakeLists.txt`](CMakeLists.txt), [`CMakePresets.json`](CMakePresets.json), [`cmake/Dependencies.cmake`](cmake/Dependencies.cmake) | Сборка, пресеты `release`/`debug`/`asan`/`coverage`, зависимости с фиксированными версиями |
| [`scripts/build_and_test.sh`](scripts/build_and_test.sh) | Сборка и все тесты одной командой |
| [`scripts/quality.sh`](scripts/quality.sh), [`scripts/coverage.sh`](scripts/coverage.sh) | cppcheck и clang-format; покрытие с порогом 80 % |
| [`scripts/verify_clean_machine.sh`](scripts/verify_clean_machine.sh), [`docker/Dockerfile`](docker/Dockerfile) | Сборка и тесты в чистых контейнерах, включая arm64 |
| [`scripts/measure_jitter.sh`](scripts/measure_jitter.sh) | Замер джиттера в простое и под нагрузкой |
| [`scripts/mavsdk_check.py`](scripts/mavsdk_check.py) | Проверка настоящим MAVSDK |

## Ограничения и что не сделано

- **Падение аппарата не моделируется.** При перезагрузке ПК в воздухе (F2), принудительном disarm и полном разряде батареи (F6) заглушка сохраняет позицию, а реальный аппарат упал бы.
- **Нет подписи MAVLink.** Управлять заглушкой может любой узел в сети.
- **Нет протоколов параметров и миссий** (`PARAM_*`, `MISSION_*`). Некоторые клиенты (QGC) будут их запрашивать.
- **Заглушка не инициирует TIMESYNC**, только отвечает на запросы.
- **Тайминги на Jetson не измерены**, `SCHED_FIFO` на этой машине недоступен (нет прав). TSan не запускался: на ядре 6.8 он не стартует без `setarch -R`, а в коде один поток.
