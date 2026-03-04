# LIDAR-Point-Cloud-Segmentation

Короткое руководство по проекту.

**Содержание**
- Быстрый старт
- Запуск для разных датасетов (helimos, helipr, hercules)
- Работа с GPS (структура CSV, визуализация)
- Конфигурация проекта (`src/config.py`)
- Примеры и отладка

---

## Быстрый старт

1. Установите зависимости:

```bash
pip install -r requirements.txt
```

2. Запускайте команды из корневой директории проекта:

```bash
# Пример: визуализация GPS (данные из датасета hercules)
python -m src.app --dataset hercules --gps 03_Day/sensor_data/gps.csv --action map
```

3. После успешного запуска будет создана HTML-карта (по умолчанию `gps_map.html` или путь из конфига).

---

## Запуск для разных датасетов

Доступные датасеты: `helimos`, `helipr`, `hercules`. GPS, INS, IMU и Radar — это сенсорные данные датасета **hercules**.

### HeLiMOS (KITTI-подобный бинарный формат)

```bash
python -m src.app --dataset helimos --bin <path_to_bin> --action cloud
```

### HeLiPR (Aeva LiDAR)

```bash
python -m src.app --dataset helipr --bin <path_to_bin> --action cloud
# график радиальной скорости от азимута
python -m src.app --dataset helipr --bin <path_to_bin> --action velocity
```

### HeRCULES

Датасет hercules содержит несколько типов сенсорных данных. Тип данных определяется аргументом пути:

```bash
# Aeva LiDAR (--bin)
python -m src.app --dataset hercules --bin <path_to_aeva.bin> --action cloud
python -m src.app --dataset hercules --bin <path_to_aeva.bin> --action velocity

# Radar (--radar)
python -m src.app --dataset hercules --radar <path_to_radar.bin> --action cloud
python -m src.app --dataset hercules --radar <path_to_radar.bin> --action velocity

# GPS (--gps)
python -m src.app --dataset hercules --gps <path_to_gps.csv> --action map

# INS (--ins)
python -m src.app --dataset hercules --ins <path_to_ins.csv> --action track

# IMU (--imu)
python -m src.app --dataset hercules --imu <path_to_imu.csv> --action accel
```

Примечание: приложение использует ленивые импорты — heavy-зависимости (например, `open3d`) подгружаются только при визуализации облака точек, поэтому визуализация GPS работает даже без установки всех пакетов.

---

## GPS: структура данных и визуализация

Файл `03_Day/sensor_data/gps.csv` в этом проекте содержит 13 колонок без заголовков. В коде чтения (`src/io/csv_reader.py`) и документации они распарсены как:

0. `timestamp` (нс)
1. `lat` (широта)
2. `lon` (долгота)
3. `height` (высота, м)
4. `velocity_north`
5. `velocity_east`
6. `velocity_up`
7. `roll`
8. `pitch`
9. `azimuth`
10. `status`
11-12. дополнительные/зарезервированные поля

В данном конкретном наборе многие колонки заполнены нулями (статический стенд, или данные отфильтрованы). Для более точной информации смотрите `03_Day/sensor_data/inspva.csv` — там есть исчерпывающие INS/IMU-значения (roll/pitch/azimuth и компоненты скорости).

Визуализация:
- Скрипт сохраняет интерактивную карту Folium (HTML). Открыть её можно:
	- через двойной клик в проводнике или в VS Code
	- через локальный сервер: `python -m http.server 8000` и перейти на `http://localhost:8000/gps_map.html`

---

## Конфигурация проекта — `src/config.py`

Файл `src/config.py` содержит все константы путей и параметры визуализации/обработки:
- Пути к данным (DATA_DIR, SENSOR_DATA_DIR, GPS_DATA_FILE и т.д.)
- Пути к результатам (GPS_MAP_FILE, GPS_TRAJECTORY_MAP_FILE и т.д.)
- Параметры визуализации (MAP_ZOOM_LEVEL, POLYLINE_COLOR...)
- Параметры обработки (POINT_SKIP, GPS_SKIP, FLOAT_PRECISION)

Если хотите изменить параметры — правьте `src/config.py` или переопределяйте значения в рантайме.

---

## Motion Object Segmentation (MOS)

Модуль сегментации движущихся объектов в облаке точек LiDAR. Классифицирует каждую точку как **static** или **moving** с помощью Random Forest, обученного на размеченных данных HeLiMOS.

### 1. Обучение модели

Обучение на HeLiMOS (ground truth labels: static=9, moving=251):

```bash
python -m src.app --dataset helimos --action mos-train --sequence data/Deskewed_LiDAR --sensor Velodyne --split train
```

Параметры:
- `--sequence` — путь к корню датасета Deskewed_LiDAR (по умолчанию: `data/Deskewed_LiDAR`)
- `--sensor` — тип сенсора: `Velodyne`, `Ouster`, `Avia`, `Aeva` (по умолчанию: `Velodyne`)
- `--split` — раздел датасета: `train`, `val`, `test` (по умолчанию: `train`)
- `--max-frames` — ограничить число кадров для обучения (по умолчанию: все)
- `--model` — путь для сохранения модели (по умолчанию: `models/mos_rf.pkl`)
- `--threshold` — порог P(moving) для классификатора (по умолчанию: `0.85`)

Обучение занимает ~2 минуты. Модель сохраняется в `models/mos_rf.pkl`.

### 2. Инференс на одном кадре

Работает с любым датасетом — loader выбирается по `--dataset`:

```bash
# HeLiMOS (Velodyne, без Допплера — на левом графике height profile)
python -m src.app --dataset helimos --bin <path_to_helimos.bin> --action mos --model models/mos_rf.pkl

# HeLiPR Aeva (есть Допплер — на левом графике radial velocity vs azimuth)
python -m src.app --dataset helipr --bin <path_to_aeva.bin> --action mos --model models/mos_rf.pkl

# Hercules Aeva
python -m src.app --dataset hercules --bin <path_to_aeva.bin> --action mos --model models/mos_rf.pkl

# Hercules Radar
python -m src.app --dataset hercules --radar <path_to_radar.bin> --action mos --model models/mos_rf.pkl
```

Результат — два графика matplotlib:
- **Левый**: radial velocity vs azimuth (если есть Допплер) или height profile (если нет) — серые точки = static, красные = moving
- **Правый**: bird's-eye view (x, y) — те же цвета

### 3. Инференс на последовательности кадров

Использует temporal consistency через SE(3) позы для более точной сегментации:

```bash
python -m src.app --dataset helimos --action mos-sequence --sequence data/Deskewed_LiDAR --sensor Velodyne --split val --n-frames 10 --n-context 3 --model models/mos_rf.pkl
```

Параметры:
- `--n-frames` — число кадров для обработки (по умолчанию: `5`)
- `--n-context` — размер временного окна для temporal consistency (по умолчанию: `3`)

---

## Примеры

- Скрипт-пример чтения и визуализации GPS: `src/examples/gps_example.py` (использует константы из `src/config.py`).
- CLI: `src/app.py` — универсальный интерфейс для всех датасетов.

---

## Зависимости

См. `requirements.txt`. Основные пакеты:
- pandas, numpy — базовая работа с данными
- folium — интерактивные карты
- matplotlib — графики
- open3d — (опционально) визуализация облаков точек
- scikit-learn — Random Forest для MOS
- joblib — сериализация модели

Примечание: если вам не нужна визуализация облаков точек, можно пропустить установку `open3d`.

