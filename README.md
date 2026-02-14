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
# Пример: визуализация GPS
python src/app.py --dataset gps --gps 03_Day/sensor_data/gps.csv --action map
```

3. После успешного запуска будет создана HTML-карта (по умолчанию `gps_map.html` или путь из конфига).

---

## Запуск для разных датасетов

- HeLiMOS (KITTI-подобный бинарный формат):

```bash
python src/app.py --dataset helimos --bin <path_to_bin> --action cloud
```

- HeLiPR (несколько форматов LiDAR, пример Aeva):

```bash
python src/app.py --dataset helipr --bin <path_to_bin> --action cloud
# или для отображения графика азимута от радиальной скорости
python src/app.py --dataset helipr --bin <path_to_bin> --action velocity
```

- GPS (CSV):

```bash
python src/app.py --dataset gps --gps 03_Day/sensor_data/gps.csv --action map
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

Примечание: если вам не нужна визуализация облаков точек, можно пропустить установку `open3d`.

