import sys
import os
import argparse

# Добавляем родительскую директорию в sys.path для импортов
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import GPS_MAP_FILE


def main() -> None:
    parser = argparse.ArgumentParser(description="LiDAR processing CLI")
    parser.add_argument("--dataset", choices=["helimos", "helipr", "gps"], required=True)
    parser.add_argument("--bin", type=str, required=False)
    parser.add_argument("--gps", type=str, required=False, help="Путь к GPS CSV файлу")
    parser.add_argument("--action", choices=["cloud", "velocity", "map"], required=True)
    parser.add_argument("--output", type=str, required=False, default=GPS_MAP_FILE, 
                       help=f"Путь для сохранения HTML карты GPS (по умолчанию: {GPS_MAP_FILE})")

    args = parser.parse_args()

    if args.dataset == "gps":
        # GPS обработка - импортируем только необходимые модули
        from src.io.csv_reader import read_GPS
        from src.viz.plots import plot_gps_on_map
        
        if not args.gps:
            print("Ошибка: для GPS требуется указать --gps <путь_к_файлу>")
            return
        
        if args.action == "map":
            gps_df = read_GPS(args.gps)
            if gps_df is not None:
                plot_gps_on_map(gps_df, args.output)
        else:
            print(f"Ошибка: для GPS поддерживается только action='map'")
    
    elif args.dataset == "helimos":
        # Импортируем модули для helimos только если нужны
        from src.datasets.helimos import load_helimos_frame
        from src.viz.clouds import visualize_point_cloud
        
        if not args.bin:
            print("Ошибка: для helimos требуется указать --bin <путь_к_файлу>")
            return
        pc = load_helimos_frame(args.bin)
        visualize_point_cloud(pc)
    
    elif args.dataset == "helipr":
        # Импортируем модули для helipr только если нужны
        from src.datasets.helipr import load_helipr_aeva
        from src.viz.clouds import visualize_point_cloud
        from src.viz.plots import plot_velocity_vs_azimuth
        
        if not args.bin:
            print("Ошибка: для helipr требуется указать --bin <путь_к_файлу>")
            return
        pc = load_helipr_aeva(args.bin)
        if args.action == "velocity":
            plot_velocity_vs_azimuth(pc)
        else:
            visualize_point_cloud(pc)


if __name__ == "__main__":
    main()