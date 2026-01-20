import argparse
from src.datasets.helimos import load_helimos_frame
from src.datasets.helipr import load_helipr_aeva
from src.viz.clouds import visualize_point_cloud
from src.viz.plots import plot_velocity_vs_azimuth


def main() -> None:
    parser = argparse.ArgumentParser(description="LiDAR processing CLI")
    parser.add_argument("--dataset", choices=["helimos", "helipr"], required=True)
    parser.add_argument("--bin", type=str, required=True)
    parser.add_argument("--action", choices=["cloud", "velocity"], required=True)

    args = parser.parse_args()

    if args.dataset == "helimos":
        pc = load_helimos_frame(args.bin)
        visualize_point_cloud(pc)
    elif args.dataset == "helipr":
        pc = load_helipr_aeva(args.bin)
        if args.action == "velocity":
            plot_velocity_vs_azimuth(pc)
        else:
            visualize_point_cloud(pc)


if __name__ == "__main__":
    main()