from src.centerline_navigation import NavigationConfig, CenterlineNavigator

def main():
    config = NavigationConfig()
    navigator = CenterlineNavigator(config)
    navigator.load_point_cloud("data/example.las")
    navigator.initialize_robot(
        [193328.856628, 5102483.658905, 143.8510], # start point xyz coordinates
        [193311.138123, 5102503.181488, 144.1012], # distination point xyz coordinates
    )
    navigator.navigate(max_steps=2000)
    navigator.save_results("results/example_output.csv")


if __name__ == "__main__":
    main()
