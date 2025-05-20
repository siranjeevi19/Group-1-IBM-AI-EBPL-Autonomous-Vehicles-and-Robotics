# Autonomous Vehicle and Robotics Simulation with Smart Navigation

An interactive simulation of an autonomous vehicle navigating a grid-based environment using A* pathfinding, dynamic obstacle avoidance, battery-aware routing, and data logging — all in real-time with a user-friendly GUI built in Google Colab.

---

## Features

- **Smart Pathfinding:** A* algorithm enables efficient navigation across the environment.
- **Dynamic Obstacle Handling:** Real-time updates and random movements of obstacles add realism.
- **Battery Management:** Simulates power consumption and redirects to the nearest charging station when low.
- **Emergency Stop:** Instantly halts all vehicle activity with a single click.
- **Live Visualization:** Displays the grid, path, obstacles, and vehicle state visually using OpenCV and Matplotlib.
- **Data Logging:** Records every simulation with vehicle state, route data, battery level, and success status into a CSV file.

---

## Technologies Used

**Languages:**  
- Python 3

**Platform:**  
- Google Colab

**Libraries:**  
- `NumPy` – Array and matrix operations  
- `OpenCV` – Visualization and image manipulation  
- `Matplotlib` – Grid and state plotting  
- `ipywidgets` – UI components  
- `pandas` – Data processing and CSV logging

---

## How It Works

1. **Grid Initialization:** The environment is randomly generated with static and dynamic obstacles.
2. **User Input:** Set your destination (X, Y) through the interface and start navigation.
3. **A\* Algorithm:** Finds the shortest path to the destination or to the nearest charging station if battery is low.
4. **Battery Monitoring:** Battery drains during movement; if it drops below 20%, the vehicle reroutes to a charging station.
5. **Obstacle Avoidance:** The vehicle dynamically updates its path if an obstacle blocks the current route.
6. **Logging:** Each run's parameters and outcomes are saved to a CSV file for later analysis.

---

## Data Collection

**Dataset Source:**  
- The dataset used for this simulation was sourced from Kaggle.

**Usage in Simulation:**  
- The dataset (`dataset.csv`) can be used to simulate predefined navigation scenarios, validate behavior, and analyze vehicle performance metrics.

---

## Execution Steps

### Option 1: Google Colab

1. Open the provided Google Colab notebook (`project.ipynb`).
2. Upload the required files:
   - `dataset.csv`
   - Icon images: `car.png`, `flag.png`, `rock.png`, `battery.png`, `charging.png`
3. Run the import & upload cell to load all necessary libraries and files.
4. Run the main simulation cell to start the vehicle navigation system.
5. Use the on-screen UI to control vehicle navigation.
6. Upon simulation completion, the navigation log is automatically saved and downloaded as `navigation_log.csv`.

### Option 2: Jupyter Notebook (Local Execution)

1. Clone this repository to your local machine.
2. Ensure you have Python 3.8+ and install required dependencies mentioned in `requirements.txt`.
3. Place `dataset.csv` and the icon image files in the same directory as the notebook.
4. Open `project.ipynb` in Jupyter Notebook.
5. Run each cell in order, starting from the upload/import section.
6. The UI will appear inline, and logs will be saved to `navigation_log.csv` in your working directory.
