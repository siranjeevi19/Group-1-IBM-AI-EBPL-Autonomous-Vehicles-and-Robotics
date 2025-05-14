# Group-1-IBM-AI-EBPL-Autonomous-Vehicles-and-Robotics


README: How to Run the Autonomous Vehicle Simulation


This guide walks you through the steps to run the autonomous vehicle simulation in Google Colab or Jupyter Notebook.

-------------------------------------------------------------------------------------

Step 1: Upload Required Icon Files

Before running the simulation, upload the following PNG files to your environment:
car.png
flag.png
rock.png
battery.png
charging.png

 ------------------------------------------------------------------------------------

Step 2: Install Required Library

Run the following command once to install necessary packages for the widget interface:

!pip install ipywidgets

Then, enable widget rendering 

from google.colab import output
output.enable_custom_widget_manager()


-------------------------------------------------------------------------------------


Step 3: Load All Icons

In a new code cell, paste and run this code to load the icons:

import cv2
icons = {}

for icon in ['car', 'flag', 'rock', 'battery', 'charging']:
    icons[icon] = cv2.imread(f"{icon}.png", cv2.IMREAD_UNCHANGED)
    assert icons[icon] is not None, f"Failed to load {icon}.png"


--------------------------------------------------------------------------------------


Step 4: Define Helper Functions and Classes

Run each of the following items in separate code cells:

overlay_icon(img, icon, pos, size)

PathPlanner class

AutonomousVehicle class



---------------------------------------------------------------------------------------


Step 5: Run the Simulation

Finally, in a new code cell, create an instance of the vehicle:
av = AutonomousVehicle()

This will display:

A destination input box
Start and Emergency Stop buttons
A live battery slider
Output status logs


---------------------------------------------------------------------------------------


How It Works
The car starts at position (0, 0) and travels to the destination you provide.

If battery drops below 20%, it automatically detours to the nearest charging station.

Dynamic obstacles (like rocks) may move randomly during travel.

You can stop the simulation at any time using the Emergency Stop button.
