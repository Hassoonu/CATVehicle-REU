import xml.etree.ElementTree as ET
import subprocess

def change_time_delay(sumo_config_file, new_delay):
    # Parse the .sumocfg file and modify the time delay
    tree = ET.parse(sumo_config_file)
    root = tree.getroot()

    for time_elem in root.iter('time'):
        time_elem.find('begin').attrib['value'] = str(new_delay)

    # Save the modified .sumocfg file
    tree.write(sumo_config_file)

def run_sumo_simulation(sumo_config_file):
    # Run the SUMO simulation with the modified config file
    subprocess.run(["sumo-gui", "-c", sumo_config_file])

if __name__ == "__main__":
    sumo_config_file = "path/to/your/sumo_config.sumocfg"
    new_delay = 0  # Set the new time delay you want

    change_time_delay(sumo_config_file, new_delay)
    run_sumo_simulation(sumo_config_file)
