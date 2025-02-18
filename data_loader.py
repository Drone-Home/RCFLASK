# Loads data from file on start
# Persistent data:
# Last drone location
# Current mode (Auto manual)
import json
import os

class DataLoader:
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.data = {}

    def save(self):
        # Save all instance variables to a file
        with open(self.file_path, "w") as file:
            json.dump(self.__dict__, file, indent=4)
    
    def load(self):
        # Load all instance variables from a file
        if os.path.exists(self.file_path):
            with open(self.file_path, "r") as file:
                self.__dict__.update(json.load(file))
    
    def set_data(self, key, value):
        # Set a key-value pair in the data dictionary
        self.data[key] = value
    
    def get_data(self, key):
        # Retrieve a value from the data dictionary
        return self.data.get(key, None)
    
    def get_dict(self):
        return self.data


if __name__ == "__main__":
    # tests
    file_path = "/workspaces/drone_home-dev/drone-home/ros2_ws/data.json"
    data_loader = DataLoader(file_path)

    # Initialize DataLoader and load existing data
    data_loader = DataLoader(file_path)
    data_loader.load()
    
    # Set new data and save it
    #data_loader.set_data("username", "test_user")
    #data_loader.set_data("score", 100)
    #data_loader.save()
    
    # Load data and print it
    print(data_loader.get_data("username"))  # Output: test_user
    print(data_loader.get_data("score"))  # Output: 100
