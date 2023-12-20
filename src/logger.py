#!/usr/bin/env python3
import csv
import os
import time
from datetime import datetime

class Logger:
    def __init__(self):
        home_dir = os.path.expanduser('~')
        self.state_log_dir = os.path.join(home_dir, "GPTSmach_Log", "state_log")
        self.conversation_log_dir = os.path.join(home_dir, "GPTSmach_Log", "conversation_log")
        self._create_dir(self.state_log_dir)
        self._create_dir(self.conversation_log_dir)

        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.state_filename = os.path.join(self.state_log_dir, current_time + "_state_log.csv")
        self.conversation_filename = os.path.join(self.conversation_log_dir, current_time + "_conversation_log.csv")
        
        self.state_log_data = []
        self.conversation_log_data = []
        self.human_count = 0
        self.robot_count = 0

    def _create_dir(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

    def log_state_time(self, state, start_time):
        self.state_log_data.append({"start_time": start_time, "state": state})

    def log_conversation(self, timestamp, speaker, content):
        self.conversation_log_data.append({"timestamp": timestamp, "speaker": speaker, "content": content})
        if speaker == "Human":
            self.human_count += 1
        elif speaker == "QTrobot":
            self.robot_count += 1

    def save_state_time_to_csv(self):
        self._save_to_csv(self.state_filename, self.state_log_data, ["start_time", "state"])

    def save_conversation_to_csv(self):
        self._save_to_csv(self.conversation_filename, self.conversation_log_data, ["timestamp", "speaker", "content"])

    def _save_to_csv(self, filename, data, fieldnames):
        with open(filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for entry in data:
                writer.writerow(entry)

        print("Human: {}, QTrobot: {}".format(self.human_count, self.robot_count))


if __name__ == "__main__":

    # Example usage
    logger = Logger()

    # Logging states with their start time
    greeting_start_time = time.time()
    logger.log_state_time("Greeting", greeting_start_time)

    conversation_start_time = time.time()
    logger.log_state_time("Conversation", conversation_start_time)

    # Save state log to a CSV file
    logger.save_state_time_to_csv()

    # Logging conversation
    logger.log_conversation(time.time(), "User", "Hello, robot!")
    logger.log_conversation(time.time(), "Robot", "Hello, human!")

    # Save conversation log to a CSV file
    logger.save_conversation_to_csv()
