import csv
import os
import os.path as osp

GPT_DEMO_PATH = osp.dirname(osp.dirname(osp.dirname(osp.abspath(__file__))))
DATA_PATH = osp.join(GPT_DEMO_PATH, 'data')

def create_csv_file(filename):
    filename = osp.join(DATA_PATH, filename)
    # Create a new CSV file with headers
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['API Time', 'Token Length'])

def load_csv_file(filename):
    filename = osp.join(DATA_PATH, filename)
    if not osp.exists(filename):
        create_csv_file(filename)
    data = []
    try:
        # Load existing data from the CSV file
        with open(filename, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                data.append((float(row['API Time']), int(row['Token Length'])))
    except FileNotFoundError:
        pass
    return data

def save_csv_file(filename, data):
    filename = osp.join(DATA_PATH, filename)
    # Save data to the CSV file
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        for api_time, token_length in data:
            writer.writerow([api_time, token_length])
