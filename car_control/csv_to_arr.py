import csv

class CSVConverter:

    def __init__(self, file_path):

        self.file_path = file_path

    def to_array(self):

        array = []
        try:
            with open(self.file_path, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    array.append(row)
            return array
        except FileNotFoundError:
            print(f"Error: File not found at {self.file_path}")
            return []
        except Exception as e:
            print(f"An error occurred: {e}")
            return []