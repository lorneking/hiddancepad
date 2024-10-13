import re
import csv
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Function to select a file using a file dialog
def select_log_file():
    root = Tk()
    root.withdraw()  # Hide the main window
    log_file = askopenfilename(title="Select the PuTTY Log File", filetypes=[("Log Files", "*.log")])
    return log_file

# Initialize variables
data = []

# Regular expression to match touchpad values (T1: [value], etc.)
pattern = re.compile(r"T(\d+): \[(\d+)\]")

# Allow the user to select the log file
input_file = select_log_file()

if input_file:
    # Define the output CSV file (same directory, different extension)
    output_file = input_file.replace('.log', '.csv')

    # Read the input log file
    with open(input_file, 'r') as file:
        for line in file:
            # Find all touchpad values in the line
            matches = pattern.findall(line)
            if matches:
                # Extract the values for each touchpad
                values = [value for _, value in matches]
                # Append the values to the data list
                data.append(values)

    # Write the data to a CSV file with an index column
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write the header (Index, T1, T2, ..., T14)
        writer.writerow(['Index'] + [f"T{i}" for i in range(1, 15)])
        
        # Write the data rows with an index
        for idx, row in enumerate(data, start=1):
            writer.writerow([idx] + row)

    print(f"Data with index has been written to {output_file}")
else:
    print("No file was selected.")
