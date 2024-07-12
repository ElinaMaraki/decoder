import tkinter as tk
from tkinter import filedialog, scrolledtext, messagebox
import subprocess
import pandas as pd


def select_file():
    # Prompt user to select input text file
    file = filedialog.askopenfile(mode='r', filetypes=[("Text files", "*.txt")])
    if file:
        # Get the selected file path
        input_file_path = file.name
        entry_input_file.delete(0, tk.END)  # Clear previous entry
        entry_input_file.insert(0, input_file_path)  # Set selected file path in entry widget
        file.close()  # Close the file object


def run_cpp_program(input_file_path, threshold_value, output_file_name):

    # Validate threshold value
    try:
        threshold_value = int(threshold_value)
    except ValueError:
        messagebox.showerror("Error", "Invalid threshold value. Please enter an integer.")
        return

    # Check if input file path is provided
    if not input_file_path:
        messagebox.showerror("Error", "Please select an input file.")
        return

    # Add .csv extension to output file name
    output = output_file_name + ".csv"

    command = ["./Parser.exe", input_file_path, str(threshold_value), output]

    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        while True:
            output_line = process.stdout.readline()
            if not output_line:
                break
            text_widget.insert(tk.END, output_line)
            text_widget.see(tk.END)  # Scroll to the end of the text widget
        process.wait()  # Wait for the process to finish
        text_widget.insert(tk.END, "CSV created.\n")
    except Exception as e:
        text_widget.insert(tk.END, f"Error running C++ program: {str(e)}\n")


# Create GUI window
root = tk.Tk()
root.title("Log Decoder")

# Set background color to black
root.configure(bg="black")

# Button to select input file
button_select_file = tk.Button(root, text="Select File", command=select_file, bg="white", fg="black")
button_select_file.pack(pady=10)

# Entry widget to display selected input file path
entry_input_file = tk.Entry(root, width=50, bg="white", fg="black")
entry_input_file.pack(pady=5)

# Add entry for threshold value
label_threshold = tk.Label(root, text="Threshold Value:", bg="black", fg="white")
label_threshold.pack()
entry_threshold = tk.Entry(root, bg="white", fg="black")
entry_threshold.pack(pady=5)

# Add entry for output file name
label_output_file = tk.Label(root, text="Output File Name:", bg="black", fg="white")
label_output_file.pack()
entry_output_file = tk.Entry(root, bg="white", fg="black")
entry_output_file.pack(pady=5)

# Button to run the C++ program
button_run_program = tk.Button(root, text="Run Program", bg="green", fg="white", command=lambda: run_cpp_program(
    entry_input_file.get(), entry_threshold.get(), entry_output_file.get()))
button_run_program.pack(pady=20)

# Add a title label above the Text widget
title_label = tk.Label(root, text="Decoder Output", font=("Helvetica", 12, "bold"), bg="black", fg="green")
title_label.pack(pady=10)

# Create a Text widget to display output
text_widget = tk.Text(root, width=80, height=20, bg="black", fg="white")
text_widget.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)  # Use fill and expand to fill the window

# Create a vertical scrollbar and associate it with the Text widget
scrollbar = tk.Scrollbar(text_widget, command=text_widget.yview)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
text_widget.config(yscrollcommand=scrollbar.set)

# Configure the Text widget to use the scrollbar
text_widget.config(yscrollcommand=scrollbar.set)

# Start GUI event loop
root.mainloop()
