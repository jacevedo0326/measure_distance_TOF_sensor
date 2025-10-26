#!/usr/bin/env python3
"""
ESP32 Measurement Data Downloader
Captures CSV data from ESP32 over serial and saves to file
"""

import serial
import time
import sys
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd

# Configuration
SERIAL_PORT = 'COM3'  # Change this to your ESP32's COM port (Windows) or /dev/ttyUSB0 (Linux)
BAUD_RATE = 115200
TIMEOUT = 10  # seconds

def find_csv_in_serial(ser):
    """Read serial output and extract CSV data between markers"""
    print("Waiting for CSV data from ESP32...")
    print("Trigger GPIO 6 on your ESP32 now!\n")

    csv_started = False
    csv_lines = []
    start_time = time.time()

    while True:
        if time.time() - start_time > TIMEOUT:
            print(f"Timeout after {TIMEOUT} seconds. No data received.")
            return None

        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Print all output for debugging
            if line and not line.startswith('==='):
                print(f"ESP32: {line}")

            # Check for CSV start marker
            if '===CSV_START===' in line:
                print("\n📥 CSV data download started...")
                csv_started = True
                csv_lines = []
                continue

            # Check for CSV end marker
            if '===CSV_END===' in line:
                print("✅ CSV data download complete!\n")
                return csv_lines

            # Collect CSV lines
            if csv_started:
                csv_lines.append(line)

def save_csv_to_file(csv_lines, filename=None):
    """Save CSV lines to a file"""
    if not csv_lines:
        print("No data to save!")
        return None

    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"measurements_{timestamp}.csv"

    with open(filename, 'w', newline='') as f:
        for line in csv_lines:
            f.write(line + '\n')

    print(f"💾 Data saved to: {filename}")
    print(f"📊 Total rows: {len(csv_lines) - 1}")  # -1 for header

    return filename

def create_plot(csv_filename):
    """Create a plot of the percentage data over time"""
    try:
        # Read the CSV file
        df = pd.read_csv(csv_filename)

        # Calculate time in seconds (100ms = 0.1s per measurement)
        df['Time_s'] = (df['Index'] - 1) * 0.1

        # Create the plot
        plt.figure(figsize=(12, 6))
        plt.plot(df['Time_s'], df['Percentage'], 'b-', linewidth=1.5)

        # Styling
        plt.xlabel('Time (seconds)', fontsize=12)
        plt.ylabel('Percentage (%)', fontsize=12)
        plt.title('Pedal Position Over Time', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.ylim(0, 105)  # 0-100% with some margin

        # Save the plot
        plot_filename = csv_filename.replace('.csv', '_plot.png')
        plt.tight_layout()
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"📈 Plot saved to: {plot_filename}")

        # Show the plot
        plt.show()

    except Exception as e:
        print(f"❌ Error creating plot: {e}")
        print("Make sure matplotlib and pandas are installed:")
        print("  pip install matplotlib pandas")

def main():
    print("=" * 50)
    print("ESP32 Measurement Data Downloader")
    print("=" * 50)
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud: {BAUD_RATE}")
    print("=" * 50)

    try:
        # Open serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize

        print("✅ Serial connection established\n")

        # Find and extract CSV data
        csv_data = find_csv_in_serial(ser)

        if csv_data:
            csv_filename = save_csv_to_file(csv_data)

            # Display first few rows
            print("\n📋 Preview (first 10 rows):")
            print("-" * 40)
            for line in csv_data[:11]:  # Header + 10 rows
                print(line)
            if len(csv_data) > 11:
                print(f"... ({len(csv_data) - 11} more rows)")

            # Create plot
            if csv_filename:
                print("\n📊 Creating plot...")
                create_plot(csv_filename)
        else:
            print("❌ No CSV data received. Make sure to trigger GPIO 6 on your ESP32.")

        ser.close()

    except serial.SerialException as e:
        print(f"❌ Serial port error: {e}")
        print(f"\nTip: Make sure {SERIAL_PORT} is the correct port.")
        print("You can find your COM port in Device Manager (Windows) or by running:")
        print("  Windows: mode")
        print("  Linux/Mac: ls /dev/tty*")
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
