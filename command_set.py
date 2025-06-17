import serial
import time
import sys
import re 
import struct 
import numpy as np # For FFT and numerical operations
import matplotlib.pyplot as plt
import os # For creating directory
from datetime import datetime # For timestamp in filename
import argparse # For command-line argument parsing

# --- Configuration ---
SERIAL_PORT = 'COM4'  # Windows PC専用、COM4に固定 (ファイルモードでは使用されない)
BAUD_RATE = 115200    # (ファイルモードでは使用されない)
SAMPLING_FREQUENCY_HZ = 16000 # サンプリング周波数

# STM32側のAUDIO_BUFFER_SIZE_BYTESに合わせてください (シリアル通信モード時)
STM32_EXPECTED_AUDIO_BUFFER_SIZE_BYTES = (300 * 1024) # 例: 300KB

DFSDM_PYTHON_PARAMS = { 
    "SINC_ORDER": 3,
    "FILTER_OVERSAMPLING": 125,
    "INTEGRATOR_OVERSAMPLING": 1,
    "CHANNEL_OUTPUT_CLOCK_DIVIDER": 54,
    "RIGHT_BIT_SHIFT": 2 # STM32 Cコードの g_dfsdm_right_bit_shift と合わせる
}
DATA_SAVE_SUBFOLDER = "audio_data" 
SAVE_BINARY_DATA = True # シリアル通信モード時: Trueで保存、Falseで保存しない

# --- Helper Functions ---

def send_stm_command(ser, command_str):
    """STM32デバイスにコマンド文字列を送信します。"""
    if ser is None or not ser.is_open:
        print("Serial port is not open.")
        return
    try:
        print(f"PC -> STM32: {command_str}")
        ser.write(command_str.encode('ascii') + b'\r\n') 
        time.sleep(0.1) 
    except serial.SerialException as e:
        print(f"Error writing to serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred during send: {e}")

def plot_audio_data_subplots(audio_data_samples, sample_rate, right_bit_shift, unpack_format_title="", input_filename=""):
    """受信または読み込んだ音声データを3行1列のグラフで表示します。"""
    if not audio_data_samples:
        print("No data to plot.")
        return

    # DCオフセット除去を行うかどうかのフラグ (必要に応じてTrueに変更)
    REMOVE_DC_OFFSET = True 
    if REMOVE_DC_OFFSET and len(audio_data_samples) > 0:
        mean_offset = np.mean(audio_data_samples)
        print(f"INFO: Calculated DC offset: {mean_offset:.2f}. Removing for plotting.")
        audio_data_samples_plot = np.array(audio_data_samples) - mean_offset
    else:
        audio_data_samples_plot = np.array(audio_data_samples)
    
    num_samples_total = len(audio_data_samples_plot)
    time_axis = np.arange(num_samples_total) / sample_rate
    
    if right_bit_shift < 0 or right_bit_shift > 23: 
        print(f"Warning: Invalid right_bit_shift value ({right_bit_shift}). Using default (2^23-1) for dBFS scaling.")
        effective_full_scale = (2**23) -1 
    else:
        effective_full_scale = ( (2**23) - 1 ) >> right_bit_shift
    
    print(f"INFO: Using effective full scale for dBFS: {effective_full_scale} (based on 24-bit signed, right shifted by {right_bit_shift})")

    plot_title_prefix = f"Audio Data from {os.path.basename(input_filename)}" if input_filename else "Received Audio Data"

    fig, axs = plt.subplots(3, 1, figsize=(12, 12)) 

    # --- 1. Time Domain Waveform (Overall) ---
    axs[0].plot(time_axis, audio_data_samples_plot)
    axs[0].set_title(f"{plot_title_prefix} - Time Domain Waveform{unpack_format_title}")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Amplitude (Digital Value)")
    axs[0].grid(True)

    # --- 2. Zoomed Time Domain Waveform (First 100 Samples) ---
    num_zoom_samples = min(100, num_samples_total) 
    if num_zoom_samples > 0:
        axs[1].plot(time_axis[:num_zoom_samples], audio_data_samples_plot[:num_zoom_samples])
        axs[1].set_title(f"Zoomed Time Domain (First {num_zoom_samples} Samples)")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Amplitude (Digital Value)")
        axs[1].grid(True)
    else:
        axs[1].set_title("Zoomed Time Domain Waveform (No data for zoom)")

    # --- 3. Frequency Spectrum (FFT) - dBFS with Peak Marking & Hanning Window ---
    if num_samples_total > 0:
        N = num_samples_total
        window = np.hanning(N)
        # audio_data_samples_plot は既にnumpy array
        yf = np.fft.fft(audio_data_samples_plot * window) 
        xf = np.fft.fftfreq(N, 1 / sample_rate)[:N//2] 
        
        sum_of_window_coeffs = np.sum(window)
        if sum_of_window_coeffs == 0: 
            print("Warning: Sum of window coefficients is zero. Using N for scaling.")
            sum_of_window_coeffs = N 
        
        magnitude_linear = (2.0 / sum_of_window_coeffs) * np.abs(yf[0:N//2])
        
        magnitude_dbfs = np.full_like(magnitude_linear, -200.0) 
        non_zero_indices = magnitude_linear > 0 
        if effective_full_scale > 0: 
            safe_magnitude_linear_for_log = np.where(magnitude_linear[non_zero_indices] > 1e-9, 
                                                     magnitude_linear[non_zero_indices], 
                                                     1e-9) 
            magnitude_dbfs[non_zero_indices] = 20 * np.log10(safe_magnitude_linear_for_log / effective_full_scale)
        else:
            print("Warning: effective_full_scale is zero, cannot calculate dBFS correctly.")

        axs[2].plot(xf, magnitude_dbfs)
        axs[2].set_title("Frequency Spectrum (FFT in dBFS with Hanning Window)")
        axs[2].set_xlabel("Frequency (Hz)")
        axs[2].set_ylabel("Magnitude (dBFS)")
        axs[2].grid(True)
        
        if np.any(np.isfinite(magnitude_dbfs)): 
            min_db_plot = np.min(magnitude_dbfs[np.isfinite(magnitude_dbfs)])
            max_db_plot = np.max(magnitude_dbfs[np.isfinite(magnitude_dbfs)])
            axs[2].set_ylim(max(min_db_plot -10 , -120), min(max_db_plot + 10, 6) ) 

            if len(magnitude_dbfs) > 0 and np.any(np.isfinite(magnitude_dbfs)): 
                valid_peak_indices = np.isfinite(magnitude_dbfs)
                if np.any(valid_peak_indices): 
                    peak_index_in_valid = np.argmax(magnitude_dbfs[valid_peak_indices])
                    peak_index = np.where(valid_peak_indices)[0][peak_index_in_valid]

                    peak_freq = xf[peak_index]
                    peak_mag_dbfs = magnitude_dbfs[peak_index]
                    axs[2].plot(peak_freq, peak_mag_dbfs, "x", markersize=10, color='red', label=f"Peak: {peak_mag_dbfs:.2f} dBFS @ {peak_freq:.1f} Hz")
                    axs[2].annotate(f"Peak: ({peak_freq:.1f} Hz, {peak_mag_dbfs:.2f} dBFS)",
                                    xy=(peak_freq, peak_mag_dbfs),
                                    xytext=(peak_freq + (xf[-1]*0.05 if len(xf)>1 and xf[-1] > 0 else 50), peak_mag_dbfs - 5), 
                                    arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=5),
                                    fontsize=9,
                                    bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.7))
                    axs[2].legend(fontsize='small', loc='upper right') 
        else:
            axs[2].set_ylim(-120, 6) 

        if len(xf) > 1 and xf[-1] > 0 : 
             axs[2].set_xlim(0, xf[-1]) 
    else:
        axs[2].set_title("Frequency Spectrum (FFT in dBFS with Hanning Window) (No data)")

    plt.tight_layout() 
    plt.show()


def listen_for_data(ser, expected_binary_size_ref=None, binary_timeout_seconds=20, text_timeout_seconds=2.0):
    """
    STM32からの応答を待ち受けます。
    テキストメッセージと期待されるバイナリデータブロックの両方を処理します。
    """
    if ser is None or not ser.is_open:
        print("Serial port is not open.")
        return None, [] 

    text_responses = []
    binary_data_buffer = bytearray()
    receiving_binary = False
    bytes_to_receive = 0
    initial_data_wait_start_time = time.time() 

    print(f"Waiting for data (Text Timeout: {text_timeout_seconds}s, Binary Timeout: {binary_timeout_seconds}s after header detection)...")

    try:
        while True:
            current_loop_time = time.time()

            if receiving_binary:
                if (current_loop_time - initial_data_wait_start_time) > binary_timeout_seconds:
                    print("Binary data reception timed out (overall).")
                    break
            else:
                if (current_loop_time - initial_data_wait_start_time) > text_timeout_seconds and not ser.in_waiting:
                    if not text_responses and not binary_data_buffer:
                         print(f"(No new text responses from STM32 within {text_timeout_seconds}s)")
                    break 

            if ser.in_waiting > 0:
                initial_data_wait_start_time = time.time() 

                if not receiving_binary:
                    try:
                        line_bytes = ser.readline()
                        line = line_bytes.decode('ascii', errors='replace').strip()
                    except UnicodeDecodeError:
                        print(f"STM32 -> PC (RAW): {line_bytes!r} (UnicodeDecodeError)")
                        text_responses.append(f"Error: UnicodeDecodeError with data: {line_bytes!r}")
                        continue

                    if line:
                        print(f"STM32 -> PC (TEXT): {line}")
                        text_responses.append(line)
                        
                        match = re.search(r"Sending (\d+) bytes of audio data", line)
                        if match:
                            bytes_to_receive = int(match.group(1))
                            if expected_binary_size_ref is not None and bytes_to_receive != expected_binary_size_ref:
                                print(f"WARNING: STM32 announced {bytes_to_receive} bytes, but script expected {expected_binary_size_ref} bytes.")
                            print(f"INFO: Detected binary data header. Expecting {bytes_to_receive} bytes.")
                            receiving_binary = True
                            binary_data_buffer.clear()
                            initial_data_wait_start_time = time.time() 
                            continue 
                
                else: # receiving_binary is True
                    bytes_available = ser.in_waiting
                    bytes_remaining = bytes_to_receive - len(binary_data_buffer)
                    read_size = min(bytes_available, bytes_remaining)
                    
                    if read_size > 0:
                        chunk = ser.read(read_size)
                        binary_data_buffer.extend(chunk)
                        initial_data_wait_start_time = time.time() 

                    if len(binary_data_buffer) >= bytes_to_receive:
                        print(f"INFO: Successfully received {len(binary_data_buffer)} bytes of binary data (expected {bytes_to_receive}).")
                        receiving_binary = False 
                        return bytes(binary_data_buffer), text_responses 
            else:
                time.sleep(0.01)
            
            if not receiving_binary and (time.time() - initial_data_wait_start_time > text_timeout_seconds):
                 if not text_responses and not binary_data_buffer and not ser.in_waiting:
                    break 

        if receiving_binary and len(binary_data_buffer) < bytes_to_receive:
            print(f"WARNING: Binary data reception timed out. Received {len(binary_data_buffer)}/{bytes_to_receive} bytes.")
            return bytes(binary_data_buffer), text_responses

        if not text_responses and not binary_data_buffer: 
             print(f"(No data received within overall timeout)")

    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred during listen: {e}")
    
    return None, text_responses

def load_and_process_binary_file(filepath):
    """指定されたバイナリファイルを読み込み、音声サンプルに変換します。"""
    audio_samples_from_file = []
    unpack_format_file = '<i' # ファイルデータも同様の形式と仮定
    num_bytes_per_sample_file = 4

    try:
        with open(filepath, "rb") as f:
            binary_data_from_file = f.read()
        print(f"Successfully read {len(binary_data_from_file)} bytes from {filepath}")

        if len(binary_data_from_file) % num_bytes_per_sample_file != 0:
            print(f"WARNING: File data length ({len(binary_data_from_file)}) is not a multiple of bytes per sample ({num_bytes_per_sample_file}).")
        
        num_samples_in_file = len(binary_data_from_file) // num_bytes_per_sample_file
        for i in range(num_samples_in_file):
            sample_bytes = binary_data_from_file[i*num_bytes_per_sample_file : (i+1)*num_bytes_per_sample_file]
            try:
                sample_value = struct.unpack(unpack_format_file, sample_bytes)[0]
                audio_samples_from_file.append(sample_value)
            except struct.error as e:
                print(f"Error unpacking sample {i} from file: {e}, bytes: {sample_bytes!r}")
                audio_samples_from_file.append(0)
        print(f"Converted {len(audio_samples_from_file)} audio samples from file (format: {unpack_format_file}).")
        return audio_samples_from_file
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None
    except Exception as e:
        print(f"Error reading or processing file {filepath}: {e}")
        return None

# --- Main Script ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="STM32 DFSDM Data Receiver and Plotter. Either connects to STM32 or plots data from a .bin file.")
    parser.add_argument("-f", "--file", type=str, help="Path to a .bin file to plot. If provided, serial communication is skipped.")
    args = parser.parse_args()

    audio_samples = []
    unpack_format_used = "<i" # Default unpack format

    if args.file:
        print(f"File mode: Plotting data from {args.file}")
        audio_samples = load_and_process_binary_file(args.file)
        if audio_samples is None:
            print("Could not load or process data from file. Exiting.")
            sys.exit(1)
        # ファイルモードの場合、ファイル名からタイトルを生成
        plot_title_suffix = f" (Format: {unpack_format_used})"
        input_filename_for_plot = args.file

    else:
        print("Serial mode: Connecting to STM32...")
        # Create data saving subfolder if it doesn't exist and SAVE_BINARY_DATA is True
        if SAVE_BINARY_DATA and not os.path.exists(DATA_SAVE_SUBFOLDER):
            try:
                os.makedirs(DATA_SAVE_SUBFOLDER)
                print(f"Created subfolder for saving data: ./{DATA_SAVE_SUBFOLDER}")
            except OSError as e:
                print(f"Error creating subfolder ./{DATA_SAVE_SUBFOLDER}: {e}")
                # Fallback to current directory if subfolder creation fails and saving is enabled
                # DATA_SAVE_SUBFOLDER = "." 

        ser_conn = None
        try:
            ser_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
            print(f"Successfully connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port {SERIAL_PORT}: {e}")
            sys.exit(1)

        print("\n--- Reading initial messages from STM32 (up to 5 seconds) ---")
        _, initial_texts = listen_for_data(ser_conn, text_timeout_seconds=5.0)

        print("\n--- Sending START_LISTEN command ---")
        send_stm_command(ser_conn, "START_LISTEN")
        
        print(f"\n--- Listening for STM32 responses and audio data (expecting up to {STM32_EXPECTED_AUDIO_BUFFER_SIZE_BYTES} bytes of binary data) ---")
        
        received_binary_data, text_messages_during_listen = listen_for_data(
            ser_conn,
            expected_binary_size_ref=STM32_EXPECTED_AUDIO_BUFFER_SIZE_BYTES,
            binary_timeout_seconds=30.0, 
            text_timeout_seconds=3.0     
        )

        if received_binary_data:
            print(f"\n--- Binary Data Reception Summary ---")
            print(f"Total binary bytes received: {len(received_binary_data)}")
            
            announced_size = STM32_EXPECTED_AUDIO_BUFFER_SIZE_BYTES 
            for msg in text_messages_during_listen: 
                match = re.search(r"Sending (\d+) bytes of audio data", msg)
                if match:
                    announced_size = int(match.group(1))
                    print(f"STM32 announced it would send: {announced_size} bytes.")
                    break
            
            if len(received_binary_data) == announced_size:
                print(f"SUCCESS: Received binary data size matches the size announced by STM32 ({announced_size} bytes).")
                
                output_filename_for_save = ""
                if SAVE_BINARY_DATA:
                    save_path_base = DATA_SAVE_SUBFOLDER
                    # Ensure save_path_base exists, especially if fallback was used
                    if save_path_base != "." and not os.path.exists(save_path_base): 
                        try:
                            os.makedirs(save_path_base)
                            print(f"Re-attempted creating subfolder: ./{save_path_base}")
                        except OSError as e:
                             print(f"Error re-creating subfolder ./{save_path_base}: {e}. Saving to current directory.")
                             save_path_base = "."

                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    output_filename_for_save = os.path.join(save_path_base, f"{timestamp}_received_audio.bin")
                    
                    try:
                        with open(output_filename_for_save, "wb") as f:
                            f.write(received_binary_data)
                        print(f"Binary data saved to: {output_filename_for_save}")
                    except IOError as e:
                        print(f"Error saving binary data to file: {e}")
                else:
                    print("INFO: Binary data saving is disabled by SAVE_BINARY_DATA flag.")

                try:
                    num_bytes_per_sample = 4 
                    unpack_format = '<i'    
                    unpack_format_used = unpack_format

                    if len(received_binary_data) % num_bytes_per_sample != 0:
                        print(f"WARNING: Received binary data length ({len(received_binary_data)}) is not a multiple of bytes per sample ({num_bytes_per_sample}).")
                    
                    num_samples_to_convert = len(received_binary_data) // num_bytes_per_sample
                    for i in range(num_samples_to_convert):
                        sample_bytes = received_binary_data[i*num_bytes_per_sample : (i+1)*num_bytes_per_sample]
                        try:
                            sample_value = struct.unpack(unpack_format, sample_bytes)[0]
                            audio_samples.append(sample_value)
                        except struct.error as e:
                            print(f"Error unpacking sample {i}: {e}, bytes: {sample_bytes!r}")
                            audio_samples.append(0) 
                    print(f"Converted to {len(audio_samples)} audio samples (format: {unpack_format_used}).")
                    input_filename_for_plot = output_filename_for_save if SAVE_BINARY_DATA and output_filename_for_save else "Live Data"


                except Exception as e:
                     print(f"Error during data conversion for plotting: {e}")

            else:
                print(f"WARNING: Received binary data size ({len(received_binary_data)}) does NOT match the size announced by STM32 ({announced_size} bytes).")
        else:
            print("\n--- No complete binary data block was received based on header detection. ---")

        print(f"\n--- Text messages received during/after START_LISTEN ---")
        if text_messages_during_listen:
            for msg in text_messages_during_listen:
                print(f"STM32 (Log): {msg}")
        else:
            print("(No additional text messages)")

        if ser_conn and ser_conn.is_open:
            ser_conn.close()
            print(f"\nDisconnected from {SERIAL_PORT}.")
    
    # Plot data if any samples were obtained (either from serial or file)
    if audio_samples:
        print("\nPlotting audio data...")
        right_bit_shift_from_config = DFSDM_PYTHON_PARAMS.get("RIGHT_BIT_SHIFT", 0) 
        plot_audio_data_subplots(audio_samples, 
                                 sample_rate=SAMPLING_FREQUENCY_HZ, 
                                 right_bit_shift=right_bit_shift_from_config,
                                 unpack_format_title=f" (Format: {unpack_format_used})",
                                 input_filename=input_filename_for_plot if 'input_filename_for_plot' in locals() else "Unknown Source") 
    else:
        print("\nNo audio samples to plot.")

    print("\nScript finished.")
