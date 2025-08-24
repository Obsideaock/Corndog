import subprocess
import sys
import time
sys.path.insert(0, '/home/Corndog')
from lcd import lcd_library as lcd

def get_wifi_status():
    try:
        # Check if WiFi is enabled or disabled
        result = subprocess.run(['nmcli', 'radio', 'wifi'], stdout=subprocess.PIPE, text=True)
        wifi_state = result.stdout.strip()

        if wifi_state == "disabled":
            return "Wifi: Off"

        # Check WiFi connection status using 'nmcli device status'
        result = subprocess.run(['nmcli', 'device', 'status'], stdout=subprocess.PIPE, text=True)
        lines = result.stdout.strip().split("\n")

        for line in lines:
            if "wifi" in line:
                columns = line.split()
                state = columns[2]  # The third column typically contains the state

                if state == "connected":
                    # Get the name (SSID) of the WiFi network it is connected to
                    ssid_result = subprocess.run(['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'], stdout=subprocess.PIPE, text=True)
                    ssid_lines = ssid_result.stdout.strip().split("\n")

                    for ssid_line in ssid_lines:
                        if ssid_line.startswith("yes"):  # 'yes' indicates an active connection
                            ssid = ssid_line.split(":")[1]
                            return f"Wifi: Connected {ssid}"
                    
                    return "Wifi: Connected Unknown"
                elif state == "disconnected":
                    return "Wifi: Searching"

        return "Wifi: Searching"

    except Exception as e:
        return f"Error checking WiFi status: {e}"

if __name__ == "__main__":
    while True:
        wifi_status = get_wifi_status()
        lcd.lcd(wifi_status)
        print(wifi_status)  # For debugging if you want to see it in logs

        if "Connected" in wifi_status or "Off" in wifi_status:
            # Show for 30 seconds, then exit
            time.sleep(30)
            lcd.clear()
            break
        else:
            # Keep checking every 5 seconds if still searching
            time.sleep(5)
