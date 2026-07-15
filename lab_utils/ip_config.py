import re, subprocess

def get_ip():
    try:
        # Run the ifconfig command and capture its output
        result = subprocess.run(['ifconfig'], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError("Error running ifconfig command.")

        # Extract IP addresses starting with '192.168.0' using regular expression
        ip_address = re.findall(r'inet (192\.168\.0\.\d+)', result.stdout)
        print(ip_address[0])
        return ip_address[0]
    except Exception as e:
        print("Error:", e)
        return []