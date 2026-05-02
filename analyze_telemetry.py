import re
import matplotlib.pyplot as plt
import sys

def parse_telemetry(file_path):
    timestamps = []
    powers = []
    yaws = []
    accels = []
    impacts = []
    states = []
    
    # Pattern to match: [   1.24 s] [M:0 W:0] [S:10000 L:00] [State:2] Pwr: 100 | Yaw:  90.2 | Ax: 0.1G | Impact:SAFE
    pattern = re.compile(r"\[\s*(?P<ts>[\d\.]+)\s*s\].*?\[S:(?P<s_bits>[01]{5})\s+L:(?P<lr>\d)(?P<ll>\d)\].*?Pwr:\s*(?P<pwr>-?\d+).*?Yaw:\s*(?P<yaw>[\d\.-]+).*?Ax:\s*(?P<ax>[\d\.-]+)G")
    
    with open(file_path, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                timestamps.append(float(match.group('ts')))
                powers.append(int(match.group('pwr')))
                yaws.append(float(match.group('yaw')))
                accels.append(float(match.group('ax')))
                
                # Digital Sensors: S:10000 -> 1 if center seen
                s_bits = match.group('s_bits')
                dists.append(1.0 if s_bits[0] == '1' else 0.0) # We'll plot if Center sees something
                
                # Line sensor triggers
                line_hits.append(1.0 if (int(match.group('lr')) or int(match.group('ll'))) else 0.0)
                
                # Check for hits in the raw line text
                impacts.append(1.0 if "!!HIT!!" in line else 0.0)

    return timestamps, powers, yaws, accels, impacts, dists, line_hits

def plot_data(ts, pwr, yaw, ax, hits, center_seen, line_hits):
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    
    # Subplot 1: Power, Impacts, and Line Triggers
    ax1.plot(ts, pwr, label='Motor Power (%)', color='blue', alpha=0.7)
    ax1.fill_between(ts, 0, [h*100 for h in hits], color='red', alpha=0.3, label='Physical Impact')
    ax1.fill_between(ts, -100, [lh*-100 for lh in line_hits], color='orange', alpha=0.3, label='Line Detected')
    ax1.set_ylabel('Power / Events')
    ax1.set_title('Robot Performance Analysis')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Subplot 2: Digital Object Detection (Center)
    ax2.step(ts, center_seen, label='Center Sensor (Sees Object)', color='brown', where='post')
    ax2.set_ylabel('Object Detected')
    ax2.set_ylim(-0.1, 1.1)
    ax2.set_yticks([0, 1])
    ax2.set_yticklabels(['OFF', 'ON'])
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # Subplot 3: Yaw (Orientation)
    ax3.plot(ts, yaw, label='Yaw (Degrees)', color='green')
    ax3.set_ylabel('Yaw')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)

    # Subplot 4: Forward Acceleration (G)
    ax4.plot(ts, ax, label='Forward Accel (G)', color='purple')
    ax4.set_ylabel('Accel (G)')
    ax4.set_xlabel('Time (seconds)')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    log_file = "robot_telemetry.log"
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
        
    try:
        # Pre-initialize dists and line_hits to avoid unpack errors
        timestamps = []
        powers = []
        yaws = []
        accels = []
        impacts = []
        dists = []
        line_hits = []
        
        data = parse_telemetry(log_file)
        if not data[0]:
            print(f"No valid data found in {log_file}. Check your format!")
        else:
            plot_data(*data)
    except FileNotFoundError:
        print(f"Error: {log_file} not found. Run 'socat' first to generate logs.")
