import json
import sys
from pathlib import Path


def load_telemetry(path):
    with open(path, "r", encoding="utf-8") as file:
        entries = json.load(file)

    if not isinstance(entries, list):
        raise ValueError("Logul trebuie sa contina o lista JSON.")

    required = {"ts", "sensors", "lines", "state", "pwr", "yaw", "ax", "impact"}
    valid_entries = [entry for entry in entries if required.issubset(entry)]
    if not valid_entries:
        raise ValueError("Logul nu contine mesaje de telemetrie valide.")
    return valid_entries


def summarize(entries):
    power = [entry["pwr"] for entry in entries]
    impacts = [entry for entry in entries if "!!HIT!!" in entry["impact"]]
    line_events = [entry for entry in entries if entry["lines"] != "00"]
    sensor_events = [entry for entry in entries if entry["sensors"] != "00000"]

    print(f"Mesaje: {len(entries)}")
    print(f"Durata: {entries[-1]['ts'] - entries[0]['ts']:.2f} s")
    print(f"Putere medie: {sum(power) / len(power):.1f}%")
    print(f"Putere maxima: {max(power)}%")
    print(f"Detectari adversar: {len(sensor_events)}")
    print(f"Evenimente linie: {len(line_events)}")
    print(f"Impacturi: {len(impacts)}")


def plot(entries):
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError(
            "Pentru grafice instaleaza matplotlib: python -m pip install matplotlib"
        ) from error

    ts = [entry["ts"] for entry in entries]
    power = [entry["pwr"] for entry in entries]
    yaw = [entry["yaw"] for entry in entries]
    accel = [entry["ax"] for entry in entries]
    sensor_names = ["Center", "Right 90", "Right", "Left 90", "Left"]
    sensors = [
        [1 if entry["sensors"][index] == "1" else 0 for entry in entries]
        for index in range(5)
    ]
    line = [1 if entry["lines"] != "00" else 0 for entry in entries]

    figure, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    axes[0].plot(ts, power, label="Power (%)", color="tab:blue")
    axes[0].set_ylabel("Power (%)")
    axes[0].grid(alpha=0.3)
    axes[0].legend()

    colors = ["tab:brown", "tab:orange", "tab:red", "tab:purple", "tab:green"]
    for values, name, color in zip(sensors, sensor_names, colors):
        axes[1].step(ts, values, where="post", label=name, color=color)
    axes[1].set_ylabel("Sensors")
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].grid(alpha=0.3)
    axes[1].legend()

    axes[2].plot(ts, yaw, label="Yaw (degrees)", color="tab:blue")
    axes[2].set_ylabel("Yaw")
    axes[2].grid(alpha=0.3)
    axes[2].legend()

    axes[3].plot(ts, accel, label="Accel X (G)", color="tab:purple")
    axes[3].fill_between(
        ts, 0, [-value for value in line], alpha=0.2,
        color="tab:orange", label="Line sensor"
    )
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("Accel")
    axes[3].grid(alpha=0.3)
    axes[3].legend()

    axes[4].step(ts, line, where="post", label="Line sensor", color="tab:orange")
    axes[4].set_xlabel("Time (s)")
    axes[4].set_ylabel("Line")
    axes[4].set_ylim(-0.1, 1.1)
    axes[4].grid(alpha=0.3)
    axes[4].legend()

    figure.suptitle("MiniSumo telemetry")
    figure.tight_layout()
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Utilizare: python analyze_telemetry.py saved_logs\\log_....json [--plot]")
        return 2

    try:
        entries = load_telemetry(Path(sys.argv[1]))
        summarize(entries)
        if "--plot" in sys.argv[2:]:
            plot(entries)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Eroare: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
