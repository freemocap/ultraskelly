"""Record a USB microphone with arecord, placing its samples on time.monotonic()'s clock."""
import math
import os
import subprocess
import threading
import time
import wave
from pathlib import Path

RATE = 48_000


def find_microphone(name: str) -> int:
    """ALSA card number of the first sound card whose name contains `name`."""
    for line in Path("/proc/asound/cards").read_text().splitlines():
        number, _, rest = line.partition("[")
        if number.strip().isdigit() and name in rest:
            return int(number)
    raise ValueError(f"No sound card matching {name!r}; check `arecord -l`")


class Microphone:
    """Records one ALSA card to a mono 16-bit WAV file until stopped.

    Each chunk's arrival time is noted, so `stop()` can say when the first sample was
    captured and the audio lines up with camera frames and motor commands.
    """

    def __init__(self, card: int, path: Path, *, rate: int = RATE, gain: int | None = None) -> None:
        if gain is not None:
            subprocess.run(["amixer", "-q", "-c", str(card), "sset", "Mic", str(gain)], check=False)
        self.rate = rate
        self.wav = wave.open(str(path), "wb")
        self.wav.setnchannels(1)
        self.wav.setsampwidth(2)
        self.wav.setframerate(rate)
        self.process = subprocess.Popen(
            ["arecord", "-q", "-D", f"plughw:{card},0", "-f", "S16_LE", "-r", str(rate), "-c", "1",
             "-t", "raw", "--period-time=10000", "--buffer-time=200000"],
            stdout=subprocess.PIPE,
        )
        self.samples = 0
        self.first_sample_at = math.inf
        self.thread = threading.Thread(target=self._read, name="microphone", daemon=True)
        self.thread.start()

    def _read(self) -> None:
        leftover = b""
        while chunk := os.read(self.process.stdout.fileno(), 65536):
            arrived = time.monotonic()
            data = leftover + chunk
            whole = len(data) // 2 * 2
            data, leftover = data[:whole], data[whole:]
            self.wav.writeframes(data)
            self.samples += whole // 2
            # A chunk arrives after its last sample was captured, so the earliest arrival,
            # less the samples before it, best estimates when the first sample was captured.
            self.first_sample_at = min(self.first_sample_at, arrived - self.samples / self.rate)

    def stop(self) -> float | None:
        """Stop recording and close the file; return the first sample's monotonic time, if any arrived."""
        self.process.terminate()
        self.thread.join(timeout=5)
        self.wav.close()
        return None if math.isinf(self.first_sample_at) else self.first_sample_at
