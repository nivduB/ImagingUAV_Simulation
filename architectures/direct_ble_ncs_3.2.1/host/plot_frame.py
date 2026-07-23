# from pathlib import Path

# PIXELS_PER_FRAME = 342

# data = Path("epc901_dump.bin").read_bytes()

# print(f"File size: {len(data)} bytes")
# print(f"Complete frames: {len(data) // PIXELS_PER_FRAME}")
# print(f"Remaining bytes: {len(data) % PIXELS_PER_FRAME}")

# frames = [
#     data[i:i + PIXELS_PER_FRAME]
#     for i in range(0, len(data), PIXELS_PER_FRAME)
#     if len(data[i:i + PIXELS_PER_FRAME]) == PIXELS_PER_FRAME
# ]

# for index, frame in enumerate(frames):
#     print(f"Frame {index}:")
#     print(list(frame[:20]))  # first 20 pixel values

from pathlib import Path
from collections import Counter

data = Path("epc901_dump.bin").read_bytes()

print("Size:", len(data))
print("Minimum:", min(data) if data else None)
print("Maximum:", max(data) if data else None)
print("Nonzero bytes:", sum(value != 0 for value in data))
print("Most common:", Counter(data).most_common(10))