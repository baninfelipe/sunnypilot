"""Extract persistent EPS 48A2 records from full rlogs as JSON lines.

python -m openpilot.tools.debug.eps48a2 /path/to/rlog.zst
"""
import argparse
import bz2
import json
from pathlib import Path

import zstandard

from openpilot.cereal import log


def records(events):
  previous = None
  for event in events:
    if event.which() != "carStateSP":
      continue
    sample = event.carStateSP.eps48a2
    if not sample.enabled and sample.status == "disabled":
      continue
    row = sample.to_dict()
    row["response"] = bytes(sample.response).hex()
    key = json.dumps(row, sort_keys=True)
    if key == previous:
      continue
    previous = key
    row["logMonoTime"] = event.logMonoTime
    # Zero-filled invalid fields are not measurements.
    if not sample.sampleValid:
      for field in ("request", "requestFlags", "directional", "returnSdiv32", "q", "internalStatus"):
        row[field] = None
    yield row


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("rlogs", type=Path, nargs="+")
  args = parser.parse_args()
  for path in args.rlogs:
    data = path.read_bytes()
    if path.suffix == ".zst":
      data = zstandard.ZstdDecompressor().decompress(data, max_output_size=512 * 1024 * 1024)
    elif path.suffix == ".bz2":
      data = bz2.decompress(data)
    for row in records(log.Event.read_multiple_bytes(data)):
      print(json.dumps(row, sort_keys=True))


if __name__ == "__main__":
  main()
