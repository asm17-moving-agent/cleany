"""Make a training bag with every eighth scan withheld for an offline fit check."""

import argparse
from pathlib import Path

import rosbag2_py as bag


def withhold(source: Path, output: Path) -> None:
    if output.exists():
        raise FileExistsError(output)
    converter = bag.ConverterOptions("", "")
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(source), storage_id="sqlite3"), converter)
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(output), storage_id="sqlite3"), converter)
    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)
    scans = withheld = 0
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        if topic == "/scan":
            scans += 1
            if (scans - 1) % 8 == 0:
                withheld += 1
                continue
        writer.write(topic, payload, stamp)
    del writer
    print(f"Held out {withheld} of {scans} scans; training scans: {scans-withheld}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    withhold(args.source, args.output)
