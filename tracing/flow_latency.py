import sys
import pandas as pd
from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.message_flow import MessageFlow

# === EDIT THESE THREE TOPICS ===
SOURCE_TOPICS = [
    '/camera/front/image',            # camera
    '/lidar/concatenated/pointcloud'  # lidar
]
TARGET_TOPIC = '/perception/object_recognition/tracking/objects'  # tracker output
# =================================

def analyze(trace_dir: str):
    events = load_file(trace_dir)
    for src in SOURCE_TOPICS:
        flow = MessageFlow(
            source_topic=src,
            target_topic=TARGET_TOPIC,
            follow_intra_process=True,
            follow_inter_process=True,
        )
        flow.process(events)
        df = flow.data.to_dataframe()
        if df.empty:
            print(f'[WARN] No matched paths from {src} → {TARGET_TOPIC}')
            continue

        # latency is in nanoseconds
        df['latency_ms'] = df['latency'] / 1e6
        out = f'latency_{src.strip("/").replace("/", "_")}__to__{TARGET_TOPIC.strip("/").replace("/", "_")}.csv'
        df.to_csv(out, index=False)

        print(f'\n{src} → {TARGET_TOPIC}  (n={len(df)})')
        print(df['latency_ms'].describe(percentiles=[0.5, 0.9, 0.95, 0.99]).to_string())

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python3 analyze_latency.py /path/to/trace_dir')
        sys.exit(1)
    analyze(sys.argv[1])
