# analyze_latency_verbose.py
import sys, os, glob, traceback
import pandas as pd
from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.message_flow import MessageFlow

# EDIT THESE if needed
SOURCE_TOPICS = [
    '/camera/front/image',
    '/lidar/concatenated/pointcloud',
]
TARGET_TOPIC = '/perception/tracking/tracked_objects'

def resolve_trace_dir(arg):
    # support glob like ~/.ros/tracing/fusion_perception_trace-*
    candidates = sorted(glob.glob(os.path.expanduser(arg)))
    if not candidates:
        print(f"[ERROR] No trace dirs matched: {arg}")
        return None
    # pick the latest
    return candidates[-1]

def main(arg):
    trace_dir = resolve_trace_dir(arg)
    if not trace_dir:
        return 2
    print(f"[INFO] Using trace dir: {trace_dir}")

    try:
        events = load_file(trace_dir)
    except Exception as e:
        print("[ERROR] Failed to load trace. Do you have babeltrace2 + bt2 installed?")
        print("Tip: sudo apt install babeltrace2 python3-bt2  (or: pip install --user bt2)")
        traceback.print_exc()
        return 3

    any_rows = False
    for src in SOURCE_TOPICS:
        print(f"\n[INFO] Analyzing path: {src} → {TARGET_TOPIC}")
        try:
            flow = MessageFlow(
                source_topic=src,
                target_topic=TARGET_TOPIC,
                follow_intra_process=True,
                follow_inter_process=True,
            )
            flow.process(events)
            df = flow.data.to_dataframe()
        except Exception as e:
            print("[ERROR] MessageFlow failed. Common cause: trace missing 'ros2:*' events.")
            traceback.print_exc()
            continue

        if df.empty:
            print(f"[WARN] No matched messages from {src} to {TARGET_TOPIC}.")
            print("      Check topic names and that both ends actually published during tracing.")
            continue

        any_rows = True
        df['latency_ms'] = df['latency'] / 1e6
        out = f'latency_{src.strip("/").replace("/", "_")}__to__{TARGET_TOPIC.strip("/").replace("/", "_")}.csv'
        df.to_csv(out, index=False)
        print(f"[OK] Rows: {len(df)}  CSV: {os.path.abspath(out)}")
        print(df['latency_ms'].describe(percentiles=[0.5,0.9,0.95,0.99]).to_string())

    if not any_rows:
        print("\n[RESULT] No matches found for any source. Run these quick checks:")
        print("  1) ros2 topic echo /perception/tracking/tracked_objects -n1   (exists?)")
        print("  2) ros2 topic echo /camera/front/image -n1                    (exists?)")
        print("  3) ros2 topic echo /lidar/concatenated/pointcloud -n1         (exists?)")
        print("  4) Ensure tracing included userspace events: events_ust=['ros2:*']")
        print("  5) Re-run the launch, let the bag play for ~30s, then analyze again.")
    return 0

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_latency_verbose.py <trace_dir_or_glob>")
        sys.exit(1)
    sys.exit(main(sys.argv[1]))
