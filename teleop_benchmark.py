import zmq
import time
import json
import csv
import os

# If your package is installed, you can import the real policy here:
# from so100_teleop.hybrid_node import RobustPolicy 

# MEASUREMENT SETTINGS
ZMQ_PORT = 5555
DURATION_SECONDS = 30  # How long the measurement should run (seconds)

benchmark_dir = "benchmark_results/similarinsametime"
file_count = len([f for f in os.listdir(benchmark_dir) if os.path.isfile(os.path.join(benchmark_dir, f))]) if os.path.exists(benchmark_dir) else 0
OUTPUT_FILE = f"benchmark_results/similarinsametime/benchmark_results{file_count}.csv"

def run_benchmark():
    print("--- SO100 TELEOP BENCHMARK START ---")
    print(f"Connecting to ZMQ network (Port: {ZMQ_PORT})...")
    
    # ZMQ Setup (Exactly as in the Follower/Hybrid node)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.CONFLATE, 1)  # Enable network optimization
    socket.connect(f"tcp://127.0.0.1:{ZMQ_PORT}")
    
    # Data collection lists
    results = []
    
    # Here you would initialize the real policy
    # policy = RobustPolicy() 
    # dummy_follower_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    print(f"Waiting for first message...")
    
    # Wait for the first message so we don't measure cold start
    socket.recv_string()
    
    print(f"Data stream detected! Starting measurement for {DURATION_SECONDS} seconds...")
    print("Control the robot continuously!\n")
    
    start_time = time.perf_counter()
    last_msg_time = start_time
    msg_count = 0
    
    try:
        while time.perf_counter() - start_time < DURATION_SECONDS:
            # 1. Measure network wait time (Jitter)
            message = socket.recv_string()
            recv_time = time.perf_counter()
            
            # Elapsed time between two messages (Delta T) in milliseconds
            delta_t_ms = (recv_time - last_msg_time) * 1000.0
            last_msg_time = recv_time
            
            # Parse data
            data = json.loads(message)
            
            # 2. Measure computation latency (Processing Latency)
            process_start = time.perf_counter()
            
            # --- REAL MATH WOULD RUN HERE ---
            # policy.forward(leader_joints, leader_gripper, dummy_follower_state)
            
            # Simulate computation time if policy is not imported (approx. 2-5 ms for typical IK)
            # time.sleep(0.003) 
            # --------------------------------
            
            process_end = time.perf_counter()
            processing_latency_ms = (process_end - process_start) * 1000.0
            
            # Save results (Timestamp, Delta_T, Processing_Time)
            results.append({
                "id": msg_count,
                "timestamp": recv_time - start_time,
                "network_delta_ms": delta_t_ms,
                "processing_ms": processing_latency_ms
            })
            
            msg_count += 1
            
            # Terminal display (not every cycle to avoid slowdown)
            if msg_count % 30 == 0:
                print(f"Measurement in progress... [{msg_count} packets recorded] | Delta: {delta_t_ms:.2f} ms")
                
    except KeyboardInterrupt:
        print("\n Measurement stopped manually.")
        
    print("\nMeasurement completed!")
    
    # 3. CSV Export
    print(f"Saving data to {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, 'w', newline='') as csvfile:
        fieldnames = ['id', 'timestamp', 'network_delta_ms', 'processing_ms']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow(row)
            
    # Print basic statistics
    if len(results) > 0:
        avg_delta = sum(r['network_delta_ms'] for r in results) / len(results)
        avg_fps = 1000.0 / avg_delta if avg_delta > 0 else 0
        avg_processing = sum(r['processing_ms'] for r in results) / len(results)
        
        print("\nQUICK STATISTICS:")
        print(f"Total recorded packets: {len(results)}")
        print(f"Average refresh frequency: {avg_fps:.1f} Hz")
        print(f"Average network interval: {avg_delta:.2f} ms")
        print(f"Average computation time (IK/FK): {avg_processing:.2f} ms")
    
    socket.close()
    context.term()

if __name__ == "__main__":
    run_benchmark()
