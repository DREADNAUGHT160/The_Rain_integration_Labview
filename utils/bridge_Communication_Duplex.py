import socket
import threading

# Configuration
PORTS = {
    "rain": 5001,
    "fog": 5002
}

# IP addresses
COMPUTER_A_IP = '10.0.0.10'        # Ethernet interface to Computer A (LabVIEW)
COMPUTER_C_IP = '192.168.1.20'     # Wi-Fi interface to Computer C

BUFFER_SIZE = 4096

def forwarder(listen_ip, listen_port, target_ip, target_port, direction_tag):
    """
    Forwards incoming data from (listen_ip:listen_port) to (target_ip:target_port).
    Prints all transfers for logging/debugging.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((listen_ip, listen_port))
        server.listen(5)
        print(f"[{direction_tag}] Listening on {listen_ip}:{listen_port}...")

        while True:
            conn, addr = server.accept()
            threading.Thread(
                target=handle_forward,
                args=(conn, addr, target_ip, target_port, direction_tag),
                daemon=True
            ).start()

def handle_forward(conn, addr, target_ip, target_port, direction_tag):
    """
    Handles a single connection: receives data, forwards it to the target IP/port.
    """
    with conn:
        try:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                print(f"[{direction_tag}] No data received from {addr}.")
                return

            print(f"[{direction_tag}] Received from {addr}: {data.decode(errors='replace').strip()}")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as out_sock:
                out_sock.connect((target_ip, target_port))
                out_sock.sendall(data)
                print(f"[{direction_tag}] Forwarded to {target_ip}:{target_port}")

        except Exception as e:
            print(f"[{direction_tag}] Error: {e}")

def main():
    # Forward Computer C → Computer A (rain/fog commands)
    for name, port in PORTS.items():
        threading.Thread(
            target=forwarder,
            args=('0.0.0.0', port, COMPUTER_A_IP, port, f"{name.upper()} C→A"),
            daemon=True
        ).start()

    # Forward Computer A → Computer C (feedback for rain/fog)
    # Listen on a separate port range if desired (here: same port as forward)
    for name, port in PORTS.items():
        threading.Thread(
            target=forwarder,
            args=('0.0.0.0', port + 100, COMPUTER_C_IP, port + 100, f"{name.upper()} A→C"),
            daemon=True
        ).start()

    print("\nBidirectional bridge is running.")
    print(" - For C→A, Computer C sends to Computer B on port 5001/5002; B forwards to A on 5001/5002.")
    print(" - For A→C, Computer A sends to Computer B on port 5101/5102; B forwards to C on 5101/5102.")
    print("You can add more channels or change port mapping as needed.\n")
    input("Press Enter to quit...\n")

if __name__ == "__main__":
    main()
