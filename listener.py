import socket
import pyaudio

# TCP server information
tcp_port = 48000

# Create a TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
sock.bind(("", tcp_port))

# Listen for incoming connections
sock.listen(1)

print("Listening for audio on port {}".format(tcp_port))

# Initialize PyAudio
p = pyaudio.PyAudio()

# Open a stream for audio playback
stream = p.open(
    format=pyaudio.paInt32,  # Assuming INT32 LE samples
    channels=1,  # Mono audio
    rate=48000,  # Sample rate
    output=True,
)

try:
    while True:
        # Accept a connection
        print("Waiting for connection...")
        conn, addr = sock.accept()
        print("Connection from", addr)

        try:
            # Receive and play audio samples
            while True:
                data = conn.recv(4096)  # Adjust buffer size as needed
                if not data:
                    break
                stream.write(data)
        finally:
            conn.close()
            print("Connection closed, waiting for new connection...")
finally:
    # Clean up
    stream.stop_stream()
    stream.close()
    p.terminate()
    sock.close()
