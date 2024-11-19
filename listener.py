import asyncio
import websockets
import pyaudio
import base64
import json


audio = pyaudio.PyAudio()
output_stream = audio.open(
    format=pyaudio.paInt16, channels=1, rate=24000, output=True, frames_per_buffer=1024
)


async def echo(websocket):
    print(f"Connected to {websocket.remote_address}")
    async for message in websocket:
        # with open("sample.json", "wt") as sample:
        # sample.write(message)
        data = json.loads(message)
        # # Decode the base64 message
        decoded_data = base64.b64decode(data["audio"])
        # print(f"Decoded data length: {len(decoded_data)}")
        # # Play the audio
        output_stream.write(decoded_data)


async def main():
    print("Listening on ws://0.0.0.0:48000")
    async with websockets.serve(echo, "0.0.0.0", 48000) as server:
        await server.serve_forever()


asyncio.run(main())
