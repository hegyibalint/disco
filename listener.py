import asyncio
import websockets
import pyaudio
import base64
import json

start_msg = """{
    "event_id": "event_1234",
    "type": "session.created",
    "session": {
        "id": "sess_001",
        "object": "realtime.session",
        "model": "gpt-4o-realtime-preview-2024-10-01",
        "modalities": ["text", "audio"],
        "instructions": "",
        "voice": "alloy",
        "input_audio_format": "pcm16",
        "output_audio_format": "pcm16",
        "input_audio_transcription": null,
        "turn_detection": {
            "type": "server_vad",
            "threshold": 0.5,
            "prefix_padding_ms": 300,
            "silence_duration_ms": 200
        },
        "tools": [],
        "tool_choice": "auto",
        "temperature": 0.8,
        "max_response_output_tokens": null
    }
}"""


audio = pyaudio.PyAudio()
output_stream = audio.open(
    format=pyaudio.paInt16, channels=1, rate=24000, output=True, frames_per_buffer=1024
)


async def echo(websocket):
    print(f"Connected to {websocket.remote_address}")
    await websocket.send(start_msg)
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
