import json

with open("sample.json") as sample:
    json.loads(sample.read())
