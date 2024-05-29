import argparse
import json
import numpy


def lpf_axis(axis, ws):
    v = numpy.ones(ws) / ws
    return numpy.convolve(axis, v, mode="valid")


def lpf_axes(axes, ws):
    return numpy.array([lpf_axis(axis, ws) for axis in axes])


def create_laps(data):
    return data["laps"]


def create_min_time(data):
    if len(data["laps"]) == 0:
        return None
    return min(data["laps"])


def create_max_jerk(data, dt, ws):
    if len(data["laps"]) == 0:
        return None
    original_v = numpy.array([[v["x"], v["y"], v["z"]] for v in data["velocities"]]).T
    filtered_v = lpf_axes(original_v, ws)
    original_a = numpy.diff(filtered_v) / dt
    filtered_a = lpf_axes(original_a, ws)
    original_j = numpy.diff(filtered_a) / dt
    filtered_j = lpf_axes(original_j, ws)
    return max(numpy.linalg.norm(j, ord=2) for j in filtered_j.T)


parser = argparse.ArgumentParser()
parser.add_argument("hz", type=float)
parser.add_argument("ws", type=int)
parser.add_argument("--input", default="result-details.json")
parser.add_argument("--output", default="result-summary.json")

args = parser.parse_args()
dt = 1.0 / args.hz
ws = args.ws

with open(args.input) as fp:
    details = json.load(fp)

summary = {
    "laps": create_laps(details),
    "min_time": create_min_time(details),
    "max_jerk": create_max_jerk(details, dt, ws),
}

with open(args.output, "w") as fp:
    json.dump(summary, fp, indent=4)
    fp.write("\n")
