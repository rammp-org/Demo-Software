#!/usr/bin/env python3
# Copyright 2025 LUCI Mobility, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Stream collision and dropoff line fit data from a LUCI sensor server.

Usage:
    python3 line_fits_stream.py <ip>
    python3 line_fits_stream.py 192.168.1.100

Requires:
    pip install grpcio
"""

import sys
import struct
import threading
import argparse
import grpc


# ---------------------------------------------------------------------------
# Minimal protobuf decoder for:
#   LineFit  { float max_height_m = 1; float distance_m = 2; float angle_rad = 3; }
#   LineFits { repeated LineFit lines = 1; }
# ---------------------------------------------------------------------------


def _decode_varint(data, pos):
    result, shift = 0, 0
    while True:
        b = data[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, pos
        shift += 7


def _decode_line_fit(data):
    fields, i = {}, 0
    while i < len(data):
        tag = data[i]
        i += 1
        wire_type = tag & 0x07
        field_num = tag >> 3
        if wire_type == 5:  # 32-bit fixed (float)
            fields[field_num] = struct.unpack_from("<f", data, i)[0]
            i += 4
        else:
            break
    return fields


def _decode_line_fits(data):
    fits, i = [], 0
    while i < len(data):
        tag = data[i]
        i += 1
        wire_type = tag & 0x07
        field_num = tag >> 3
        if wire_type == 2 and field_num == 1:  # length-delimited LineFit
            length, i = _decode_varint(data, i)
            fits.append(_decode_line_fit(data[i : i + length]))
            i += length
        else:
            break
    return fits


# ---------------------------------------------------------------------------
# Streaming
# ---------------------------------------------------------------------------


def _stream(channel, rpc_path, label):
    method = channel.unary_stream(
        rpc_path,
        request_serializer=lambda _: b"",  # Empty request serialises to nothing
        response_deserializer=_decode_line_fits,
    )
    try:
        for fits in method(None):
            for fit in fits:
                max_h = fit.get(1, float("nan"))
                max_h_in = max_h * 39.3701
                distance = fit.get(2, float("nan"))
                distance_in = distance * 39.3701
                angle = fit.get(3, float("nan"))
                print(
                    f"[{label}]  max_height={max_h_in:+.3f}in  "
                    f"distance={distance_in:.3f}in  "
                    f"angle={angle:.4f}rad"
                )
    except grpc.RpcError as e:
        print(f"[{label}] stream ended: {e.code()}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Stream LUCI line fit data over gRPC")
    parser.add_argument("ip", help="Sensor server IP  e.g. 192.168.1.100")
    args = parser.parse_args()

    channel = grpc.insecure_channel(f"{args.ip}:50051")

    threads = [
        threading.Thread(
            target=_stream,
            daemon=True,
            args=(channel, "/sensors.Sensors/CollisionLineFitsStream", "COLLISION"),
        ),
        threading.Thread(
            target=_stream,
            daemon=True,
            args=(channel, "/sensors.Sensors/DropoffLineFitsStream", "DROPOFF"),
        ),
    ]

    for t in threads:
        t.start()

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        print("\nStopped.")
        channel.close()


if __name__ == "__main__":
    main()
