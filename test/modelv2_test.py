#!/usr/bin/env python3
import time
import numpy as np
import cereal.messaging as messaging

RATE_HZ = 20
PERIOD = 1.0 / RATE_HZ
TRAJECTORY_SIZE = 33
pm = messaging.PubMaster(['modelV2'])


def make_safe_modelV2(frame_id: int):
    msg = messaging.new_message('modelV2')
    m = msg.modelV2

    now = int(time.monotonic() * 1e9)
    msg.logMonoTime = now
    m.timestampEof = now
    m.frameId = frame_id

    # === 필수 init() ===
    m.init('meta')
    m.meta.init('desireState', 4)
    m.init('position')
    m.init('velocity')
    m.init('orientation')
    m.init('orientationRate')
    m.init('laneLineProbs', 4)
    # ===================

    # meta
    m.meta.hardBrakePredicted = False
    for i, val in enumerate([0.0, 1.0, 0.0, 0.0]):
        m.meta.desireState[i] = val

    # lane line probabilities
    for i, val in enumerate([0.8, 0.7, 0.6, 0.5]):
        m.laneLineProbs[i] = val

    # trajectory
    t = np.arange(TRAJECTORY_SIZE, dtype=np.float32) * 0.1
    x = np.arange(TRAJECTORY_SIZE, dtype=np.float32)
    y = 0.5 * np.sin(np.arange(TRAJECTORY_SIZE, dtype=np.float32) * 0.1)
    z = np.zeros(TRAJECTORY_SIZE, dtype=np.float32)

    vx = np.ones(TRAJECTORY_SIZE, dtype=np.float32)
    vy = np.zeros(TRAJECTORY_SIZE, dtype=np.float32)
    vz = np.zeros(TRAJECTORY_SIZE, dtype=np.float32)

    yaw = np.arange(TRAJECTORY_SIZE, dtype=np.float32) * 0.01
    yaw_rate = np.full(TRAJECTORY_SIZE, 0.01, dtype=np.float32)

    # assign
    m.position.x = x.tolist()
    m.position.y = y.tolist()
    m.position.z = z.tolist()
    m.position.t = t.tolist()

    m.velocity.x = vx.tolist()
    m.velocity.y = vy.tolist()
    m.velocity.z = vz.tolist()

    m.orientation.x = yaw.tolist()
    m.orientation.y = [0.0] * TRAJECTORY_SIZE
    m.orientation.z = yaw.tolist()

    m.orientationRate.x = [0.0] * TRAJECTORY_SIZE
    m.orientationRate.y = [0.0] * TRAJECTORY_SIZE
    m.orientationRate.z = yaw_rate.tolist()

    return msg


def main():
    frame = 0
    next_t = time.monotonic()
    print(f"[INFO] modelV2 safe publisher running at {RATE_HZ} Hz")
    while True:
        msg = make_safe_modelV2(frame)
        pm.send('modelV2', msg)
        frame += 1
        next_t += PERIOD
        time.sleep(max(0, next_t - time.monotonic()))


if __name__ == "__main__":
    main()

