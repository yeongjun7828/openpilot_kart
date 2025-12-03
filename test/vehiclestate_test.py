#!/usr/bin/env python3
import time
import os
import cereal.messaging as messaging

RATE_HZ = 40
PERIOD = 1.0 / RATE_HZ
pm = messaging.PubMaster(['vehicleState'])
PARAM_FILE = "vehicle_params.txt"

def read_params():
    """params 파일에서 VEGO, STEER, CRUISE_SPEED 읽기"""
    params = {"VEGO": 15.0, "STEER": 0.0, "CRUISE_SPEED": 25.0}
    try:
        with open(PARAM_FILE, "r") as f:
            for line in f:
                if "=" in line:
                    key, val = line.strip().split("=", 1)
                    if key in params:
                        params[key] = float(val)
    except FileNotFoundError:
        pass
    return params


def make_full_vehicleState(params):
    msg = messaging.new_message('vehicleState')
    vs = msg.vehicleState

    vEgo = params["VEGO"]
    steer_angle = params["STEER"]
    cruise_speed = params["CRUISE_SPEED"]

    # CAN health
    vs.canValid = True
    vs.canTimeout = False

    # Car speed and motion
    vs.vEgo = vEgo
    vs.vEgoRaw = vEgo
    vs.aEgo = 0.0
    vs.vEgoCluster = vEgo
    vs.yawRate = 0.0
    vs.standstill = vEgo < 0.1

    # Wheel speeds
    vs.wheelSpeeds.fl = vEgo
    vs.wheelSpeeds.fr = vEgo
    vs.wheelSpeeds.rl = vEgo
    vs.wheelSpeeds.rr = vEgo

    # Gas / brake
    vs.gas = 0.2
    vs.gasPressed = False
    vs.engineRpm = 1500.0
    vs.brake = 0.0
    vs.brakePressed = False
    vs.regenBraking = False
    vs.parkingBrake = False
    vs.brakeHoldActive = False

    # Steering
    vs.steeringAngleDeg = steer_angle
    vs.steeringAngleOffsetDeg = 0.0
    vs.steeringRateDeg = 0.0
    vs.steeringTorque = 0.0
    vs.steeringTorqueEps = 0.0
    vs.steeringPressed = False
    vs.steerFaultTemporary = False
    vs.steerFaultPermanent = False

    # Stock system states
    vs.stockAeb = False
    vs.stockFcw = False
    vs.espDisabled = False
    vs.accFaulted = False
    vs.carFaultedNonCritical = False

    # Cruise state
    cs = vs.cruiseState
    cs.enabled = True
    cs.available = True
    cs.speed = cruise_speed
    cs.speedCluster = cruise_speed
    cs.speedOffset = 0.0
    cs.standstill = vs.standstill
    cs.nonAdaptive = False

    # Gear / buttons / etc
    vs.gearShifter = 2  # drive
    vs.buttonEvents = []
    vs.leftBlinker = False
    vs.rightBlinker = False
    vs.genericToggle = False

    # Lock info
    vs.doorOpen = False
    vs.seatbeltUnlatched = False

    # Clutch (manual only)
    vs.clutchPressed = False

    # Blindspot
    vs.leftBlindspot = False
    vs.rightBlindspot = False

    # Fuel / charging
    vs.fuelGauge = 0.75
    vs.charging = False

    # Deprecated (남겨두되 채워줌)
    vs.errorsDEPRECATED = []
    vs.brakeLightsDEPRECATED = False
    vs.steeringRateLimitedDEPRECATED = False
    vs.canMonoTimesDEPRECATED = []

    # events 리스트
    vs.events = []

    return msg


def main():
    next_t = time.monotonic()
    print(f"[INFO] vehicleState publisher running at {RATE_HZ} Hz (reading {PARAM_FILE})")
    while True:
        params = read_params()
        msg = make_full_vehicleState(params)
        pm.send('vehicleState', msg)

        next_t += PERIOD
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.monotonic()


if __name__ == "__main__":
    main()

