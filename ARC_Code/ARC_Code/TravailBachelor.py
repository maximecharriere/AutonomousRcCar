



while True:
    reply = input("Change speed(s) or angle(a): ")
    if reply == 's':
        reply = input("Speed [0 - 100]: ")
        SpeedCtrl.Speed(float(reply))
    elif reply == 'a':
        reply = input("Angle [0 - 100]: ")
        SteeringCtrl.Angle(float(reply))
    else:
        print("Invalid")


def main():
    for device in devices:
        print(device)


if __name__ == "__main__":
    main()