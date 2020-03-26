


def Map(x, in_min, in_max, out_min, out_max):
    print(f"X: {x}  Min: {in_min}   Max: {in_max}   in_max - in_min: {in_max - in_min}");
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

