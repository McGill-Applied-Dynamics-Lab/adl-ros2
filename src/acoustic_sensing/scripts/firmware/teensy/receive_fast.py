import serial
import matplotlib.pyplot as plt




while True:

    ser = serial.Serial('/dev/ttyACM0', 3000000)  # adjust port!

    data0 = []
    data1 = []
    data2 = []
    data3 = []


    # Wait for START (Rangefinder 0)
    while True:
        line = ser.readline().decode().strip()
        if line == "S0":
            break

    # Read data
    while True:
        line = ser.readline().decode().strip()
        if line == "T":
            break
        try:
            data0.append(int(line))
        except:
            pass

    # Wait for START (Rangefinder 1)
    while True:
        line = ser.readline().decode().strip()
        if line == "S1":
            break

    # Read data
    while True:
        line = ser.readline().decode().strip()
        if line == "T":
            break
        try:
            data1.append(int(line))
        except:
            pass
    
    # Wait for START (Rangefinder 2)
    while True:
        line = ser.readline().decode().strip()
        if line == "S2":
            break

    # Read data
    while True:
        line = ser.readline().decode().strip()
        if line == "T":
            break
        try:
            data2.append(int(line))
        except:
            pass

    # Wait for START (Rangefinder 1)
    while True:
        line = ser.readline().decode().strip()
        if line == "S3":
            break

    # Read data
    while True:
        line = ser.readline().decode().strip()
        if line == "T":
            break
        try:
            data3.append(int(line))
        except:
            pass

    ser.close()

    # Plot
    fig, ax = plt.subplots(nrows=1, ncols=4, figsize = (16, 5))
    
    ax[0].plot(data0)
    ax[0].set_title("Rangefinder 0")
    ax[1].plot(data1)
    ax[1].set_title("Rangefinder 1")
    ax[2].plot(data2)
    ax[2].set_title("Rangefinder 2")
    ax[3].plot(data3)
    ax[3].set_title("Rangefinder 3")
    plt.title("Captured Signal (10k samples)")
    plt.show(block=False)

    input()
    #close plot
    plt.close()