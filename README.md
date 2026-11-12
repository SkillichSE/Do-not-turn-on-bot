# NII-fight-bot

Combat robot firmware for ESP32, controlled with a DualShock 4 gamepad over Bluetooth.

> [!IMPORTANT]
> Download the project **only with this command**. The libraries (arduino-esp32 and btstack) are git submodules, and without `--recursive` they will not be downloaded and the project will not build.
>
> ```
> git clone --recursive --depth 1 --shallow-submodules https://github.com/SkillichSE/NII-fight-bot
> ```
>
> If you already cloned it with a plain `git clone`, run this inside the project folder:
> ```
> git submodule update --init --depth 1
> ```

## Build and upload

You need VS Code with the PlatformIO extension, or PlatformIO Core installed.

> [!IMPORTANT]
> Connect the ESP32 over USB before uploading, and when using a terminal, go to the project folder first (`cd NII-fight-bot`). The first build takes a while: PlatformIO downloads the toolchain and dependencies on its own.

Build:
```
pio run -e esp32dev
```

Upload to the board:
```
pio run -e esp32dev -t upload
```

Open the serial monitor (115200 baud):
```
pio device monitor
```

Upload and open the monitor right away:
```
pio run -e esp32dev -t upload -t monitor
```

If the board does not flash, hold the **BOOT** button on the board when the upload starts.

## Connecting the gamepad

Hold **SHARE + PS** on the gamepad until it starts blinking quickly. The robot connects automatically, the gamepad vibrates briefly, and the buzzer and LED trigger.

## Controls

| Action | What it does |
|---|---|
| Left stick | Drive (forward/back and turning) |
| X button | Flipper: one lift, then lowers right away |

If the connection to the gamepad is lost, the motors stop.
