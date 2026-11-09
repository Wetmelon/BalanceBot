# PlatformIO post-script for the C33: reach the DFU bootloader with a 1200 bps touch before dfu-util.
# dfu-util's own way in, a DFU detach through the running sketch, times out (core 1.6.0, Windows).
Import("env")  # noqa: F821


def touch_to_bootloader(source, target, env):
    try:
        env.AutodetectUploadPort()
        env.TouchSerialPort("$UPLOAD_PORT", 1200)
    except Exception:  # no serial port: already in the bootloader (reset double-tapped)
        pass


env.AddPreAction("upload", touch_to_bootloader)  # noqa: F821
env.Prepend(UPLOADERFLAGS=["-w"])  # noqa: F821  # wait for the bootloader to enumerate
