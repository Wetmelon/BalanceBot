# PlatformIO pre-script: C++-only flags, kept off the framework's C files.
Import("env")  # noqa: F821

env.Append(  # noqa: F821
    CXXFLAGS=[
        "-std=gnu++20",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-threadsafe-statics",
        "-Wno-volatile",  # the Arduino cores still use volatile ++/--
        "-Wno-register",  # CMSIS
    ]
)
