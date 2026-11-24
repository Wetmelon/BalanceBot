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
        "-fconstexpr-ops-limit=1073741824",  # the designs are solved at compile time: the OFLQG needs 2^26, the OFMPC 2^27, past the default 2^25
    ]
)
