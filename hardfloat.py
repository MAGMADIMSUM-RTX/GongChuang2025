Import("env")

env.Append(
    LINKFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"],
    CCFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"],
    ASFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"],
    # LINKFLAGS=["--specs=nosys.specs", "--specs=nano.specs"]
)

env.Append(LINKFLAGS=["--specs=nosys.specs", "--specs=nano.specs"])
