Import("env")

if "compiledb" in COMMAND_LINE_TARGETS:
    toolchain_includes = env.DumpIntegrationIncludes().get("toolchain", ())
    env.Append(
        CPPPATH=[
            path
            for path in toolchain_includes
            if "toolchain-xtensa-esp" in path
            and "/picolibc/" not in path
        ]
    )
