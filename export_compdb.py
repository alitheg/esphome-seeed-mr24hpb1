Import("env")

env.AddPostAction(
    "buildprog",
    env.VerboseAction("pio run -t compiledb", "Generating compile_commands.json")
)