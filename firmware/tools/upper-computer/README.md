# Step42 Upper Computer

`Step42上位机.exe` is the packaged Windows control and tuning application for
the ST-T42 driver. It can run on a Windows computer without a separate Python
installation. Its pulse-speed controls use the same dynamic limit as the
firmware: `min(5000 * microsteps, 30000)` pulse/s.

The `误差PID` page configures the built-in single- or dual-loop error PID. The
filter, prediction, and dynamic-gain options are disabled unless their own
checkboxes are enabled. The page has separate RAM and Flash reads, a one-step
RAM apply action, and a rate-limited page Flash save. The `连续位置` page is a
separate external multi-turn position controller with its own RAM/Flash
configuration; it uses the global zero point and can hold, return to zero, or
disable on target loss.

Continuous telemetry is off by default. When it is off, the displayed values
remain at their last sample until `Read once` is used; the application does
not poll the drive in the background. Enable the stream only when live values
or waveforms are needed.

Before connecting, select the serial port connected to the driver and use the
baud rate required by the firmware documentation.

## AI connection

Open the `Settings` page and choose either OpenAI Responses API or an
OpenAI-compatible Chat Completions endpoint. Enter the endpoint, a model that
supports function calling, and the API key, then save it as a named interface.
The API key is encrypted with Windows DPAPI for the current Windows account.
After saving, its fields are locked and the key is never displayed again; the
saved interface can only be selected, renamed, or deleted.

The AI is limited to one connected motor and validated RAM operations. It has
no tool for Flash writes, firmware updates, factory commands, or disabling the
temperature and stall protections. Saving an instruction program to Flash
always requires the dedicated manual confirmation button.

DeepSeek endpoints are detected from their URL so thinking-mode requests do
not receive the unsupported `tool_choice` field. The packaged application
uses protocol v20, matching the firmware and the SDK in `firmware/SDK`.
