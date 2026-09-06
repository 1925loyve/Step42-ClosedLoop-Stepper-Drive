# Step42 Upper Computer

`Step42上位机_30kHz.exe` is the current packaged Windows control and tuning
application for the ST-T42 driver. It can run on a Windows computer without a
separate Python installation. Its pulse-speed controls use the same dynamic
limit as the firmware: `min(5000 * microsteps, 30000)` pulse/s.

Before connecting, select the serial port connected to the driver and use the
baud rate required by the firmware documentation.

## AI connection

Open the `Settings` page and choose either OpenAI Responses API or an
OpenAI-compatible Chat Completions endpoint. Enter the endpoint, a model that
supports function calling, and the API key, then use the `AI Assistant` page.
The endpoint and model can be saved locally; the API key remains in process
memory and is not written to disk.

The AI is limited to one connected motor and validated RAM operations. It has
no tool for Flash writes, firmware updates, factory commands, or disabling the
temperature and stall protections. Saving an instruction program to Flash
always requires the dedicated manual confirmation button.
