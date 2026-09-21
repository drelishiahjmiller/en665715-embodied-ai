"""Standalone MCP server exposing offline robot datasheet lookup tools.

Normally this is launched as a subprocess by rosa_agent_mcp.py over stdio.
"""
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("robot-datasheets")

BNO055_NOTES = {
    "calibration": "BNO055 calibration status ranges 0-3 per sensor (system, gyro, accel, mag). "
                   "Read via the CALIB_STAT register (0x35); 3 = fully calibrated.",
    "i2c address": "Default I2C address is 0x28 (0x29 if the ADR pin is pulled high).",
    "output modes": "Supports NDOF fusion mode (0x0C) for absolute orientation using accel+gyro+mag.",
}

TB6612FNG_NOTES = {
    "pinout": "AIN1/AIN2/PWMA control Motor A; BIN1/BIN2/PWMB control Motor B; STBY must be high to enable outputs.",
    "voltage": "VM (motor supply) up to 15V; VCC (logic) 2.7-5.5V; logic pins are 3.3V-safe for the Pico.",
    "direction control": "Set AIN1=1, AIN2=0 for forward; AIN1=0, AIN2=1 for reverse; PWM pin sets speed.",
}


@mcp.tool()
def search_bno055_docs(query: str) -> str:
    """Search offline BNO055 IMU datasheet notes for a keyword."""
    query_lower = query.lower()
    for key, note in BNO055_NOTES.items():
        if key in query_lower or query_lower in key:
            return note
    return f"No BNO055 note found for '{query}'. Available topics: {', '.join(BNO055_NOTES)}"


@mcp.tool()
def search_tb6612fng_docs(query: str) -> str:
    """Search offline TB6612FNG motor driver datasheet notes for a keyword."""
    query_lower = query.lower()
    for key, note in TB6612FNG_NOTES.items():
        if key in query_lower or query_lower in key:
            return note
    return f"No TB6612FNG note found for '{query}'. Available topics: {', '.join(TB6612FNG_NOTES)}"


if __name__ == "__main__":
    mcp.run(transport="stdio")