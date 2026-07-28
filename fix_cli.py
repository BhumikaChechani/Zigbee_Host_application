import re

with open("src/cli.c", "r") as f:
    content = f.read()

# Helper function to generate replacement
def gen_replacement(command, code_blocks):
    return f"""            bool found = false;
{code_blocks}
            if (!found) {{
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {{
                    printf("Error: Address 0x%04X is not registered in the network.\\n", addr);
                }} else {{
                    printf("Error: The device at 0x%04X (%s) does not support the '{command}' command.\\n", addr, devName);
                }}
            }}"""

replacements = [
    (
        r"""#if ENABLE_AQARA_OCCUPANCY\s*AqaraOccupancy_SetZone\( addr, zoneIdx, minCm, maxCm \);\s*#endif""",
        gen_replacement("zone", """#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetZone( addr, zoneIdx, minCm, maxCm );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_AQARA_OCCUPANCY\s*AqaraOccupancy_DeleteZone\( addr, zoneIdx \);\s*#endif""",
        gen_replacement("zonedel", """#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_DeleteZone( addr, zoneIdx );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_AQARA_OCCUPANCY\s*if \(AqaraOccupancy_IsKnown\(addr\)\) \{\s*AqaraOccupancy_SetSensitivity\( addr, level \);\s*\}\s*#endif\s*#if ENABLE_VIBRATION_SENSOR\s*if \(VibrationSensor_IsKnown\(addr\)\) \{\s*VibrationSensor_SetSensitivity\( addr, level \);\s*\}\s*#endif""",
        gen_replacement("sensitivity", """#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetSensitivity( addr, level );
                found = true;
            }
#endif
#if ENABLE_VIBRATION_SENSOR
            if (VibrationSensor_IsKnown(addr)) {
                VibrationSensor_SetSensitivity( addr, level );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_AQARA_OCCUPANCY\s*AqaraOccupancy_SpatialLearning\( addr \);\s*#endif""",
        gen_replacement("spatiallearn", """#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SpatialLearning( addr );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_AQARA_OCCUPANCY\s*AqaraOccupancy_PostAssign\( addr \);\s*#endif""",
        gen_replacement("forcesetup", """#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_PostAssign( addr );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_AQARA_OCCUPANCY\s*AqaraOccupancy_SetLightThreshold\( addr, threshold \);\s*#endif""",
        gen_replacement("lightthreshold", """#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetLightThreshold( addr, threshold );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_ONICS_BUTTON\s*if \( OnicsButton_IsKnown\( addr \) \)\s*\{\s*printf\( "Writing ButtonPressActionDelay.*?\\n", delayMs, addr \);\s*uint8_t payloadFull\[8\];\s*static uint8_t s_cliSeq = 0;\s*payloadFull\[0\] = 0x00;\s*payloadFull\[1\] = s_cliSeq\+\+;\s*payloadFull\[2\] = 0x02;\s*payloadFull\[3\] = 0x01;\s*payloadFull\[4\] = 0x80;\s*payloadFull\[5\] = 0x21; // Uint16\s*payloadFull\[6\] = delayMs & 0xFF;\s*payloadFull\[7\] = \(delayMs >> 8\) & 0xFF;\s*ZNP_AfDataRequestExt\( 2, addr, 0x20, 0, 8, 0x0006, 0, 0, 15, payloadFull, 8 \);\s*\}\s*else\s*\{\s*printf\( "Error: 0x%04X is not a known Onics Button.\\n", addr \);\s*\}\s*#endif""",
        gen_replacement("onicsdelay", """#if ENABLE_ONICS_BUTTON
            if ( OnicsButton_IsKnown( addr ) )
            {
                printf( "Writing ButtonPressActionDelay (0x8001) = %d ms to Onics 0x%04X...\\n", delayMs, addr );
                uint8_t payloadFull[8];
                static uint8_t s_cliSeq = 0;
                payloadFull[0] = 0x00;
                payloadFull[1] = s_cliSeq++;
                payloadFull[2] = 0x02;
                payloadFull[3] = 0x01;
                payloadFull[4] = 0x80;
                payloadFull[5] = 0x21; // Uint16
                payloadFull[6] = delayMs & 0xFF;
                payloadFull[7] = (delayMs >> 8) & 0xFF;

                ZNP_AfDataRequestExt( 2, addr, 0x20, 0, 8, 0x0006, 0, 0, 15, payloadFull, 8 );
                found = true;
            }
#endif""")
    ),
    (
        r"""#if ENABLE_SIREN\s*if \( Siren_IsKnown\( addr \) \)\s*\{\s*Siren_Remove\( addr \);\s*\}\s*else\s*\{\s*printf\( "Siren 0x%04X is not registered.\\n", addr \);\s*\}\s*#endif""",
        gen_replacement("siren remove", """#if ENABLE_SIREN
            if ( Siren_IsKnown( addr ) )
            {
                Siren_Remove( addr );
                found = true;
            }
#endif""")
    )
]

for old, new in replacements:
    content = re.sub(old, new, content, flags=re.DOTALL)

with open("src/cli.c", "w") as f:
    f.write(content)
